/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( O ) ( L ) ( A ) ( V )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +                  OLAV: Off-Road Light Autonomous Vehicle                  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <olav_control/drive_by_wire/node.hpp>

namespace OLAV {
namespace ROS {

DriveByWireNode::DriveByWireNode() : rclcpp::Node("drive_by_wire") {
    Configure();
    Activate();
}

void DriveByWireNode::Configure() {
    GetParameters();
    Initialize();
}

void DriveByWireNode::GetParameters() {
    declare_parameter("connection.address", "192.168.69.3");
    connection_address_ = get_parameter("connection.address").as_string();

    declare_parameter("connection.port", 502);
    connection_port_ = get_parameter("connection.port").as_int();

    declare_parameter("rates.connect", 1.0);
    get_parameter("rates.connect").as_double();
    connect_period_ = 1.0 / get_parameter("rates.connect").as_double();

    declare_parameter("rates.writer", 100.0);
    writer_period_ = 1.0 / get_parameter("rates.writer").as_double();

    declare_parameter("rates.reader", 100.0);
    reader_period_ = 1.0 / get_parameter("rates.reader").as_double();

    declare_parameter("rates.health_check", 1.0);
    health_check_period_ =
        1.0 / get_parameter("rates.health_check").as_double();

    declare_parameter("rates.debug", 1.0);
    debug_period_ = 1.0 / get_parameter("rates.debug").as_double();

    declare_parameter("starter.duration", 2.0);
    engine_starter_duration_ = get_parameter("starter.duration").as_double();

    // FIXME: Check that the provided value is positive.
    declare_parameter("starter.check.window", 10);
    engine_speed_window_size_ =
        uint(get_parameter("starter.check.window").as_int());

    declare_parameter("starter.check.duration", 2.0);
    engine_speed_check_window_ =
        get_parameter("starter.check.duration").as_double();

    declare_parameter("starter.check.threshold", 1000.0);
    engine_speed_threshold_ =
        get_parameter("starter.check.threshold").as_double();

    declare_parameter("safety.limits.engine_speed", 5000.0);
    maximum_engine_speed_ =
        get_parameter("safety.limits.engine_speed").as_double();

    declare_parameter("safety.limits.longitudinal_speed", 8.33);
    maximum_longitudinal_speed_ =
        get_parameter("safety.limits.longitudinal_speed").as_double();

    declare_parameter("controllers.rate", 1.0);
    controllers_period_ = 1.0 / get_parameter("controllers.rate").as_double();

    declare_parameter("debug.use_mock_interface", false);
    use_mock_interface_ = get_parameter("debug.use_mock_interface").as_bool();

    GetSpeedControllerParameters();
    GetSteeringControllerParameters();

    // Add the "on set parameters" event callback for runtime parameter
    // reconfiguration.
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DriveByWireNode::OnSetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void DriveByWireNode::Initialize() {
    // Initialize the callback groups for the node subscriptions and timers.
    // Note that operations that can be executed asynchronously are delegated to
    // separate callback groups, but each individual callback group may only
    // spawn one thread (e.g. only one service can be active at any one time),
    // with the exception of the control commands subscriptions which are
    // processed entirely asynchronously.
    services_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscriptions_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    writer_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    controllers_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reader_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    debug_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize the drive-by-wire setpoint and feedback structures. These are
    // assumed to be null until the drive-by-wire is connected and the initial
    // values are written to the PLC registers and a first set of feedback
    // values is retrieved.
    drive_by_wire_setpoint_ = std::make_shared<DriveByWireSetpoint>();
    drive_by_wire_feedback_ = std::make_shared<DriveByWireFeedback>();

    // Initialize the drive-by-wire interface and optionally attempt to connect.
    interface_ = std::make_shared<DriveByWireInterface>(connection_address_,
                                                        connection_port_);

    InitializeSpeedController();
    InitializeSteeringController();

    // Initialize the time references for the health checks.
    last_engine_speed_time_ =
        get_clock()->now() - rclcpp::Duration(health_check_period_, 0);
    last_odometry_time_ =
        get_clock()->now() - rclcpp::Duration(health_check_period_, 0);
    last_heartbeat_time_ =
        get_clock()->now() - rclcpp::Duration(health_check_period_, 0);
    last_control_time_ =
        get_clock()->now() - rclcpp::Duration(health_check_period_, 0);
}

void DriveByWireNode::Activate() {
    CreateSubscriptions();
    CreateServices();
    CreateTimers();
    CreatePublishers();
    StartTimers();
}

void DriveByWireNode::CreateSubscriptions() {
    // Instantiate a common set of subscription options to group all command and
    // health check subscriptions under the same re-entrant callback group.
    subscription_options_.callback_group = subscriptions_callback_group_;

    engine_speed_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "feedback/engine/speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&DriveByWireNode::EngineSpeedCallback,
                      this,
                      std::placeholders::_1),
            subscription_options_);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "feedback/odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&DriveByWireNode::OdometryCallback,
                  this,
                  std::placeholders::_1),
        subscription_options_);

    throttle_brake_steering_subscription_ =
        create_subscription<olav_interfaces::msg::ThrottleBrakeSteering>(
            "controls/tbs",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&DriveByWireNode::ThrottleBrakeSteeringCallback,
                      this,
                      std::placeholders::_1),
            subscription_options_);
}

void DriveByWireNode::CreateHeartbeatSubscription() {
    heartbeat_subscription_ = create_subscription<std_msgs::msg::Header>(
        "signals/heartbeat",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&DriveByWireNode::HeartbeatCallback,
                  this,
                  std::placeholders::_1),
        subscription_options_);
}

void DriveByWireNode::DestroyHeartbeatSubscription() {
    heartbeat_subscription_.reset();
}

void DriveByWireNode::CreatePublishers() {
    ready_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "signals/drive_by_wire",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    emergency_stop_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "signals/estop",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_angle_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "sensors/steering/angle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "model/joints/steering",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    plc_status_publisher_ =
        create_publisher<olav_interfaces::msg::DriveByWirePLCStatus>(
            "status",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    speed_controller_status_publisher_ =
        create_publisher<olav_interfaces::msg::PIDStatus>(
            "status/controllers/speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_controller_status_publisher_ =
        create_publisher<olav_interfaces::msg::PIDStatus>(
            "status/controllers/steering",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void DriveByWireNode::CreateTimers() {
    connect_timer_ =
        create_wall_timer(std::chrono::duration<double>(connect_period_),
                          std::bind(&DriveByWireNode::ConnectCallback, this));
    connect_timer_->cancel();

    health_check_timer_ = create_wall_timer(
        std::chrono::duration<double>(health_check_period_),
        std::bind(&DriveByWireNode::HealthCheckCallback, this));
    health_check_timer_->cancel();

    writer_timer_ =
        create_wall_timer(std::chrono::duration<double>(writer_period_),
                          std::bind(&DriveByWireNode::WriterCallback, this),
                          writer_callback_group_);
    writer_timer_->cancel();

    reader_timer_ =
        create_wall_timer(std::chrono::duration<double>(reader_period_),
                          std::bind(&DriveByWireNode::ReaderCallback, this),
                          reader_callback_group_);
    reader_timer_->cancel();

    controllers_timer_ = create_wall_timer(
        std::chrono::duration<double>(controllers_period_),
        std::bind(&DriveByWireNode::ControllersCallback, this),
        controllers_callback_group_);
    controllers_timer_->cancel();

    debug_timer_ =
        create_wall_timer(std::chrono::duration<double>(debug_period_),
                          std::bind(&DriveByWireNode::DebugCallback, this),
                          debug_callback_group_);
    debug_timer_->cancel();

    diagnostics_timer_ = create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&DriveByWireNode::DiagnosticsCallback, this),
        debug_callback_group_);
    diagnostics_timer_->cancel();
}

void DriveByWireNode::CreateServices() {
    ready_service_ = create_service<std_srvs::srv::Trigger>(
        "ready",
        std::bind(&DriveByWireNode::Ready,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    set_control_mode_service_ =
        create_service<olav_interfaces::srv::SetControlMode>(
            "set_control_mode",
            std::bind(&DriveByWireNode::SetControlMode,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

    set_ignition_service_ = create_service<std_srvs::srv::SetBool>(
        "set_ignition",
        std::bind(&DriveByWireNode::SetIgnition,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    cycle_ignition_service_ = create_service<std_srvs::srv::Trigger>(
        "cycle_ignition",
        std::bind(&DriveByWireNode::CycleIgnition,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    start_engine_service_ = create_service<std_srvs::srv::Trigger>(
        "start_engine",
        std::bind(&DriveByWireNode::StartEngine,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_default,
        services_callback_group_);

    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
        "emergency_stop",
        std::bind(&DriveByWireNode::EmergencyStop,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_default,
        services_callback_group_);

    set_steering_pid_gains_service_ =
        create_service<olav_interfaces::srv::SetPIDGains>(
            "set_steering_pid_gains",
            std::bind(&DriveByWireNode::SetSteeringPIDGains,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_default,
            services_callback_group_);

    shift_gear_up_service_ = create_service<std_srvs::srv::Trigger>(
        "shift_gear_up",
        std::bind(&DriveByWireNode::ShiftGearUp,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    shift_gear_down_service_ = create_service<std_srvs::srv::Trigger>(
        "shift_gear_down",
        std::bind(&DriveByWireNode::ShiftGearDown,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
}

void DriveByWireNode::StartTimers() {
    RCLCPP_INFO(get_logger(), "Starting timers ...");

    connect_timer_->reset();
    health_check_timer_->reset();
    writer_timer_->reset();
    reader_timer_->reset();
    controllers_timer_->reset();
    debug_timer_->reset();
    diagnostics_timer_->reset();
}

void DriveByWireNode::InitializeRegisters() {
    RCLCPP_INFO(get_logger(), "Writing initial state to PLC ...");

    {
        std::lock_guard<std::mutex> modbus_lock(modbus_mutex_);
        interface_->Write(0.0, // Steering
                          1.0, // Brake,
                          0.0, // Throttle,
                          false, // Ignition
                          false, // Emergency stop
                          false, // Engine starter
                          DriveByWireInterface::GearPosition::PARK // Gear
        );
    }
}

void DriveByWireNode::WriterCallback() {
    if(!is_connected_) return;
    if(use_mock_interface_) return;

    DriveByWireSetpoint setpoint;

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        setpoint = *drive_by_wire_setpoint_;
    }

    // Lock the modbus mutex and refresh the PLC register values to reflect
    // the current setpoint.
    {
        std::lock_guard<std::mutex> modbus_lock(modbus_mutex_);

        try {
            interface_->Write(setpoint);
        } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
            RCLCPP_ERROR(get_logger(),
                         "Could not write setpoints, the drive-by-wire "
                         "interface raised the following exception: %s",
                         exception.what());
            Disconnect();
        }
    }
}

void DriveByWireNode::ReaderCallback() {
    if(!is_connected_) return;
    if(use_mock_interface_) return;

    DriveByWireFeedback feedback;

    {
        std::lock_guard<std::mutex> modbus_lock(modbus_mutex_);

        try {
            feedback = interface_->Read();
        } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
            RCLCPP_ERROR(get_logger(),
                         "Could not read feedback, the drive-by-wire "
                         "interface raised the following exception: %s",
                         exception.what());
            Disconnect();
        }
    }

    // Update the drive-by-wire feedback structure with the current feedback
    // values and publish the steering angle.
    {
        std::unique_lock<std::shared_mutex> feedback_lock(feedback_mutex_);

        drive_by_wire_feedback_ =
            std::make_shared<DriveByWireFeedback>(feedback);

        // TODO: Delegate this to a separate thread.
        PublishSteeringAngle();

        // TODO: Delegate this to a separate thread.
        auto emergency_stop_message = std::make_shared<std_msgs::msg::Bool>();
        emergency_stop_message->data = emergency_stop_;
        emergency_stop_publisher_->publish(*emergency_stop_message);

        // TODO: Delegate this to a separate thread.
        auto ready_message = std::make_shared<std_msgs::msg::Bool>();
        ready_message->data = feedback.IsAutonomousModeOn() && is_ready_;
        ready_publisher_->publish(*ready_message);
    }
}

void DriveByWireNode::ControllersCallback() {
    if(!IsHealthy()) {
        speed_controller_->Reset();
        steering_controller_->Reset();
        return;
    }

    /* TODO: Implement the speed controller feedforward offset.
    // Compute the current feedforward value to be fed to the controller.
    controller_->SetFeedforwardOffset(feedforward_offset);
    double feedforward_offset =
        feedforward_spline_->Evaluate(current_setpoint_);
    */

    double steering_angle;
    {
        std::unique_lock<std::shared_mutex> feedback_lock(feedback_mutex_);
        steering_angle =
            drive_by_wire_feedback_->GetSteeringActuatorPositionInDegrees();
    }

    double speed_controller_output;
    double steering_controller_output;
    {
        const std::lock_guard<std::mutex> controller_lock(controllers_mutex_);
        // TODO: The vehicle speed and odometry needs its own mutex.
        speed_controller_->SetFeedback(vehicle_speed_);
        speed_controller_->Tick();
        speed_controller_output = speed_controller_->GetOutput();

        steering_controller_->SetFeedback(steering_angle);
        steering_controller_->Tick();
        steering_controller_output = steering_controller_->GetOutput();
    }

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);

        const double throttle_effort =
            speed_controller_output > 0.0 ? speed_controller_output : 0.0;
        const double brake_effort = speed_controller_output < 0.0
            ? std::min(brake_threshold_, std::abs(speed_controller_output))
            // TODO: This should instead be set as a minimum output of the speed
            // controller at -brake_threshold! This check is still useful just
            // in case ...
            : 0.0;

        drive_by_wire_setpoint_->SetThrottle(throttle_effort);
        drive_by_wire_setpoint_->SetBrake(brake_effort);
        drive_by_wire_setpoint_->SetSteering(steering_controller_output);
    }

    /* TODO: This goes in the DEBUG publisher callback.
    if(publish_status_) {
        const std::lock_guard<std::mutex> controller_lock(controllers_mutex_);
        olav_interfaces::msg::PIDStatus pid_status_message;
        pid_status_message.setpoint = controller_->GetSetpoint();
        pid_status_message.feedback = controller_->GetFeedback();
        pid_status_message.output = controller_->GetOutput();
        pid_status_message.feedforward_term = controller_->GetFeedforwardTerm();
        pid_status_message.proportional_term =
            controller_->GetProportionalTerm();
        pid_status_message.integral_term = controller_->GetIntegralTerm();
        pid_status_message.derivative_term = controller_->GetDerivativeTerm();
        status_publisher_->publish(pid_status_message);
    }
    */
}

void DriveByWireNode::DebugCallback() {
    if(!is_connected_) return;

    PublishPLCStatus();
    PublishSpeedControllerStatus();
    PublishSteeringControllerStatus();
}

void DriveByWireNode::DiagnosticsCallback() {
    auto diagnostic_array_message =
        std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_array_message->header.stamp = get_clock()->now(),
    GetDiagnostics(diagnostic_array_message);
    diagnostics_publisher_->publish(*diagnostic_array_message);
}

std::shared_ptr<olav_interfaces::msg::PIDStatus>
DriveByWireNode::GetControllerStatusMessage(
    std::shared_ptr<PIDController> controller) {
    auto message = std::make_shared<olav_interfaces::msg::PIDStatus>();

    message->setpoint = controller->GetSetpoint();
    message->feedback = controller->GetFeedback();
    message->output = controller->GetOutput();
    message->feedforward_term = controller->GetFeedforwardTerm();
    message->proportional_term = controller->GetProportionalTerm();
    message->integral_term = controller->GetIntegralTerm();
    message->derivative_term = controller->GetDerivativeTerm();

    return message;
}

void DriveByWireNode::PublishSpeedControllerStatus() {
    speed_controller_status_publisher_->publish(
        *GetControllerStatusMessage(speed_controller_));
}

void DriveByWireNode::PublishSteeringControllerStatus() {
    steering_controller_status_publisher_->publish(
        *GetControllerStatusMessage(steering_controller_));
}

void DriveByWireNode::PublishPLCStatus() {
    auto plc_status_message =
        std::make_shared<olav_interfaces::msg::DriveByWirePLCStatus>();

    // Populate the message header.
    plc_status_message->header.frame_id = "plc";
    plc_status_message->header.stamp = get_clock()->now();

    // Populate the status message fields.
    plc_status_message->setpoint_throttle =
        drive_by_wire_setpoint_->GetThrottle();
    plc_status_message->setpoint_brake = drive_by_wire_setpoint_->GetBrake();
    plc_status_message->setpoint_steering =
        drive_by_wire_setpoint_->GetSteering();
    plc_status_message->setpoint_gear = drive_by_wire_setpoint_->GetGear();
    plc_status_message->setpoint_ignition_on =
        drive_by_wire_setpoint_->GetIgnition();
    plc_status_message->setpoint_ignition_off =
        !drive_by_wire_setpoint_->GetIgnition();
    plc_status_message->setpoint_engine_starter =
        drive_by_wire_setpoint_->GetEngineStarter();
    plc_status_message->setpoint_emergency_stop =
        drive_by_wire_setpoint_->GetEmergencyStop();
    plc_status_message->steering_actuator_position_raw =
        drive_by_wire_feedback_->GetSteeringActuatorPosition();
    plc_status_message->steering_wheel_angle_degrees =
        drive_by_wire_feedback_->GetSteeringActuatorPositionInDegrees();
    plc_status_message->steering_wheel_angle_radians =
        drive_by_wire_feedback_->GetSteeringActuatorPositionInRadians();
    plc_status_message->selected_gear =
        drive_by_wire_feedback_->GetSelectedGear();
    plc_status_message->gear_actuator_position =
        drive_by_wire_feedback_->GetGearActuatorPosition();
    plc_status_message->brake_actuator_position =
        drive_by_wire_feedback_->GetBrakeActuatorPosition();
    plc_status_message->is_ignition_on =
        drive_by_wire_feedback_->IsIgnitionOn();
    plc_status_message->is_autonomous_mode_on =
        drive_by_wire_feedback_->IsAutonomousModeOn();
    plc_status_message->is_gear_actuator_in_position =
        drive_by_wire_feedback_->IsGearActuatorInPosition();

    // Publish the status message.
    plc_status_publisher_->publish(*plc_status_message);
}

void DriveByWireNode::PublishSteeringAngle() {
    auto stamp = get_clock()->now();
    auto steering_angle =
        drive_by_wire_feedback_->GetSteeringActuatorPositionInRadians();

    auto steering_angle_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    steering_angle_message->header.stamp = stamp;
    steering_angle_message->header.frame_id = "drive-by-wire";
    steering_angle_message->setpoint = steering_angle;
    steering_angle_publisher_->publish(*steering_angle_message);

    auto joint_state_message = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_message->header.stamp = stamp;
    joint_state_message->header.frame_id = "base_link";
    if(steering_angle == 0) {
        joint_state_message->effort.push_back(0.0);
        joint_state_message->effort.push_back(0.0);
    } else if(steering_angle > 0) {
        joint_state_message->effort.push_back(-1.0);
        joint_state_message->effort.push_back(-1.0);
    } else {
        joint_state_message->effort.push_back(1.0);
        joint_state_message->effort.push_back(1.0);
    }
    joint_state_message->name.push_back(
        "front_axle_to_front_right_wheel_revolute");
    joint_state_message->name.push_back(
        "front_axle_to_front_left_wheel_revolute");
    joint_state_message->position.push_back(-steering_angle);
    joint_state_message->position.push_back(-steering_angle);
    joint_state_publisher_->publish(*joint_state_message);
}

void DriveByWireNode::ConnectCallback() {
    if(is_connected_) return;

    if(use_mock_interface_) {
        is_connected_ = true;
        RCLCPP_INFO(get_logger(), "CONNECTED");
        return;
    }

    try {
        interface_->Open();
    } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
        return;
    }

    is_connected_ = true;
    RCLCPP_INFO(get_logger(),
                "Successfully connected to the drive-by-wire Modbus TCP "
                "server on << %s:%i >>!",
                connection_address_.c_str(),
                connection_port_);
}

void DriveByWireNode::Disconnect() {
    if(use_mock_interface_) {
        is_connected_ = false;
        is_ready_ = false;
        RCLCPP_INFO(get_logger(),
                    "Successfully disconnected from the drive-by-wire "
                    "mock interface!");
    }

    if(!interface_->IsConnected()) {
        is_connected_ = false;
        is_ready_ = false;
    } else {
        try {
            interface_->Close();
            is_connected_ = false;
            is_ready_ = false;
            RCLCPP_INFO(get_logger(),
                        "Successfully disconnected from the drive-by-wire "
                        "Modbus TCP server!");
        } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
            RCLCPP_ERROR(get_logger(),
                         "Could not disconnect from the drive-by-wire Modbus "
                         "TCP server!");
        }
    }
}

void DriveByWireNode::HealthCheckCallback() {
    if(use_mock_interface_) {
        has_engine_speed_ = true;
        engine_speed_ = 3000.0;
        has_odometry_ = true;
        vehicle_speed_ = 0.0;
        return;
    };

    auto current_time = get_clock()->now();

    if(is_connected_ && !interface_->IsConnected()) {
        // TODO: This needs a mutex.
        is_connected_ = false;
        RCLCPP_ERROR(get_logger(),
                     "Lost connection to the drive-by-wire Modbus TCP server!");
    };

    has_engine_speed_ = (current_time - last_engine_speed_time_).seconds() >=
            health_check_period_
        ? false
        : true;

    has_odometry_ =
        (current_time - last_odometry_time_).seconds() >= health_check_period_
        ? false
        : true;

    if(active_control_mode_.GetModeIdentifier() ==
           ControlModeIdentifier::DRIVE_ACKERMANN &&
       active_control_mode_.GetAuthorityIdentifier() ==
           ControlAuthorityIdentifier::AUTONOMY)
        has_heartbeat_ = (current_time - last_heartbeat_time_).seconds() >=
                health_check_period_
            ? false
            : true;

    if(!HasValidControl(current_time)) {
        {
            std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);

            drive_by_wire_setpoint_->SetThrottle(0.0);
            drive_by_wire_setpoint_->SetBrake(0.0);
            drive_by_wire_setpoint_->SetSteering(0.0);

            active_control_mode_ = ControlMode();
        }
    }
}

bool DriveByWireNode::IsHealthy() {
    return is_connected_ && has_engine_speed_ && has_odometry_ && is_ready_;
}

void DriveByWireNode::ThrottleBrakeSteeringCallback(
    olav_interfaces::msg::ThrottleBrakeSteering::ConstSharedPtr message) {
    if(!IsHealthy()) { return; }

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);

        drive_by_wire_setpoint_->SetBrake(message->brake);
        drive_by_wire_setpoint_->SetThrottle(message->throttle);
    }

    // TODO: Add a check on the maximum steering angle to ensure it is
    // within bounds.
    {
        std::unique_lock<std::mutex> controllers_lock(controllers_mutex_);

        steering_controller_->SetSetpoint(message->steering *
                                          maximum_steering_angle_);
    }
}

void DriveByWireNode::EngineSpeedCallback(
    const olav_interfaces::msg::SetpointStamped::ConstSharedPtr
        engine_speed_message) {
    // Update the received engine speed magnitude.
    engine_speed_ = engine_speed_message->setpoint;
    has_engine_speed_ = true;
    last_engine_speed_time_ = get_clock()->now();

    // Check if the engine speed exceeds the maximum allowed value.
    if(engine_speed_message->setpoint > maximum_engine_speed_) {
        RCLCPP_ERROR(get_logger(),
                     "Maximum allowed engine speed exceeded: %f > %f",
                     engine_speed_message->setpoint,
                     maximum_engine_speed_);
        TriggerEmergencyStop();
    }
}

void DriveByWireNode::HeartbeatCallback(
    const std_msgs::msg::Header::ConstSharedPtr heartbeat_message) {
    (void)heartbeat_message;

    // Update the last received heartbeat timestamp.
    has_heartbeat_ = true;
    last_heartbeat_time_ = get_clock()->now();
}

void DriveByWireNode::OdometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odometry_message) {
    // Update the received vehicle speed magnitude.
    vehicle_speed_ = odometry_message->twist.twist.linear.x;
    has_odometry_ = true;
    last_odometry_time_ = get_clock()->now();

    // Check if the wheel speed exceeds the maximum allowed value.
    if(odometry_message->twist.twist.linear.x > maximum_longitudinal_speed_) {
        RCLCPP_ERROR(get_logger(),
                     "Maximum allowed vehicle speed exceeded: %0.2f > %0.2f",
                     odometry_message->twist.twist.linear.x,
                     maximum_longitudinal_speed_);

        TriggerEmergencyStop();
    }
}

void DriveByWireNode::CycleIgnition(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    // Check that the drive-by-wire and all auxiliary systems are ready before
    // cycling ignition.
    if(!is_ready_) {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "The drive-by-wire is not ready to receive a cycle ignition "
            "command!");
        return;
    };

    // Read the current ignition state from the PLC.
    bool ignition_state = false;
    {
        std::shared_lock<std::shared_mutex> feedback_lock(feedback_mutex_);
        ignition_state = drive_by_wire_feedback_->IsIgnitionOn();
    }

    // Change the state of the ignition to its complement.
    try {
        const std::string current_ignition_state =
            ignition_state ? "ON" : "OFF";
        const std::string target_ignition_state =
            !ignition_state ? "ON" : "OFF";

        RCLCPP_INFO(get_logger(),
                    "Changing ignition state from [%s] to [%s] ...",
                    current_ignition_state.c_str(),
                    target_ignition_state.c_str());

        WriteIgnitionState(!ignition_state);

        SetResponseSuccessInfo<std_srvs::srv::Trigger::Response>(
            response,
            "Ignition state set to " + target_ignition_state + ".");
    } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "Could not set ignition state: " + std::string(exception.what()));
    }
}

void DriveByWireNode::WriteIgnitionState(const bool& ignition_state) {
    std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
    drive_by_wire_setpoint_->SetIgnition(ignition_state);
}

void DriveByWireNode::SetIgnition(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    try {
        {
            std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
            drive_by_wire_setpoint_->SetIgnition(request->data);
        }

        response->success = true;
        response->message = "Updated ignition state.";
    } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
        RCLCPP_ERROR(get_logger(), exception.what());
        response->success = false;
        response->message = exception.what();
    }
}

void DriveByWireNode::StartEngine(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    if(!has_engine_speed_) {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "Engine speed not available.");
        return;
    }

    // Turn on the engine starter.
    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        drive_by_wire_setpoint_->StartEngine(true);
    }

    // Let the engine starter run for the defined time, notify the user every
    // half second.
    auto initial_time = get_clock()->now();
    rclcpp::Duration elapsed_time(0, 0);

    while(elapsed_time.seconds() < engine_starter_duration_) {
        elapsed_time = (get_clock()->now() - initial_time);
    }

    // Turn off the engine starter.
    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        drive_by_wire_setpoint_->StartEngine(false);
    }

    // Check engine speed.
    boost::accumulators::accumulator_set<
        double,
        boost::accumulators::stats<boost::accumulators::tag::rolling_mean>>
        accumulator(boost::accumulators::tag::rolling_window::window_size =
                        engine_speed_window_size_);

    initial_time = get_clock()->now();
    elapsed_time = rclcpp::Duration(0, 0);
    while(elapsed_time.seconds() < engine_speed_check_window_) {
        elapsed_time = (get_clock()->now() - initial_time);
        accumulator(engine_speed_);
    }

    // Compute the mean engine speed over the specified period.
    auto mean_engine_speed = boost::accumulators::rolling_mean(accumulator);

    if(mean_engine_speed > engine_speed_threshold_) {
        SetResponseSuccessInfo<std_srvs::srv::Trigger::Response>(
            response,
            "Engine started successfully, mean engine speed is " +
                std::to_string(mean_engine_speed));

    } else {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "Could not start engine.");
    }
}

void DriveByWireNode::SetSteeringPIDGains(
    const std::shared_ptr<olav_interfaces::srv::SetPIDGains::Request> request,
    std::shared_ptr<olav_interfaces::srv::SetPIDGains::Response> response) {
    // Convert the double precision values for the gains into a matching
    // 16-bit integer mapping 1/100 units in the range [0, 10000].
    auto proportional_gain =
        int16_t(boost::algorithm::clamp(request->proportional_gain / 100.0,
                                        0.0,
                                        100.0));
    auto integral_gain = int16_t(
        boost::algorithm::clamp(request->integral_gain / 100.0, 0.0, 100.0));
    auto derivative_gain = int16_t(
        boost::algorithm::clamp(request->derivative_gain / 100.0, 0.0, 100.0));

    try {
        {
            std::lock_guard<std::mutex> modbus_lock(modbus_mutex_);
            interface_->WriteSteeringPIDState(request->use_dynamic_gains,
                                              proportional_gain,
                                              integral_gain,
                                              derivative_gain);
        }

        response->success = true;
        response->message =
            "Set steering PID dynamic gains mode to \"%i\" and updated "
            "steering PID gains to (k_p: %0.4f, k_i: %0.4f, k_d: %0.4f)",
        request->proportional_gain, request->integral_gain,
        request->derivative_gain;
    } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
        RCLCPP_ERROR(get_logger(), exception.what());
        response->success = false;
        response->message = exception.what();
    }
}

void DriveByWireNode::EmergencyStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    RCLCPP_ERROR(get_logger(), ">>> EMERGENCY STOP TRIGGERED! <<<");

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        drive_by_wire_setpoint_->SetEmergencyStop(true);
    }

    std::string message("Emergency stop request processed.");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->message = message;
    response->success = true;

    // TODO: This needs to be an "is_in_emergency" variable.
    // is_ready_ = false;
}

void DriveByWireNode::Ready(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    if(is_connected_) {
        RCLCPP_INFO(get_logger(),
                    "All systems healthy, readying drive-by-wire...");
    } else {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "Could not ready drive-by-wire, aborting ...");
        return;
    }

    if(is_ready_) {
        SetResponseSuccessWarning<std_srvs::srv::Trigger::Response>(
            response,
            "Drive-by-wire interface already started and "
            "accepting commands!");
        return;
    } else {
        // Attempt to initialize the PLC registers to a set of default values.
        try {
            InitializeRegisters();
        } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
            SetResponseFailError<std_srvs::srv::Trigger::Response>(
                response,
                "Could not initialize the PLC registers!");
            return;
        }

        WriteIgnitionState(true);

        is_ready_ = true;
        writer_timer_->reset();
        SetResponseSuccessInfo<std_srvs::srv::Trigger::Response>(
            response,
            "Drive-by-wire interface started.");
    }
}

double DriveByWireNode::Smoothstep(double x, double edge0, double edge1) {
    // Scale, and clamp x to 0..1 range
    x = boost::algorithm::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);

    return x * x * (3.0f - 2.0f * x);
}

void DriveByWireNode::ShiftGearUp(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_INFO(get_logger(), "Received a gear upshift request!");

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);

        int gear = drive_by_wire_setpoint_->GetGear();

        if(gear == 5) {
            SetResponseFailError<std_srvs::srv::Trigger::Response>(
                response,
                "Could not shift gear up: the gear selector is in the highest "
                "possible position.");
        } else {
            drive_by_wire_setpoint_->SetGear(gear + 1);
            SetResponseSuccessInfo<std_srvs::srv::Trigger::Response>(
                response,
                "Gear upshift request completed successfully");
        }
    }
}

void DriveByWireNode::ShiftGearDown(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_INFO(get_logger(), "Received a gear downshift request!");

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);

        int gear = drive_by_wire_setpoint_->GetGear();

        if(gear == 1) {
            SetResponseFailError<std_srvs::srv::Trigger::Response>(
                response,
                "Could not shift gear down: the gear selector is in the lowest "
                "possible position.");
        } else {
            drive_by_wire_setpoint_->SetGear(gear - 1);
            SetResponseSuccessInfo<std_srvs::srv::Trigger::Response>(
                response,
                "Gear downshift request completed successfully.");
        }
    }
}

void DriveByWireNode::GetDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    GetConnectionDiagnostics(diagnostic_array_message);
    GetStateDiagnostics(diagnostic_array_message);
    GetEngineSpeedDiagnostics(diagnostic_array_message);
    GetOdometryDiagnostics(diagnostic_array_message);
    GetHeartbeatDiagnostics(diagnostic_array_message);
    GetReadyDiagnostics(diagnostic_array_message);
}

void DriveByWireNode::GetEngineSpeedDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    auto diagnostic_status =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_status->level = has_engine_speed_
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic_status->name = "olav/drive-by-wire/engine_speed";
    diagnostic_status->message = has_engine_speed_
        ? "System has an engine speed reading."
        : "The system is not receiving an engine speed. Please check the "
          "powertrain interface status and the microcontroller connection.";
    diagnostic_status->hardware_id = hardware_id_;
    diagnostic_array_message->status.push_back(*diagnostic_status);
}

void DriveByWireNode::GetReadyDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    auto diagnostic_status =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_status->level = is_ready_
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic_status->name = "olav/drive-by-wire/ready";
    diagnostic_status->message = is_ready_
        ? "System is armed and ready to receive commands."
        : "The system is not ready to receive commands. Please issue a ready "
          "acknowledgement through a control authority to arm the           "
          "system.";
    diagnostic_status->hardware_id = hardware_id_;
    diagnostic_array_message->status.push_back(*diagnostic_status);
}

void DriveByWireNode::SetControlMode(
    const std::shared_ptr<olav_interfaces::srv::SetControlMode::Request>
        request,
    std::shared_ptr<olav_interfaces::srv::SetControlMode::Response> response) {
    const auto target_control_mode = ControlMode(
        static_cast<ControlModeIdentifier>(request->mode),
        static_cast<ControlAuthorityIdentifier>(request->authority));

    RCLCPP_INFO(
        get_logger(),
        "Set control mode request received: [MODE: %s | AUTHORITY: %s] => "
        "[MODE: %s | AUTHORITY: %s]",
        active_control_mode_.GetModeName().c_str(),
        active_control_mode_.GetAuthorityName().c_str(),
        target_control_mode.GetModeName().c_str(),
        target_control_mode.GetAuthorityName().c_str());

    active_control_mode_ = target_control_mode;

    if(target_control_mode.GetModeIdentifier() ==
           ControlModeIdentifier::DRIVE_ACKERMANN &&
       target_control_mode.GetAuthorityIdentifier() ==
           ControlAuthorityIdentifier::AUTONOMY) {
        CreateHeartbeatSubscription();
    } else if(active_control_mode_.GetModeIdentifier() ==
                  ControlModeIdentifier::DRIVE_ACKERMANN &&
              active_control_mode_.GetAuthorityIdentifier() ==
                  ControlAuthorityIdentifier::AUTONOMY) {
        DestroyHeartbeatSubscription();
    }

    response->success = true;
    response->message = "Changed mode successfully";
}

void DriveByWireNode::GetHeartbeatDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    auto diagnostic_status =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_status->level = has_heartbeat_
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic_status->name = "olav/drive-by-wire/heartbeat";
    diagnostic_status->message =
        has_heartbeat_ ? "Hearbeat present." : "Heartbeat missing!";
    diagnostic_status->hardware_id = hardware_id_;
    diagnostic_array_message->status.push_back(*diagnostic_status);
}

void DriveByWireNode::GetConnectionDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    auto diagnostic_status =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_status->level = is_connected_
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic_status->name = "olav/drive-by-wire/connection";
    diagnostic_status->message =
        is_connected_ ? "Connected successfully!" : "Connection failed.";
    diagnostic_status->hardware_id = hardware_id_;
    diagnostic_array_message->status.push_back(*diagnostic_status);
}

void DriveByWireNode::GetStateDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    auto diagnostic_status =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_status->level = emergency_stop_
        ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
        : diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic_status->name = "olav/drive-by-wire/state";
    diagnostic_status->message = emergency_stop_
        ? "The system is in emergency state."
        : "The system is in a valid state..";
    diagnostic_status->hardware_id = hardware_id_;

    diagnostic_msgs::msg::KeyValue mode_item;
    mode_item.key = "state";
    mode_item.value = active_control_mode_.GetModeName();
    diagnostic_status->values.push_back(mode_item);

    diagnostic_msgs::msg::KeyValue authority_item;
    authority_item.key = "authority";
    authority_item.value = active_control_mode_.GetAuthorityName();
    diagnostic_status->values.push_back(authority_item);

    diagnostic_array_message->status.push_back(*diagnostic_status);
}

void DriveByWireNode::GetOdometryDiagnostics(
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_array_message) {
    auto diagnostic_status =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_status->level = has_odometry_
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic_status->name = "olav/drive-by-wire/odometry";
    diagnostic_status->message =
        has_odometry_ ? "Odometry is present." : "Missing odometry!";
    diagnostic_status->hardware_id = hardware_id_;
    diagnostic_array_message->status.push_back(*diagnostic_status);
}

void DriveByWireNode::TriggerEmergencyStop() {}

void DriveByWireNode::GetSpeedControllerParameters() {
    declare_parameter("controllers.speed.rate", 100.0);

    declare_parameter("controllers.speed.pid.gains.feedforward", 0.0);

    declare_parameter("controllers.speed.pid.gains.proportional", 1.0);

    declare_parameter("controllers.speed.pid.gains.integral", 0.1);

    declare_parameter("controllers.speed.pid.gains.derivative", 0.0);

    declare_parameter("controllers.speed.pid.setpoint.ramp.enabled", true);

    declare_parameter("controllers.speed.pid.setpoint.ramp.magnitude", 0.001);

    declare_parameter("controllers.speed.pid.output.change.enabled", true);

    declare_parameter("controllers.speed.pid.output.change.magnitude", 0.001);

    declare_parameter("controllers.speed.pid.limit.integral.enabled", true);

    declare_parameter("controllers.speed.pid.limit.integral.magnitude", 40.0);

    declare_parameter("controllers.speed.pid.feedforward.offset.positive", 1.0);

    declare_parameter("controllers.speed.pid.feedforward.offset.negative", 1.0);

    declare_parameter("controllers.speed.pid.deadband.filter.enabled", true);

    declare_parameter("controllers.speed.pid.deadband.filter.thresholds.lower",
                      -0.18);

    declare_parameter("controllers.speed.pid.deadband.filter.thresholds.upper",
                      0.3);

    declare_parameter("controllers.speed.brake.enabled", false);

    declare_parameter("controllers.speed.brake.threshold", 0.3);

    declare_parameter("controllers.speed.pid.feedforward.curve.knots",
                      std::vector<double>{0.0, 0.0, 0.0, 0.0});

    declare_parameter("controllers.speed.pid.feedforward.curve.values",
                      std::vector<double>{0.0, 0.0, 0.0, 0.0});

    declare_parameter("controllers.speed.pid.feedforward.curve.degree", 3);
}

void DriveByWireNode::InitializeSpeedController() {
    speed_controller_ = std::make_shared<PIDController>();

    speed_controller_->SetFeedforwardGain(
        get_parameter("controllers.speed.pid.gains.feedforward").as_double());

    speed_controller_->SetProportionalGain(
        get_parameter("controllers.speed.pid.gains.proportional").as_double());

    speed_controller_->SetIntegralGain(
        get_parameter("controllers.speed.pid.gains.integral").as_double());

    speed_controller_->SetDerivativeGain(
        get_parameter("controllers.speed.pid.gains.derivative").as_double());

    speed_controller_->UseSetpointRamping(
        get_parameter("controllers.speed.pid.setpoint.ramp.enabled").as_bool());

    speed_controller_->SetMaximumSetpointChange(
        get_parameter("controllers.speed.pid.setpoint.ramp.magnitude")
            .as_double());

    speed_controller_->UseOutputLimiter(true);

    brake_threshold_ =
        get_parameter("controllers.speed.brake.threshold").as_double();

    speed_controller_->SetMinimumOutput(-brake_threshold_);

    speed_controller_->SetMaximumOutput(1.0);

    speed_controller_->UseOutputChangeLimiter(
        get_parameter("controllers.speed.pid.output.change.enabled").as_bool());

    speed_controller_->SetMaximumOutputChange(
        get_parameter("controllers.speed.pid.output.change.magnitude")
            .as_double());

    speed_controller_->UseIntegralTermLimiter(
        get_parameter("controllers.speed.pid.limit.integral.enabled")
            .as_bool());

    speed_controller_->SetMaximumIntegralTerm(
        get_parameter("controllers.speed.pid.limit.integral.magnitude")
            .as_double());

    speed_controller_->UseDeadbandFilter(
        get_parameter("controllers.speed.pid.deadband.filter.enabled")
            .as_bool());

    speed_controller_->SetDeadbandLowerThreshold(
        get_parameter("controllers.speed.pid.deadband.filter.thresholds.lower")
            .as_double());

    speed_controller_->SetDeadbandUpperThreshold(
        get_parameter("controllers.speed.pid.deadband.filter.thresholds.upper")
            .as_double());

    speed_controller_feedforward_spline_ = std::make_shared<CubicSpline>(
        GetParameterVector(
            get_parameter("controllers.speed.pid.feedforward.curve.knots")
                .as_double_array()),
        GetParameterVector(
            get_parameter("controllers.speed.pid.feedforward.curve.values")
                .as_double_array()),
        get_parameter("controllers.speed.pid.feedforward.curve.degree")
            .as_int());
}

void DriveByWireNode::GetSteeringControllerParameters() {
    declare_parameter("controllers.steering.pid.feedforward.offset", 0.3);

    declare_parameter("controllers.steering.pid.gains.proportional", 1.0);

    declare_parameter("controllers.steering.pid.gains.integral", 0.1);

    declare_parameter("controllers.steering.pid.gains.derivative", 0.01);

    declare_parameter("controllers.steering.pid.setpoint.ramp.enabled", true);

    declare_parameter("controllers.steering.pid.setpoint.ramp.magnitude",
                      0.001);

    declare_parameter("controllers.steering.pid.output.change.enabled", true);

    declare_parameter("controllers.steering.pid.output.change.magnitude",
                      0.001);

    declare_parameter("controllers.steering.pid.limit.integral.enabled", true);

    declare_parameter("controllers.steering.pid.limit.integral.magnitude",
                      50.0);

    declare_parameter("controllers.steering.pid.deadband.filter.enabled", true);

    declare_parameter(
        "controllers.steering.pid.deadband.filter.thresholds.lower",
        -0.3);

    declare_parameter(
        "controllers.steering.pid.deadband.filter.thresholds.upper",
        0.3);

    declare_parameter("controllers.steering.pid.limit.error.enabled", true);

    declare_parameter("controllers.steering.pid.limit.error.magnitude", 0.04);

    declare_parameter("controllers.steering.max_steering_angle", 30.0);
    maximum_steering_angle_ =
        get_parameter("controllers.steering.max_steering_angle").as_double();
}

void DriveByWireNode::InitializeSteeringController() {
    steering_controller_ = std::make_shared<PIDController>();

    steering_controller_->SetFeedforwardOffset(
        get_parameter("controllers.steering.pid.feedforward.offset")
            .as_double());

    steering_controller_->SetProportionalGain(
        get_parameter("controllers.steering.pid.gains.proportional")
            .as_double());

    steering_controller_->SetIntegralGain(
        get_parameter("controllers.steering.pid.gains.integral").as_double());

    steering_controller_->SetDerivativeGain(
        get_parameter("controllers.steering.pid.gains.derivative").as_double());

    steering_controller_->UseSetpointRamping(
        get_parameter("controllers.steering.pid.setpoint.ramp.enabled")
            .as_bool());

    steering_controller_->SetMaximumSetpointChange(
        get_parameter("controllers.steering.pid.setpoint.ramp.magnitude")
            .as_double());

    steering_controller_->UseOutputLimiter(true);

    steering_controller_->UseOutputChangeLimiter(
        get_parameter("controllers.steering.pid.output.change.enabled")
            .as_bool());

    steering_controller_->SetMaximumOutputChange(
        get_parameter("controllers.steering.pid.output.change.magnitude")
            .as_double());

    steering_controller_->SetMinimumOutput(-1.0);

    steering_controller_->SetMaximumOutput(1.0);

    steering_controller_->UseDeadbandFilter(
        get_parameter("controllers.steering.pid.deadband.filter.enabled")
            .as_bool());

    steering_controller_->SetDeadbandLowerThreshold(
        get_parameter(
            "controllers.steering.pid.deadband.filter.thresholds.lower")
            .as_double());

    steering_controller_->SetDeadbandUpperThreshold(
        get_parameter(
            "controllers.steering.pid.deadband.filter.thresholds.upper")
            .as_double());

    steering_controller_->UseErrorThreshold(
        get_parameter("controllers.steering.pid.limit.error.enabled")
            .as_bool());

    steering_controller_->SetErrorThreshold(
        get_parameter("controllers.steering.pid.limit.error.magnitude")
            .as_double());

    steering_controller_->UseIntegralTermLimiter(
        get_parameter("controllers.steering.pid.limit.integral.enabled")
            .as_bool());

    steering_controller_->SetMaximumIntegralTerm(
        get_parameter("controllers.steering.pid.limit.integral.magnitude")
            .as_double());
}

Eigen::RowVectorXd
DriveByWireNode::GetParameterVector(std::vector<double> vector) {
    Eigen::RowVectorXd row_vector =
        Eigen::Map<Eigen::VectorXd>(vector.data(), vector.size());

    return row_vector;
}

bool DriveByWireNode::HasValidControl(rclcpp::Time& time) {
    if((time - last_control_time_).seconds() > control_timeout_) {
        return false;
    }

    return true;
}

rcl_interfaces::msg::SetParametersResult
DriveByWireNode::OnSetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    SetSpeedControllerParameters(parameters, result);
    SetSteeringControllerParameters(parameters, result);

    if(!result.successful) {
        RCLCPP_WARN(get_logger(),
                    "At least one of the provided parameters does not support "
                    "runtime reconfiguration!");
    }

    return result;
}

void DriveByWireNode::SetSpeedControllerParameters(
    const std::vector<rclcpp::Parameter>& parameters,
    rcl_interfaces::msg::SetParametersResult& result) {
    for(const auto& parameter : parameters) {
        if(parameter.get_name() == "controllers.speed.pid.gains.feedforward") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetFeedforwardGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.gains.proportional") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetProportionalGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.gains.integral") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetIntegralGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.gains.derivative") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetDerivativeGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.setpoint.ramp.enabled") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->UseSetpointRamping(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.setpoint.ramp.magnitude") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetMaximumSetpointChange(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.output.change.enabled") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->UseOutputChangeLimiter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.output.change.magnitude") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetMaximumOutputChange(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.limit.integral.enabled") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->UseIntegralTermLimiter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.limit.integral.magnitude") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetMaximumIntegralTerm(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.deadband.filter.enabled") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->UseDeadbandFilter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.deadband.filter.thresholds.lower") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetDeadbandLowerThreshold(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.speed.pid.deadband.filter.thresholds.upper") {
            const std::lock_guard<std::mutex> lock(controllers_mutex_);
            speed_controller_->SetDeadbandUpperThreshold(parameter.as_double());
        } else {
            return;
        }

        result.successful = true;
        speed_controller_->Reset();
    }
}

void DriveByWireNode::SetSteeringControllerParameters(
    const std::vector<rclcpp::Parameter>& parameters,
    rcl_interfaces::msg::SetParametersResult& result) {
    for(const auto& parameter : parameters) {
        if(parameter.get_name() ==
           "controllers.steering.pid.gains.proportional") {
            steering_controller_->SetProportionalGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.gains.integral") {
            steering_controller_->SetIntegralGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.gains.derivative") {
            steering_controller_->SetDerivativeGain(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.output.change.enabled") {
            steering_controller_->UseOutputChangeLimiter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.output.change.magnitude") {
            steering_controller_->SetMaximumOutputChange(parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.setpoint.ramp.enabled") {
            steering_controller_->UseSetpointRamping(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.setpoint.ramp.magnitude") {
            steering_controller_->SetMaximumSetpointChange(
                parameter.as_double());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.limit.integral.enabled") {
            steering_controller_->UseIntegralTermLimiter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controllers.steering.pid.limit.integral.magnitude") {
            steering_controller_->SetMaximumIntegralTerm(parameter.as_double());
        } else {
            return;
        }
    }

    result.successful = true;
    steering_controller_->Reset();
}

} // namespace ROS
} // namespace OLAV