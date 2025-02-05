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

    declare_parameter("debug.log_throttle", 1.0);
    log_throttle_delay_ = get_parameter("debug.log_throttle").as_double();
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

    // Initialize the time references for the health checks.
    last_engine_speed_time_ =
        get_clock()->now() - rclcpp::Duration(health_check_period_, 0);
    last_odometry_time_ =
        get_clock()->now() - rclcpp::Duration(health_check_period_, 0);
    last_heartbeat_time_ =
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
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscriptions_callback_group_;

    engine_speed_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "feedback/engine/speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&DriveByWireNode::EngineSpeedCallback,
                      this,
                      std::placeholders::_1),
            subscription_options);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "feedback/odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&DriveByWireNode::OdometryCallback,
                  this,
                  std::placeholders::_1),
        subscription_options);

    heartbeat_subscription_ = create_subscription<std_msgs::msg::Header>(
        "signals/heartbeat",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&DriveByWireNode::HeartbeatCallback,
                  this,
                  std::placeholders::_1),
        subscription_options);

    throttle_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "controls/throttle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&DriveByWireNode::ThrottleCallback,
                      this,
                      std::placeholders::_1),
            subscription_options);

    brake_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "controls/brake",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&DriveByWireNode::BrakeCallback,
                      this,
                      std::placeholders::_1),
            subscription_options);

    steering_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "controls/steering",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&DriveByWireNode::SteeringCallback,
                      this,
                      std::placeholders::_1),
            subscription_options);
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
    connect_timer_->reset();
    health_check_timer_->reset();
    writer_timer_->reset();
    reader_timer_->reset();
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

void DriveByWireNode::DebugCallback() {
    if(!is_connected_) return;
    PublishPLCStatus();
}

void DriveByWireNode::DiagnosticsCallback() {
    auto diagnostic_array_message =
        std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_array_message->header.stamp = get_clock()->now(),
    GetDiagnostics(diagnostic_array_message);
    diagnostics_publisher_->publish(*diagnostic_array_message);
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

    has_heartbeat_ =
        (current_time - last_heartbeat_time_).seconds() >= health_check_period_
        ? false
        : true;
}

bool DriveByWireNode::IsHealthy() {
    return is_connected_ && has_engine_speed_ && has_odometry_ &&
        has_heartbeat_ && is_ready_;
}

void DriveByWireNode::ThrottleCallback(
    olav_interfaces::msg::SetpointStamped::ConstSharedPtr throttle_message) {
    if(!IsHealthy()) return;

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        drive_by_wire_setpoint_->SetThrottle(throttle_message->setpoint);
    }
}

void DriveByWireNode::BrakeCallback(
    olav_interfaces::msg::SetpointStamped::ConstSharedPtr brake_message) {
    if(!IsHealthy()) return;

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        drive_by_wire_setpoint_->SetBrake(brake_message->setpoint);
    }
}

void DriveByWireNode::SteeringCallback(
    olav_interfaces::msg::SetpointStamped::ConstSharedPtr steering_message) {
    if(!IsHealthy()) return;

    {
        std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
        drive_by_wire_setpoint_->SetSteering(steering_message->setpoint);
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

    if(!is_ready_) {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "The drive-by-wire is not ready to receive a cycle ignition "
            "command!!");
        return;
    };

    bool ignition_state = false;
    {
        std::shared_lock<std::shared_mutex> feedback_lock(feedback_mutex_);
        ignition_state = drive_by_wire_feedback_->IsIgnitionOn();
    }

    try {
        {
            std::unique_lock<std::shared_mutex> setpoint_lock(setpoint_mutex_);
            drive_by_wire_setpoint_->SetIgnition(!ignition_state);
        }

        SetResponseSuccessInfo<std_srvs::srv::Trigger::Response>(
            response,
            "Ignition state set to " + std::to_string(ignition_state) + ".");
    } catch(OLAV::Exceptions::DriveByWireInterfaceException& exception) {
        SetResponseFailError<std_srvs::srv::Trigger::Response>(
            response,
            "Could not set ignition state: " + std::string(exception.what()));
    }
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

} // namespace ROS
} // namespace OLAV