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

#include <olav_control/multiplexer/node.hpp>

namespace OLAV {
namespace ROS {

ControlMultiplexerNode::ControlMultiplexerNode()
    : rclcpp::Node("control_multiplexer") {
    Configure();
    Activate();
}

void ControlMultiplexerNode::Configure() {
    GetParameters();
    Initialize();
}

void ControlMultiplexerNode::GetParameters() {
    declare_parameter("authorities.generic",
                      std::vector<std::string>{"terminal"});
    generic_ids_ = get_parameter("authorities.generic").as_string_array();

    declare_parameter("authorities.autonomy",
                      std::vector<std::string>{"autonomy"});
    autonomy_ids_ = get_parameter("authorities.autonomy").as_string_array();

    declare_parameter("authorities.gamepad", "gamepad");
    gamepad_id_ = get_parameter("authorities.gamepad").as_string();

    declare_parameter("mode.initial", "disabled");
    target_control_mode_ = active_control_mode_ =
        ControlMode(get_parameter("mode.initial").as_string());
}

void ControlMultiplexerNode::Initialize() {
    services_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscriptions_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    is_emergency_stop_triggered_ = false;
    is_drive_by_wire_enabled_ = false;
}

void ControlMultiplexerNode::Activate() {
    CreateSubscriptions();
    CreateServices();
    CreateClients();
    CreateTimers();
    CreatePublishers();
    StartTimers();
}

void ControlMultiplexerNode::CreateSubscriptions() {
    throttle_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "in/throttle",
            1,
            std::bind(&ControlMultiplexerNode::ThrottleCallback,
                      this,
                      std::placeholders::_1));

    brake_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "in/brake",
            1,
            std::bind(&ControlMultiplexerNode::BrakeCallback,
                      this,
                      std::placeholders::_1));

    steering_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "in/steering",
            1,
            std::bind(&ControlMultiplexerNode::SteeringCallback,
                      this,
                      std::placeholders::_1));

    heartbeat_subscription_ = create_subscription<std_msgs::msg::Header>(
        "in/heartbeat",
        1,
        std::bind(&ControlMultiplexerNode::HeartbeatCallback,
                  this,
                  std::placeholders::_1));

    emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "in/estop",
        1,
        std::bind(&ControlMultiplexerNode::EmergencyStopCallback,
                  this,
                  std::placeholders::_1));

    drive_by_wire_switch_subscription_ =
        create_subscription<std_msgs::msg::Bool>(
            "signals/drive_by_wire",
            1,
            std::bind(&ControlMultiplexerNode::DriveByWireSwitchCallback,
                      this,
                      std::placeholders::_1));

    ackermann_drive_subscription_ =
        create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "in/drive",
            1,
            std::bind(&ControlMultiplexerNode::AckermannDriveCallback,
                      this,
                      std::placeholders::_1));
}

void ControlMultiplexerNode::CreateServices() {
    set_control_mode_service_ =
        create_service<olav_interfaces::srv::SetControlMode>(
            "set_control_mode",
            std::bind(&ControlMultiplexerNode::SetControlMode,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_services_default,
            services_callback_group_);

    cycle_control_mode_service_ = create_service<std_srvs::srv::Trigger>(
        "cycle_control_mode",
        std::bind(&ControlMultiplexerNode::CycleControlMode,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default,
        services_callback_group_);
}

void ControlMultiplexerNode::CreateClients() {
    reset_speed_controller_client_ =
        create_client<std_srvs::srv::Trigger>("controllers/speed/reset");

    reset_steering_controller_client_ =
        create_client<std_srvs::srv::Trigger>("controllers/steering/reset");
}

void ControlMultiplexerNode::CreateTimers() {
    diagnostic_timer_ = create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&ControlMultiplexerNode::DiagnosticTimerCallback, this));
    diagnostic_timer_->cancel();
}

void ControlMultiplexerNode::CreatePublishers() {
    throttle_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "out/throttle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    brake_publisher_ = create_publisher<olav_interfaces::msg::SetpointStamped>(
        "out/brake",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "out/steering",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    speed_setpoint_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "out/speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_angle_setpoint_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "out/steering_angle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    heartbeat_publisher_ = create_publisher<std_msgs::msg::Header>(
        "out/heartbeat",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    diagnostic_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void ControlMultiplexerNode::StartTimers() { diagnostic_timer_->reset(); }

void ControlMultiplexerNode::Reset() { // TODO: Implement a reset function.
}

void ControlMultiplexerNode::UpdateMode() {
    if(target_control_mode_ == active_control_mode_) return;

    // If we are disabling the multiplexer, no additional steps are required.
    if(target_control_mode_.GetId() ==
       ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED) {
        is_speed_controller_ready_ = false;
        is_steering_controller_ready_ = false;
    }

    if(target_control_mode_ ==
       ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD) {
        // If we are moving from a state that used the speed controller, we must
        // ensure this is no longer marked as ready.
        if(active_control_mode_.GetId() ==
               ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS ||
           active_control_mode_.GetId() ==
               ControlModeIdentifier::
                   MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE) {
            is_speed_controller_ready_ = false;
        }
        // If we are moving from a state that was using no controllers at all,
        // we must ensure the steering controller is reset.
        else if(active_control_mode_.GetId() ==
                    ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED ||
                active_control_mode_.GetId() ==
                    ControlModeIdentifier::
                        MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER) {
            ResetSteeringController();
        }
    }

    // We are moving to autonomous mode: check the speed and steering
    // controllers state.
    if(target_control_mode_.GetId() ==
       ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS) {
        // If we are moving from a state that was not using any internal
        // controller, then we must reset both controllers.
        if(active_control_mode_.GetId() ==
               ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED ||
           active_control_mode_.GetId() ==
               ControlModeIdentifier::
                   MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER) {
            ResetSpeedController();
            ResetSteeringController();
        }
        // If we are moving from the gamepad state, the steering controller is
        // already active and does not have to be reset.
        else if(active_control_mode_.GetId() ==
                ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD) {
            ResetSpeedController();
        }
    }

    active_control_mode_ = target_control_mode_;
}

void ControlMultiplexerNode::HeartbeatCallback(
    const std_msgs::msg::Header::ConstSharedPtr heartbeat_message) {
    if(!IsEnabled()) return;

    switch(active_control_mode_.GetId()) {
    case ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED: return;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD:
        if(!ValidateHeader(*heartbeat_message, gamepad_id_)) return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS:
        if(!ValidateHeader(*heartbeat_message, autonomy_ids_)) return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER:
        if(!ValidateHeader(*heartbeat_message, generic_ids_)) return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE:
        if(!ValidateHeader(*heartbeat_message, generic_ids_)) return;
        break;
    default: throw std::invalid_argument("Invalid control mode!");
    }

    heartbeat_publisher_->publish(*heartbeat_message);
}

bool ControlMultiplexerNode::ValidateHeader(const std_msgs::msg::Header& header,
                                            const std::string& authority) {
    return header.frame_id == authority;
}

bool ControlMultiplexerNode::ValidateHeader(
    const std_msgs::msg::Header& header,
    const std::vector<std::string>& authorities) {
    return std::find(authorities.begin(), authorities.end(), header.frame_id) !=
        authorities.end();
}

bool ControlMultiplexerNode::IsEnabled() {
    return !is_emergency_stop_triggered_ && is_drive_by_wire_enabled_;
}

void ControlMultiplexerNode::PublishSafetyThrottle() {
    olav_interfaces::msg::SetpointStamped safety_throttle_message;
    safety_throttle_message.header.stamp = get_clock()->now();
    safety_throttle_message.header.frame_id = "olav-mux-74fb";
    safety_throttle_message.setpoint = 0.0;
    throttle_publisher_->publish(safety_throttle_message);
}

void ControlMultiplexerNode::PublishSafetyBrake() {
    olav_interfaces::msg::SetpointStamped safety_brake_message;
    safety_brake_message.header.stamp = get_clock()->now();
    safety_brake_message.header.frame_id = "olav-mux-74fb";
    safety_brake_message.setpoint = 1.0;
    brake_publisher_->publish(safety_brake_message);
}

void ControlMultiplexerNode::PublishSafetySteering() {
    olav_interfaces::msg::SetpointStamped safety_steering_message;
    safety_steering_message.header.stamp = get_clock()->now();
    safety_steering_message.header.frame_id = "olav-mux-74fb";
    safety_steering_message.setpoint = 0.0;
    steering_publisher_->publish(safety_steering_message);
}

void ControlMultiplexerNode::ThrottleCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr throttle_effort_message) {
    if(!IsEnabled()) return;

    switch(active_control_mode_.GetId()) {
    case ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED: return;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD:
        if(!ValidateHeader(throttle_effort_message->header, gamepad_id_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS:
        if(!ValidateHeader(throttle_effort_message->header,
                           speed_controller_id_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER:
        if(!ValidateHeader(throttle_effort_message->header, generic_ids_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE:
        if(!ValidateHeader(throttle_effort_message->header,
                           speed_controller_id_))
            return;
        break;
    default: throw std::invalid_argument("Invalid control mode!");
    }

    throttle_publisher_->publish(*throttle_effort_message);
}

void ControlMultiplexerNode::BrakeCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr brake_effort_message) {
    if(!IsEnabled()) return;

    switch(active_control_mode_.GetId()) {
    case ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED: return;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD:
        if(!ValidateHeader(brake_effort_message->header, gamepad_id_)) return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS:
        if(!ValidateHeader(brake_effort_message->header, speed_controller_id_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER:
        if(!ValidateHeader(brake_effort_message->header, generic_ids_)) return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE:
        if(!ValidateHeader(brake_effort_message->header, speed_controller_id_))
            return;
        break;
    default: throw std::invalid_argument("Invalid control mode!");
    }

    brake_publisher_->publish(*brake_effort_message);
}

void ControlMultiplexerNode::SteeringCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr steering_effort_message) {
    if(!IsEnabled()) return;

    switch(active_control_mode_.GetId()) {
    case ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED: return;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD:
        if(!ValidateHeader(steering_effort_message->header,
                           steering_controller_id_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS:
        if(!ValidateHeader(steering_effort_message->header,
                           steering_controller_id_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER:
        if(!ValidateHeader(steering_effort_message->header, generic_ids_))
            return;
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE:
        if(!ValidateHeader(steering_effort_message->header,
                           steering_controller_id_))
            return;
        break;
    default: throw std::invalid_argument("Invalid control mode!");
    }

    steering_publisher_->publish(*steering_effort_message);
}

void ControlMultiplexerNode::PublishSpeedSetpoint(
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
        ackermann_drive_message) {
    olav_interfaces::msg::SetpointStamped speed_setpoint_message;
    speed_setpoint_message.header = ackermann_drive_message->header;
    speed_setpoint_message.setpoint = ackermann_drive_message->drive.speed;
    speed_setpoint_publisher_->publish(speed_setpoint_message);
}

void ControlMultiplexerNode::PublishSteeringAngleSetpoint(
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
        ackermann_drive_message) {
    olav_interfaces::msg::SetpointStamped steering_angle_setpoint_message;
    steering_angle_setpoint_message.header = ackermann_drive_message->header;
    steering_angle_setpoint_message.setpoint =
        ackermann_drive_message->drive.steering_angle;
    steering_angle_setpoint_publisher_->publish(
        steering_angle_setpoint_message);
}

void ControlMultiplexerNode::AckermannDriveCallback(
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
        ackermann_drive_message) {
    if(!IsEnabled()) return;

    switch(active_control_mode_.GetId()) {
    case ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED: return;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD:
        if(!ValidateHeader(ackermann_drive_message->header, gamepad_id_))
            return;
        PublishSteeringAngleSetpoint(ackermann_drive_message);
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS:
        if(!ValidateHeader(ackermann_drive_message->header, autonomy_ids_))
            return;
        PublishSpeedSetpoint(ackermann_drive_message);
        PublishSteeringAngleSetpoint(ackermann_drive_message);
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER:
        return;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE:
        if(!ValidateHeader(ackermann_drive_message->header, generic_ids_))
            return;
        PublishSpeedSetpoint(ackermann_drive_message);
        PublishSteeringAngleSetpoint(ackermann_drive_message);
        break;
    default: throw std::invalid_argument("Invalid control mode!");
    }
}

void ControlMultiplexerNode::ResetSpeedController() {
    if(!IsActive()) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = reset_speed_controller_client_->async_send_request(
        request,
        std::bind(&ControlMultiplexerNode::ResetSpeedControllerCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::ResetSpeedControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if(future.get()->success) {
        RCLCPP_INFO(get_logger(), "Speed controller reset successfully!");
    } else {
        RCLCPP_ERROR(get_logger(), "Could not reset speed controller!");
    }
}

void ControlMultiplexerNode::ResetSteeringController() {
    if(!IsActive()) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = reset_steering_controller_client_->async_send_request(
        request,
        std::bind(&ControlMultiplexerNode::ResetSteeringControllerCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::ResetSteeringControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if(future.get()->success) {
        RCLCPP_INFO(get_logger(), "Steering controller reset successfully!");
    } else {
        RCLCPP_ERROR(get_logger(), "Could not reset steering controller!");
    }
}

void ControlMultiplexerNode::CycleControlMode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    if(!IsActive()) {
        std::string message(
            "Could not cycle control mode: the system is not active.");
        RCLCPP_ERROR(get_logger(), message.c_str());
        response->success = false;
        response->message = message;
        return;
    }

    target_control_mode_ = active_control_mode_.GetNext();
    UpdateMode();

    std::string message("Switched control mode to " +
                        target_control_mode_.GetName());
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message;
}

void ControlMultiplexerNode::SetControlMode(
    const std::shared_ptr<olav_interfaces::srv::SetControlMode::Request>
        request,
    std::shared_ptr<olav_interfaces::srv::SetControlMode::Response> response) {
    if(!IsActive()) {
        std::string message(
            "Could not set control mode: the system is not active.");
        RCLCPP_ERROR(get_logger(), message.c_str());
        response->success = false;
        response->message = message;
        return;
    }

    target_control_mode_ =
        ControlMode(static_cast<ControlModeIdentifier>(request->mode));
    UpdateMode();

    std::string message("Set control mode to " +
                        target_control_mode_.GetName());
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message;
}

bool ControlMultiplexerNode::IsActive() {
    return !emergency_stop_ && is_drive_by_wire_enabled_;
}

void ControlMultiplexerNode::EmergencyStopCallback(
    const std_msgs::msg::Bool::ConstSharedPtr emergency_stop_message) {
    RCLCPP_ERROR(get_logger(), "Emergency stop triggered!");

    if(emergency_stop_message->data) {
        emergency_stop_ = true;
        return;
    }

    emergency_stop_ = false;
}

void ControlMultiplexerNode::DriveByWireSwitchCallback(
    const std_msgs::msg::Bool::ConstSharedPtr drive_by_wire_switch_message) {
    if(drive_by_wire_switch_message->data && !last_drive_by_wire_state_) {
        ResetSpeedController();
        ResetSteeringController();
        last_drive_by_wire_state_ = true;
    } else if(!drive_by_wire_switch_message->data &&
              last_drive_by_wire_state_) {
        is_speed_controller_ready_ = false;
        is_steering_controller_ready_ = false;
        last_drive_by_wire_state_ = false;
    }

    is_drive_by_wire_enabled_ = drive_by_wire_switch_message->data;
}

void ControlMultiplexerNode::DiagnosticTimerCallback() {
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_message;
    diagnostic_message.level = emergency_stop_
        ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
        : diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic_message.name = "olav/multiplexer/estop";
    diagnostic_message.message = emergency_stop_
        ? "Emergency stop raised! All node functionality is halted"
        : "The emergency stop is not raised.";
    diagnostic_message.hardware_id = "olav-mux-74fb";

    {
        diagnostic_msgs::msg::KeyValue key_value_message;
        key_value_message.key = "mode";
        key_value_message.value = active_control_mode_.GetName();
        diagnostic_message.values.push_back(key_value_message);
    }

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array_message;
    diagnostic_array_message.header.stamp = get_clock()->now();
    diagnostic_array_message.status.push_back(diagnostic_message);

    diagnostic_publisher_->publish(diagnostic_array_message);
}

} // namespace ROS
} // namespace OLAV