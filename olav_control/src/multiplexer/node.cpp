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
    declare_parameter("initial.mode", "manual");
}

void ControlMultiplexerNode::Initialize() {
    services_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscriptions_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    active_control_mode_ = get_parameter("initial.mode").as_string();
}

void ControlMultiplexerNode::Activate() {
    CreateSubscriptions();
    CreateServices();
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

    CreateCommandSubscriptions();
}

void ControlMultiplexerNode::CreateCommandSubscriptions() {
    ackermann_drive_subscription_ =
        create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "in/drive",
            1,
            std::bind(&ControlMultiplexerNode::AckermannDriveCallback,
                      this,
                      std::placeholders::_1));
}

void ControlMultiplexerNode::DeleteCommandSubscriptions() {
    ackermann_drive_subscription_.reset();
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
    start_speed_controller_client_ =
        create_client<std_srvs::srv::Trigger>("controllers/speed/start");

    stop_speed_controller_client_ =
        create_client<std_srvs::srv::Trigger>("controllers/speed/stop");

    start_steering_controller_client_ =
        create_client<std_srvs::srv::Trigger>("controllers/steering/start");

    stop_steering_controller_client_ =
        create_client<std_srvs::srv::Trigger>("controllers/steering/stop");
}

void ControlMultiplexerNode::CreateTimers() {
    diagnostic_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0),
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

void ControlMultiplexerNode::HeartbeatCallback(
    const std_msgs::msg::Header::ConstSharedPtr heartbeat_message) {
    if(!IsActive()) return;

    if((active_control_mode_ == "manual" &&
        heartbeat_message->frame_id == "gamepad") ||
       (active_control_mode_ == "autonomous" &&
        heartbeat_message->frame_id == "autonomy")) {
        heartbeat_publisher_->publish(*heartbeat_message);
    }
}

void ControlMultiplexerNode::ThrottleCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr throttle_effort_message) {
    if(!IsActive()) return;

    if((active_control_mode_ == "gamepad" &&
        throttle_effort_message->header.frame_id == "gamepad") ||
       (active_control_mode_ == "autonomy" &&
        throttle_effort_message->header.frame_id == "controller"))
        brake_publisher_->publish(*throttle_effort_message);
}

void ControlMultiplexerNode::BrakeCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr brake_effort_message) {
    if(!IsActive()) return;

    if((active_control_mode_ == "gamepad" &&
        brake_effort_message->header.frame_id == "gamepad") ||
       (active_control_mode_ == "autonomy" &&
        brake_effort_message->header.frame_id == "controller"))
        brake_publisher_->publish(*brake_effort_message);
}

void ControlMultiplexerNode::SteeringCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr steering_effort_message) {
    if(!IsActive()) return;

    if((active_control_mode_ == "gamepad" &&
        steering_effort_message->header.frame_id == "controller") ||
       (active_control_mode_ == "autonomy" &&
        steering_effort_message->header.frame_id == "controller")) {
        steering_publisher_->publish(*steering_effort_message);
    }
}

void ControlMultiplexerNode::AckermannDriveCallback(
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
        ackermann_drive_message) {
    if(!IsActive()) return;

    // Check the active control mode and authority.
    if((ackermann_drive_message->header.frame_id == "autonomy") &&
       (active_control_mode_ == "autonomous")) {
        olav_interfaces::msg::SetpointStamped speed_setpoint;
        speed_setpoint.header = ackermann_drive_message->header;
        speed_setpoint.setpoint = ackermann_drive_message->drive.speed;

        olav_interfaces::msg::SetpointStamped steering_angle_setpoint;
        steering_angle_setpoint.header = ackermann_drive_message->header;
        steering_angle_setpoint.setpoint =
            ackermann_drive_message->drive.steering_angle;
    }
}

void ControlMultiplexerNode::StartSpeedController() {
    if(!IsActive()) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = start_speed_controller_client_->async_send_request(
        request,
        std::bind(&ControlMultiplexerNode::StartSpeedControllerCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::StartSpeedControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if(future.get()->success) {
        RCLCPP_INFO(get_logger(), "Speed controller started successfully!");

    } else {
        RCLCPP_ERROR(get_logger(), "Could not start speed controller!");
        is_faulty_ = true;
    }
}

void ControlMultiplexerNode::StopSpeedController() {
    if(!IsActive()) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = stop_speed_controller_client_->async_send_request(
        request,
        std::bind(&ControlMultiplexerNode::StopSpeedControllerCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::StopSpeedControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if(future.get()->success) {
        RCLCPP_INFO(get_logger(), "Speed controller stopped successfully!");
    } else {
        RCLCPP_ERROR(get_logger(), "Could not stop speed controller!");
    }
}

void ControlMultiplexerNode::StartSteeringController() {
    if(!IsActive()) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = start_steering_controller_client_->async_send_request(
        request,
        std::bind(&ControlMultiplexerNode::StartSteeringControllerCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::StartSteeringControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if(future.get()->success) {
        RCLCPP_INFO(get_logger(), "Steering controller started successfully!");
    } else {
        RCLCPP_ERROR(get_logger(), "Could not start steering controller!");
        is_faulty_ = true;
    }
}

void ControlMultiplexerNode::StopSteeringController() {
    if(!IsActive()) return;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = stop_steering_controller_client_->async_send_request(
        request,
        std::bind(&ControlMultiplexerNode::StopSteeringControllerCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::StopSteeringControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if(future.get()->success) {
        RCLCPP_INFO(get_logger(), "Steering controller stopped successfully!");
    } else {
        RCLCPP_ERROR(get_logger(), "Could not stop steering controller!");
        is_faulty_ = true;
    }
}

void ControlMultiplexerNode::SetMode(const std::string& mode) {
    if(mode == "manual") {
        active_control_mode_ = "manual";
        DeleteCommandSubscriptions();
    } else if(mode == "autonomous") {
        active_control_mode_ = "autonomous";
        CreateCommandSubscriptions();
    }
}

void ControlMultiplexerNode::CycleControlMode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    if(emergency_stop_) { // TODO ERROR STATE
        return;
    };

    std::string target_control_mode;
    if(active_control_mode_ == "manual") {
        target_control_mode = "autonomous";
        StartSpeedController();
        StartSteeringController();
    } else if(active_control_mode_ == "autonomous") {
        target_control_mode = "manual";
        StopSpeedController();
    } else {
        std::string message("Invalid control mode.");
        response->success = false;
        response->message = message;
        return;
    }

    SetMode(target_control_mode);

    std::string message(target_control_mode.c_str());
    RCLCPP_INFO(get_logger(), "Switched control mode to %s.", message.c_str());
    response->success = true;
    response->message = message;
}

void ControlMultiplexerNode::SetControlMode(
    const std::shared_ptr<olav_interfaces::srv::SetControlMode::Request>
        request,
    std::shared_ptr<olav_interfaces::srv::SetControlMode::Response> response) {
    if(emergency_stop_) { // TODO ERROR STATE
        return;
    };

    // Set the control mode to manual.
    if(request->mode == olav_interfaces::srv::SetControlMode::Request::MANUAL) {
        SetMode("manual");
        // Set the control mode to autonomous.
    } else if(request->mode ==
              olav_interfaces::srv::SetControlMode::Request::AUTONOMOUS) {
        SetMode("autonomous");
    } else {
        std::string message("Invalid control mode");
        RCLCPP_WARN(get_logger(), message.c_str());
        response->success = false;
        response->message = message;
        return;
    }

    std::string message("Switched control mode to " + active_control_mode_);
    RCLCPP_WARN(get_logger(), message.c_str());
    response->success = false;
    response->message = message;
}

bool ControlMultiplexerNode::IsActive() {
    return !emergency_stop_ && !is_faulty_;
}

void ControlMultiplexerNode::EmergencyStopCallback(
    const std_msgs::msg::Bool::ConstSharedPtr emergency_stop_message) {
    RCLCPP_ERROR(get_logger(), "Emergency stop triggered!");

    if(emergency_stop_message->data) {
        emergency_stop_ = true;
        StopSpeedController();
        StopSteeringController();
        return;
    }

    emergency_stop_ = false;
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

    diagnostic_msgs::msg::KeyValue keyvalue_message;
    keyvalue_message.key = "mode";
    keyvalue_message.value = active_control_mode_;
    diagnostic_message.values.push_back(keyvalue_message);

    {
        diagnostic_msgs::msg::KeyValue keyvalue_message;
        keyvalue_message.key = "speed_controller";
        keyvalue_message.value = "true";
        diagnostic_message.values.push_back(keyvalue_message);
    }

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array_message;
    diagnostic_array_message.header.stamp = get_clock()->now();
    diagnostic_array_message.status.push_back(diagnostic_message);

    diagnostic_publisher_->publish(diagnostic_array_message);
}

} // namespace ROS
} // namespace OLAV