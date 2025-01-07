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

    declare_parameter("limits.warn", false);
    warn_on_bounds_violations_ = get_parameter("limits.warn").as_bool();

    declare_parameter("limits.efforts.throttle", 1.0);
    maximum_throttle_effort_ =
        get_parameter("limits.efforts.throttle").as_double();

    declare_parameter("limits.efforts.brake", 1.0);
    maximum_brake_effort_ = get_parameter("limits.efforts.brake").as_double();

    declare_parameter("limits.efforts.steering", 1.0);
    maximum_steering_effort_ =
        get_parameter("limits.efforts.steering").as_double();

    declare_parameter("limits.setpoint.speed", 4.17);
    maximum_speed_setpoint_ =
        get_parameter("limits.setpoint.speed").as_double();

    declare_parameter("limits.setpoint.steering_angle", 0.576);
    maximum_steering_angle_setpoint_ =
        get_parameter("limits.setpoint.steering_angle").as_double();

    declare_parameter("brake_threshold", 95.0);
    brake_threshold_ = get_parameter("brake_threshold").as_double();

    declare_parameter("debug.warn_once", true);
    warn_once_ = get_parameter("debug.warn_once").as_bool();

    set_parameters_callback_ = add_on_set_parameters_callback(
        std::bind(&ControlMultiplexerNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void ControlMultiplexerNode::Initialize() {
    services_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscriptions_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    active_control_mode_ =
        ToControlMode(get_parameter("initial.mode").as_string());
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

    CreateSetpointSubscriptions();
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

void ControlMultiplexerNode::CreateSetpointSubscriptions() {
    ackermann_drive_subscription_ =
        create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "in/drive",
            1,
            std::bind(&ControlMultiplexerNode::AckermannDriveCallback,
                      this,
                      std::placeholders::_1));
}

void ControlMultiplexerNode::ThrottleCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr throttle_effort_message) {
    if((active_control_mode_ == ControlMode::MANUAL &&
        throttle_effort_message->header.frame_id == "gamepad")) {
        if(IsValidThrottleEffort(throttle_effort_message->setpoint)) {
            throttle_publisher_->publish(*throttle_effort_message);
        }
    } else if(active_control_mode_ == ControlMode::AUTONOMOUS &&
              throttle_effort_message->header.frame_id == "controller") {
        if(IsValidThrottleEffort(throttle_effort_message->setpoint)) {
            throttle_publisher_->publish(*throttle_effort_message);
        }
    }
}

void ControlMultiplexerNode::BrakeCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr brake_effort_message) {
    if((active_control_mode_ == ControlMode::MANUAL &&
        brake_effort_message->header.frame_id == "gamepad")) {
        if(IsValidBrakeEffort(brake_effort_message->setpoint)) {
            brake_publisher_->publish(*brake_effort_message);
        }
    } else if(active_control_mode_ == ControlMode::AUTONOMOUS &&
              brake_effort_message->header.frame_id == "controller") {
        if(IsValidBrakeEffort(brake_effort_message->setpoint)) {
            brake_publisher_->publish(*brake_effort_message);
        }
    }
}

void ControlMultiplexerNode::SteeringCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr steering_effort_message) {
    if(steering_effort_message->header.frame_id == "controller") {
        if(IsValidSteeringEffort(steering_effort_message->setpoint)) {
            steering_publisher_->publish(*steering_effort_message);
        }
    }
}

void ControlMultiplexerNode::AckermannDriveCallback(
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
        ackermann_drive_message) {
    // Check the active control mode and authority.
    if((ackermann_drive_message->header.frame_id == "autonomy") &&
       (active_control_mode_ == ControlMode::AUTONOMOUS)) {
        // Check and publish the speed setpoint.
        if(IsValidSpeedSetpoint(ackermann_drive_message->drive.speed)) {
            auto speed_setpoint =
                std::make_shared<olav_interfaces::msg::SetpointStamped>();
            speed_setpoint->header = ackermann_drive_message->header;
            speed_setpoint->setpoint = ackermann_drive_message->drive.speed;

            // TODO: Uh, what to do now that there is no speed setpoint?
            // speed_setpoint_publisher_->publish(*speed_setpoint);
        } else {
            RCLCPP_WARN(get_logger(), "Autonomy sent an invalid target speed!");
        }

        // Check and publish the steering angle setpoint.
        if(IsValidSteeringAngleSetpoint(
               ackermann_drive_message->drive.steering_angle)) {
            auto steering_angle_setpoint =
                std::make_shared<olav_interfaces::msg::SetpointStamped>();
            steering_angle_setpoint->header = ackermann_drive_message->header;
            steering_angle_setpoint->setpoint =
                ackermann_drive_message->drive.steering_angle;

            // TODO: What to do now that this one does not exist?
            // steering_angle_setpoint_publisher_->publish(
            //    *steering_angle_setpoint);
        } else {
            RCLCPP_WARN(get_logger(),
                        "Autonomy sent an invalid target steering angle!");
        }
    }
}

void ControlMultiplexerNode::HeartbeatCallback(
    const std_msgs::msg::Header::ConstSharedPtr heartbeat_message) {
    if((active_control_mode_ == ControlMode::MANUAL &&
        heartbeat_message->frame_id == "gamepad") ||
       (active_control_mode_ == ControlMode::AUTONOMOUS &&
        heartbeat_message->frame_id == "autonomy")) {
        heartbeat_publisher_->publish(*heartbeat_message);
    }
}

void ControlMultiplexerNode::EmergencyStopCallback(
    const std_msgs::msg::Bool::ConstSharedPtr emergency_stop_message) {
    if(emergency_stop_message->data) {
        RCLCPP_ERROR(get_logger(), "Emergency stop triggered!");
        emergency_stop_ = true;

        // FIXME: Send stop requests, zero stored controls, etc.
    }
}

void ControlMultiplexerNode::SetMode(ControlMode mode) {
    if(mode == ControlMode::MANUAL) {
        active_control_mode_ = ControlMode::MANUAL;
        // TODO: Uh, what to do now that there is no speed subscription?
        // speed_subscription_.reset();
        // Do not reset the subscription pointer.
        // target_steering_angle_subscription_.reset();
    } else if(mode == ControlMode::AUTONOMOUS) {
        active_control_mode_ = ControlMode::AUTONOMOUS;
        CreateSetpointSubscriptions();
    }
}

void ControlMultiplexerNode::CycleControlMode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    ControlMode target_control_mode;
    if(active_control_mode_ == ControlMode::MANUAL) {
        if(autonomy_switch_) {
            // Set the control mode to autonomous.
            target_control_mode = ControlMode::AUTONOMOUS;

            // Send a request to start the speed controller.
            {
                auto request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                const auto future =
                    start_speed_controller_client_->async_send_request(
                        request,
                        std::bind(&ControlMultiplexerNode::
                                      StartSpeedControllerCallback,
                                  this,
                                  std::placeholders::_1));
            }

            // Send a request to start the steering controller.
            {
                auto request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                const auto future =
                    start_steering_controller_client_->async_send_request(
                        request,
                        std::bind(&ControlMultiplexerNode::
                                      StartSteeringControllerCallback,
                                  this,
                                  std::placeholders::_1));
            }
        } else {
            std::string message(
                "Cycle mode requested, but autonomy is not enabled!");
            RCLCPP_ERROR(get_logger(), message.c_str());
            response->success = false;
            response->message = message;
            return;
        }

    } else if(active_control_mode_ == ControlMode::AUTONOMOUS) {
        // Set the control mode to manual.
        target_control_mode = ControlMode::MANUAL;

        // Send a request to stop the speed controller.
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            const auto future =
                stop_speed_controller_client_->async_send_request(
                    request,
                    std::bind(
                        &ControlMultiplexerNode::StopSpeedControllerCallback,
                        this,
                        std::placeholders::_1));
        }
        // Send a request to stop the speed controller.
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            const auto future =
                stop_steering_controller_client_->async_send_request(
                    request,
                    std::bind(
                        &ControlMultiplexerNode::StopSteeringControllerCallback,
                        this,
                        std::placeholders::_1));
        }
    } else {
        std::string message("Invalid control mode.");
        response->success = false;
        response->message = message;
        return;
    }
    SetMode(target_control_mode);

    std::string message(FromControlMode(target_control_mode).c_str());
    RCLCPP_INFO(get_logger(), "Switched control mode to %s.", message.c_str());
    response->success = true;
    response->message = message;
}

void ControlMultiplexerNode::SetControlMode(
    const std::shared_ptr<olav_interfaces::srv::SetControlMode::Request>
        request,
    std::shared_ptr<olav_interfaces::srv::SetControlMode::Response> response) {
    // Set the control mode to manual.
    if(request->mode == olav_interfaces::srv::SetControlMode::Request::MANUAL) {
        SetMode(ControlMode::MANUAL);
        // Set the control mode to autonomous.
    } else if(request->mode ==
              olav_interfaces::srv::SetControlMode::Request::AUTONOMOUS) {
        SetMode(ControlMode::AUTONOMOUS);
    } else {
        std::string message("Invalid control mode");
        RCLCPP_WARN(get_logger(), message.c_str());
        response->success = false;
        response->message = message;
        return;
    }

    std::string message("Switched control mode to " +
                        FromControlMode(active_control_mode_));
    RCLCPP_WARN(get_logger(), message.c_str());
    response->success = false;
    response->message = message;
}

rcl_interfaces::msg::SetParametersResult
ControlMultiplexerNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    for(const auto& parameter : parameters) {
        RCLCPP_INFO(get_logger(),
                    "%s [%s] = %s",
                    parameter.get_name().c_str(),
                    parameter.get_type_name().c_str(),
                    parameter.value_to_string().c_str());

        if(parameter.get_name() == "brake_threshold") {
            if(parameter.as_double() >= 0.0 && parameter.as_double() <= 1.0) {
                brake_threshold_ = parameter.as_double();
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters set successfully.";
    return result;
}

void ControlMultiplexerNode::StartSpeedControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_DEBUG(get_logger(),
                     "Speed controller start request request successful:\n%s",
                     response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Speed controller start request failed:\n%s",
                     response->message.c_str());
    }

    // TODO: Handle the start failure.
}

void ControlMultiplexerNode::StopSpeedControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_DEBUG(get_logger(),
                     "Speed controller stop request request successful:\n%s",
                     response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Speed controller stop request failed:\n%s",
                     response->message.c_str());
    }

    // TODO: Handle the stop failure.
}

void ControlMultiplexerNode::StartSteeringControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_DEBUG(
            get_logger(),
            "Steering controller start request request successful:\n%s",
            response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Steering controller start request failed:\n%s",
                     response->message.c_str());
    }

    // TODO: Handle the start failure.
}

void ControlMultiplexerNode::StopSteeringControllerCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_DEBUG(get_logger(),
                     "Steering controller stop request request successful:\n%s",
                     response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Steering controller stop request failed:\n%s",
                     response->message.c_str());
    }

    // TODO: Handle the stop failure.
}

bool ControlMultiplexerNode::IsValidThrottleEffort(
    const double& throttle_effort) {
    if(throttle_effort > maximum_throttle_effort_ || throttle_effort < 0.0) {
        if(warn_on_bounds_violations_) {
            RCLCPP_WARN(get_logger(),
                        "Throttle effort out of bounds: %0.2f is outside range "
                        "(%0.2f, %0.2f)!",
                        throttle_effort,
                        0.0,
                        maximum_throttle_effort_);
        }
        return false;
    }
    return true;
}

bool ControlMultiplexerNode::IsValidBrakeEffort(const double& brake_effort) {
    if(brake_effort > maximum_brake_effort_ || brake_effort < 0.0) {
        if(warn_on_bounds_violations_) {
            RCLCPP_WARN(get_logger(),
                        "Brake effort out of bounds: %0.2f is outside range "
                        "(%0.2f, %0.2f)!",
                        brake_effort,
                        0.0,
                        maximum_brake_effort_);
        }
        return false;
    }
    return true;
}

bool ControlMultiplexerNode::IsValidSteeringEffort(
    const double& steering_effort) {
    if(steering_effort > maximum_steering_effort_ ||
       steering_effort < -maximum_steering_effort_) {
        if(warn_on_bounds_violations_) {
            RCLCPP_WARN(get_logger(),
                        "Steering effort out of bounds: %0.2f is outside range "
                        "(%0.2f, %0.2f)!",
                        steering_effort,
                        -maximum_steering_effort_,
                        maximum_steering_effort_);
        }
        return false;
    }
    return true;
}

// TODO: Expose the valid speed setpoint range.
bool ControlMultiplexerNode::IsValidSpeedSetpoint(
    const double& speed_setpoint) {
    if(speed_setpoint > 15.0 || speed_setpoint < -15.0) {
        if(warn_on_bounds_violations_) {
            RCLCPP_WARN(get_logger(),
                        "Speed setpoint out of bounds: %0.2f is outside range "
                        "(%0.2f, %0.2f)!",
                        speed_setpoint,
                        -15.0,
                        15.0);
        }
        return false;
    }
    return true;
}

// TODO: Expose the valid speed steering angle range.
bool ControlMultiplexerNode::IsValidSteeringAngleSetpoint(
    const double& steering_angle_setpoint) {
    if(steering_angle_setpoint > 0.525 || steering_angle_setpoint < -0.525) {
        if(warn_on_bounds_violations_) {
            RCLCPP_WARN(
                get_logger(),
                "Steering angle setpoint out of bounds: %0.2f is outside range "
                "(%0.2f, %0.2f)!",
                steering_angle_setpoint,
                -0.525,
                0.525);
        }
        return false;
    }
    return true;
}

void ControlMultiplexerNode::DiagnosticTimerCallback() {
    auto diagnostic_array_message =
        std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_array_message->header.stamp = get_clock()->now();

    auto diagnostic_message =
        std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diagnostic_message->level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic_message->name = "olav/mux/running";
    diagnostic_message->message =
        "The control multiplexer is currently muxing commands.";
    diagnostic_message->hardware_id = "olav-mux-74fb";
    diagnostic_array_message->status.push_back(*diagnostic_message);

    diagnostic_publisher_->publish(*diagnostic_array_message);
}

} // namespace ROS
} // namespace OLAV