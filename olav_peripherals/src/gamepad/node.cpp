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

#include <olav_peripherals/gamepad/node.hpp>

namespace OLAV {
namespace ROS {

GamepadInterfaceNode::GamepadInterfaceNode() : rclcpp::Node("gamepad_node") {
    Configure();
    Activate();
}

void GamepadInterfaceNode::Configure() {
    GetParameters();
    Initialize();
}

void GamepadInterfaceNode::GetParameters() {
    declare_parameter("throttle.deadzone", 0.01);
    throttle_deadzone_ = get_parameter("throttle.deadzone").as_double();

    declare_parameter("throttle.interpolate", false);
    use_throttle_curve_ = get_parameter("throttle.interpolate").as_bool();

    declare_parameter("throttle.curve", std::vector<double>{0.0, 0.5, 1.0});
    throttle_curve_points_ = get_parameter("throttle.curve").as_double_array();
    throttle_curve_ =
        boost::math::interpolators::cardinal_cubic_b_spline<double>(
            throttle_curve_points_.begin(),
            throttle_curve_points_.end(),
            throttle_deadzone_,
            (1.0 - throttle_deadzone_) / (throttle_curve_points_.size() - 1),
            0.0,
            0.0);

    declare_parameter("brake.deadzone", 0.01);
    brake_deadzone_ = get_parameter("brake.deadzone").as_double();

    declare_parameter("brake.interpolate", false);
    use_brake_curve_ = get_parameter("brake.interpolate").as_bool();

    declare_parameter("brake.curve", std::vector<double>{0.0, 0.5, 1.0});
    brake_curve_points_ = get_parameter("brake.curve").as_double_array();
    brake_curve_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
        brake_curve_points_.begin(),
        brake_curve_points_.end(),
        brake_deadzone_,
        (1.0 - brake_deadzone_) / (brake_curve_points_.size() - 1),
        0.0,
        0.0);

    declare_parameter("steering.maximum_angle", 0.524);
    maximum_steering_angle_ =
        get_parameter("steering.maximum_angle").as_double();

    declare_parameter("steering.deadzone", 0.01);
    steering_deadzone_ = get_parameter("steering.deadzone").as_double();

    declare_parameter("steering.interpolate", false);
    use_steering_curve_ = get_parameter("steering.interpolate").as_bool();

    declare_parameter("steering.curve", std::vector<double>{-1.0, 0.0, 1.0});
    steering_curve_points_ = get_parameter("steering.curve").as_double_array();
    steering_curve_ =
        boost::math::interpolators::cardinal_cubic_b_spline<double>(
            steering_curve_points_.begin(),
            steering_curve_points_.end(),
            -1.0 + steering_deadzone_,
            (2.0 * (1.0 - steering_deadzone_)) /
                (steering_curve_points_.size() - 1),
            0.0,
            0.0);

    declare_parameter("debounce.slow", 3.0);
    slow_debounce_time_ = get_parameter("debounce.slow").as_double();

    declare_parameter("debounce.fast", 0.1);
    fast_debounce_time_ = get_parameter("debounce.fast").as_double();

    declare_parameter("rate", 100.0);
    rate_ = get_parameter("rate").as_double();
}

void GamepadInterfaceNode::Initialize() {
    last_ignition_request_time_ = get_clock()->now();
    last_starter_request_time_ = get_clock()->now();
    last_shifter_request_time_ = get_clock()->now();
    last_control_mode_cycle_time_ = get_clock()->now();
    last_control_mode_set_time_ = get_clock()->now();
    last_ready_to_run_time_ = get_clock()->now();
}

void GamepadInterfaceNode::Activate() {
    CreateSubscriptions();
    CreateClients();
    CreateTimers();
    CreatePublishers();
    StartTimers();
}

void GamepadInterfaceNode::CreateSubscriptions() {
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        1,
        std::bind(&GamepadInterfaceNode::JoyStateCallback,
                  this,
                  std::placeholders::_1));
}

void GamepadInterfaceNode::CreateClients() {
    start_engine_client_ =
        create_client<std_srvs::srv::Trigger>("start_engine");

    shift_gear_down_client_ =
        create_client<std_srvs::srv::Trigger>("shift_gear_down");

    shift_gear_up_client_ =
        create_client<std_srvs::srv::Trigger>("shift_gear_up");

    cycle_ignition_client_ =
        create_client<std_srvs::srv::Trigger>("cycle_ignition");

    cycle_control_mode_client_ =
        create_client<std_srvs::srv::Trigger>("cycle_control_mode");

    set_control_mode_client_ =
        create_client<olav_interfaces::srv::SetControlMode>("set_control_mode");

    emergency_stop_client_ =
        create_client<std_srvs::srv::Trigger>("emergency_stop");

    ready_to_run_client_ = create_client<std_srvs::srv::Trigger>("ready");
}

void GamepadInterfaceNode::CreateTimers() {
    controls_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        std::bind(&GamepadInterfaceNode::TimerCallback, this));
    controls_timer_->cancel();
}

void GamepadInterfaceNode::CreatePublishers() {
    throttle_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "throttle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    brake_publisher_ = create_publisher<olav_interfaces::msg::SetpointStamped>(
        "brake",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    ackermann_drive_publisher_ =
        create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    heartbeat_publisher_ = create_publisher<std_msgs::msg::Header>(
        "heartbeat",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void GamepadInterfaceNode::StartTimers() { controls_timer_->reset(); }

void GamepadInterfaceNode::JoyStateCallback(
    const sensor_msgs::msg::Joy::ConstSharedPtr joy_message) {
    auto time = get_clock()->now();

    auto throttle =
        1.0 - (joy_message->axes[GamepadAxes::RIGHT_TRIGGER] + 1.0) / 2.0;
    if(use_throttle_curve_) {
        throttle_ = (std::abs(throttle) > throttle_deadzone_)
            ? boost::algorithm::clamp(throttle_curve_(throttle), 0.0, 1.0)
            : 0.0;
    } else {
        throttle_ = throttle;
    }

    auto brake =
        1.0 - (joy_message->axes[GamepadAxes::LEFT_TRIGGER] + 1.0) / 2.0;
    if(use_brake_curve_) {
        brake_ = (std::abs(brake) > brake_deadzone_)
            ? boost::algorithm::clamp(brake_curve_(brake), 0.0, 1.0)
            : 0.0;
    } else {
        brake_ = brake;
    }

    auto steering = joy_message->axes[GamepadAxes::LEFT_STICK_HORIZONTAL];
    if(use_steering_curve_) {
        // Notice the negative sign, as we are inverting the axis (negative
        // steering to the left).
        steering_ = (std::abs(steering) > steering_deadzone_)
            ? -boost::algorithm::clamp(steering_curve_(steering), -1.0, 1.0)
            : 0.0;
    } else {
        steering_ = -steering;
    }

    // ... ignition on.
    if(joy_message->buttons[GamepadButtons::XBOX] &&
       (time - last_ignition_request_time_).seconds() > slow_debounce_time_) {
        RCLCPP_INFO(get_logger(), "Sending ignition state cycle request ...");

        last_ignition_request_time_ = time;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future = cycle_ignition_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::CycleIgnitionCallback,
                      this,
                      std::placeholders::_1));
    }

    // ... emergency.
    if(joy_message->buttons[GamepadButtons::B]) {
        RCLCPP_INFO(get_logger(), "Sending emergency stop request ...");

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future = emergency_stop_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::TriggerEmergencyCallback,
                      this,
                      std::placeholders::_1));
    }

    // ... engine starter state.
    if(joy_message->buttons[GamepadButtons::SHARE] &&
       (time - last_starter_request_time_).seconds() > slow_debounce_time_) {
        RCLCPP_INFO(get_logger(), "Sending engine start request ...");

        last_starter_request_time_ = time;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = start_engine_client_->async_send_request(request);
    }

    // ... gear downshift.
    if(joy_message->buttons[GamepadButtons::LEFT_BUMPER] &&
       (time - last_shifter_request_time_).seconds() > slow_debounce_time_) {
        RCLCPP_INFO(get_logger(), "Sending downshift request ...");

        last_shifter_request_time_ = time;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future = shift_gear_down_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::ShiftGearDownCallback,
                      this,
                      std::placeholders::_1));
    }

    // ... gear upshift.
    if(joy_message->buttons[GamepadButtons::RIGHT_BUMPER] &&
       (time - last_shifter_request_time_).seconds() > slow_debounce_time_) {
        RCLCPP_INFO(get_logger(), "Sending upshift request ...");

        last_shifter_request_time_ = time;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future = shift_gear_up_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::ShiftGearUpCallback,
                      this,
                      std::placeholders::_1));
    }

    if(joy_message->buttons[GamepadButtons::VIEW] &&
       (time - last_control_mode_cycle_time_).seconds() > fast_debounce_time_) {
        RCLCPP_INFO(get_logger(), "Sending control mode cycle request ...");

        last_control_mode_cycle_time_ = time;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future = cycle_control_mode_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::CycleControlModeCallback,
                      this,
                      std::placeholders::_1));
    }

    if(joy_message->axes[GamepadAxes::DPAD_HORIZONTAL] == -1.0 &&
       (time - last_control_mode_set_time_).seconds() > fast_debounce_time_) {
        RCLCPP_INFO(get_logger(),
                    "Sending control mode set to GAMEPAD request ...");

        last_control_mode_cycle_time_ = time;
        auto request =
            std::make_shared<olav_interfaces::srv::SetControlMode::Request>();
        request->mode = olav_interfaces::srv::SetControlMode::Request::GAMEPAD;
        const auto future = set_control_mode_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::SetControlModeCallback,
                      this,
                      std::placeholders::_1));
    }

    if(joy_message->axes[GamepadAxes::DPAD_HORIZONTAL] == 1.0 &&
       (time - last_control_mode_set_time_).seconds() > fast_debounce_time_) {
        RCLCPP_INFO(get_logger(),
                    "Sending control mode set to AUTONOMOUS request ...");

        last_control_mode_cycle_time_ = time;
        auto request =
            std::make_shared<olav_interfaces::srv::SetControlMode::Request>();
        request->mode =
            olav_interfaces::srv::SetControlMode::Request::AUTONOMOUS;
        const auto future = set_control_mode_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::SetControlModeCallback,
                      this,
                      std::placeholders::_1));
    }

    if(joy_message->axes[GamepadAxes::DPAD_VERTICAL] == -1.0 &&
       (time - last_control_mode_set_time_).seconds() > fast_debounce_time_) {
        RCLCPP_INFO(get_logger(),
                    "Sending control mode set to GENERIC_THROTTLE_BRAKE_STEER "
                    "request ...");

        last_control_mode_cycle_time_ = time;
        auto request =
            std::make_shared<olav_interfaces::srv::SetControlMode::Request>();
        request->mode = olav_interfaces::srv::SetControlMode::Request::
            GENERIC_THROTTLE_BRAKE_STEER;
        const auto future = set_control_mode_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::SetControlModeCallback,
                      this,
                      std::placeholders::_1));
    }

    if(joy_message->axes[GamepadAxes::DPAD_VERTICAL] == 1.0 &&
       (time - last_control_mode_set_time_).seconds() > fast_debounce_time_) {
        RCLCPP_INFO(get_logger(),
                    "Sending control mode set to GENERIC_SPEED_STEERING_ANGLE "
                    "request ...");

        last_control_mode_cycle_time_ = time;
        auto request =
            std::make_shared<olav_interfaces::srv::SetControlMode::Request>();
        request->mode = olav_interfaces::srv::SetControlMode::Request::
            GENERIC_SPEED_STEERING_ANGLE;
        const auto future = set_control_mode_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::SetControlModeCallback,
                      this,
                      std::placeholders::_1));
    }

    if(joy_message->buttons[GamepadButtons::MENU] &&
       (time - last_ready_to_run_time_).seconds() > slow_debounce_time_) {
        RCLCPP_INFO(get_logger(), "Sending ready to run request ...");

        last_ready_to_run_time_ = time;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future = ready_to_run_client_->async_send_request(
            request,
            std::bind(&GamepadInterfaceNode::ReadyToRunCallback,
                      this,
                      std::placeholders::_1));
    }
}

void GamepadInterfaceNode::TimerCallback() {
    double throttle = 0.0;
    double brake = 0.0;
    double steering = 0.0;

    {
        std::lock_guard<std::mutex> controls_lock(controls_mutex_);

        throttle = throttle_;
        brake = brake_;
        steering = steering_;
    }

    std_msgs::msg::Header heartbeat_message;
    heartbeat_message.frame_id = frame_id_;
    heartbeat_message.stamp = get_clock()->now();
    heartbeat_publisher_->publish(heartbeat_message);

    olav_interfaces::msg::SetpointStamped throttle_message;
    throttle_message.header.frame_id = frame_id_;
    throttle_message.setpoint = throttle;
    throttle_publisher_->publish(throttle_message);

    olav_interfaces::msg::SetpointStamped brake_message;
    brake_message.header.frame_id = frame_id_;
    brake_message.setpoint = brake;
    brake_publisher_->publish(brake_message);

    ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_message;
    ackermann_drive_message.header.frame_id = frame_id_;
    ackermann_drive_message.header.stamp = get_clock()->now();
    ackermann_drive_message.drive.steering_angle =
        steering * maximum_steering_angle_;
    ackermann_drive_publisher_->publish(ackermann_drive_message);
}

void GamepadInterfaceNode::ShiftGearDownCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Gear downshift request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Gear downshift request failed: %s",
                     response->message.c_str());
    }
}

void GamepadInterfaceNode::ShiftGearUpCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Gear upshift request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Gear upshift request failed: %s",
                     response->message.c_str());
    }
}

void GamepadInterfaceNode::CycleControlModeCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Control mode cycle request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Control mode cycle request failed: %s",
                     response->message.c_str());
    }
}

void GamepadInterfaceNode::SetControlModeCallback(
    rclcpp::Client<olav_interfaces::srv::SetControlMode>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Control mode set request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Control mode set request failed: %s",
                     response->message.c_str());
    }
}

void GamepadInterfaceNode::CycleIgnitionCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Ignition state cycle request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Ignition state cycle request request failed: %s",
                     response->message.c_str());
    }
}

void GamepadInterfaceNode::TriggerEmergencyCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Emergency stop request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Emergency stop request failed: %s",
                     response->message.c_str());
    }
}

void GamepadInterfaceNode::ReadyToRunCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto response = future.get();
    if(response->success) {
        RCLCPP_INFO(get_logger(),
                    "Ready to run request successful: %s",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Ready to run request failed: %s",
                     response->message.c_str());
    }
}

} // namespace ROS
} // namespace OLAV