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

    declare_parameter("debounce.slow", 2.0);
    shifter_debounce_period_ = get_parameter("debounce.slow").as_double();

    declare_parameter("debounce.button", 0.15);
    button_debounce_period_ = get_parameter("debounce.button").as_double();

    declare_parameter("debounce.service_button", 0.3);
    service_button_debounce_period_ =
        get_parameter("debounce.service_button").as_double();

    declare_parameter("debounce.starter", 3.0);
    starter_debounce_period_ = get_parameter("debounce.starter").as_double();

    declare_parameter("rate", 100.0);
    rate_ = get_parameter("rate").as_double();

    declare_parameter("multiplier", 10.0);
    shift_multiplier_ = get_parameter("multiplier").as_double();

    declare_parameter("feedback.intensity", 0.01);
    force_feedback_intensity_ = get_parameter("feedback.intensity").as_double();

    declare_parameter("feedback.period", 0.2);
    force_feedback_period_ = get_parameter("feedback.period").as_double();
}

void GamepadInterfaceNode::Initialize() {
    last_ignition_request_time_ = get_clock()->now();
    last_starter_request_time_ = get_clock()->now();
    last_shifter_request_time_ = get_clock()->now();
    last_dpad_time_ = get_clock()->now();

    last_button_time_ = get_clock()->now();
    last_rumble_time_ = get_clock()->now();
    last_service_call_time_ = get_clock()->now();

    throttle_pad_settings_ = std::make_tuple(-1.0, 1.0, 0.01, 5.0);
    steering_pad_settings_ = std::make_tuple(-33.0, 33.0, 0.25, 4.0);
    speed_pad_settings_ = std::make_tuple(0.0, 10.0, 0.5, 2.0);

    control_horizontal_lower_bound_ = std::get<0>(steering_pad_settings_);
    control_horizontal_upper_bound_ = std::get<1>(steering_pad_settings_);
    control_horizontal_step_ = std::get<2>(steering_pad_settings_);
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

    emergency_stop_client_ =
        create_client<std_srvs::srv::Trigger>("emergency_stop");

    ready_to_run_client_ = create_client<std_srvs::srv::Trigger>("ready");

    set_control_mode_client_ =
        create_client<olav_interfaces::srv::SetControlMode>("set_control_mode");

    start_datalogger_client_ =
        create_client<std_srvs::srv::Trigger>("datalogger/start");

    stop_datalogger_client_ =
        create_client<std_srvs::srv::Trigger>("datalogger/stop");
}

void GamepadInterfaceNode::CreateTimers() {
    controls_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        std::bind(&GamepadInterfaceNode::TimerCallback, this));
    controls_timer_->cancel();

    feedback_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        std::bind(&GamepadInterfaceNode::FeedbackCallback, this));
    feedback_timer_->cancel();
}

void GamepadInterfaceNode::CreatePublishers() {
    tbs_publisher_ =
        create_publisher<olav_interfaces::msg::ThrottleBrakeSteering>(
            "tbs",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    ackermann_drive_publisher_ =
        create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    force_feedback_publisher_ = create_publisher<sensor_msgs::msg::JoyFeedback>(
        "feedback",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void GamepadInterfaceNode::StartTimers() {
    controls_timer_->reset();
    feedback_timer_->reset();
}

void GamepadInterfaceNode::HandleTriggerRight(const double& position) {
    auto throttle = 1.0 - (position + 1.0) / 2.0;
    if(use_throttle_curve_) {
        throttle_ = (std::abs(throttle) > throttle_deadzone_)
            ? boost::algorithm::clamp(throttle_curve_(throttle), 0.0, 1.0)
            : 0.0;
    } else {
        throttle_ = throttle;
    }
}

void GamepadInterfaceNode::HandleTriggerLeft(const double& position) {
    auto brake = 1.0 - (position + 1.0) / 2.0;
    if(use_brake_curve_) {
        brake_ = (std::abs(brake) > brake_deadzone_)
            ? boost::algorithm::clamp(brake_curve_(brake), 0.0, 1.0)
            : 0.0;
    } else {
        brake_ = brake;
    }
}

void GamepadInterfaceNode::HandleStickLeftHorizontal(const double& position) {
    auto steering = position;
    if(use_steering_curve_) {
        // Notice the negative sign, as we are inverting the axis (negative
        // steering to the left).
        steering_ = (std::abs(steering) > steering_deadzone_)
            ? -boost::algorithm::clamp(steering_curve_(steering), -1.0, 1.0)
            : 0.0;
    } else {
        steering_ = -steering;
    }
}

void GamepadInterfaceNode::JoyStateCallback(
    const sensor_msgs::msg::Joy::ConstSharedPtr joy_message) {
    auto time = get_clock()->now();

    is_left_modifier_pressed_ =
        bool(joy_message->buttons[GamepadButtons::LEFT_STICK]);
    is_right_modifier_pressed_ =
        bool(joy_message->buttons[GamepadButtons::RIGHT_STICK]);

    HandleTriggerRight(joy_message->axes[GamepadAxes::RIGHT_TRIGGER]);
    HandleTriggerLeft(joy_message->axes[GamepadAxes::LEFT_TRIGGER]);
    HandleStickLeftHorizontal(
        joy_message->axes[GamepadAxes::LEFT_STICK_HORIZONTAL]);

    if(joy_message->buttons[GamepadButtons::LEFT_BUMPER]) HandleLeftBumper();
    if(joy_message->buttons[GamepadButtons::RIGHT_BUMPER]) HandleRightBumper();
    if(joy_message->axes[GamepadAxes::DPAD_HORIZONTAL])
        HandleDirectionalPadHorizontal(
            joy_message->axes[GamepadAxes::DPAD_HORIZONTAL]);
    if(joy_message->axes[GamepadAxes::DPAD_VERTICAL])
        HandleDirectionalPadVertical(
            joy_message->axes[GamepadAxes::DPAD_VERTICAL]);
    if(joy_message->buttons[GamepadButtons::XBOX]) HandleXboxButton();
    if(joy_message->buttons[GamepadButtons::MENU]) HandleMenuButton();
    if(joy_message->buttons[GamepadButtons::SHARE]) HandleShareButton();
    if(joy_message->buttons[GamepadButtons::VIEW]) HandleViewButton();
    if(joy_message->buttons[GamepadButtons::X]) HandleXButton();
    if(joy_message->buttons[GamepadButtons::Y]) HandleYButton();
    if(joy_message->buttons[GamepadButtons::A]) HandleAButton();
    if(joy_message->buttons[GamepadButtons::B]) HandleBButton();
}

bool GamepadInterfaceNode::CheckDebounce(rclcpp::Time& last_press_time,
                                         const double& period) {
    const auto current_time = get_clock()->now();
    if((current_time - last_press_time).seconds() > period) {
        last_press_time = current_time;
        return true;
    }
    return false;
}

void GamepadInterfaceNode::HandleLeftBumper() {
    if(!CheckDebounce(last_shifter_request_time_, shifter_debounce_period_))
        return;

    ScheduleRumble();

    RCLCPP_INFO(get_logger(), "Sending downshift request ...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = shift_gear_down_client_->async_send_request(
        request,
        std::bind(&GamepadInterfaceNode::ShiftGearDownCallback,
                  this,
                  std::placeholders::_1));
}

void GamepadInterfaceNode::HandleRightBumper() {
    if(!CheckDebounce(last_shifter_request_time_, shifter_debounce_period_))
        return;

    ScheduleRumble();

    RCLCPP_INFO(get_logger(), "Sending upshift request ...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = shift_gear_up_client_->async_send_request(
        request,
        std::bind(&GamepadInterfaceNode::ShiftGearUpCallback,
                  this,
                  std::placeholders::_1));
}

void GamepadInterfaceNode::HandleDirectionalPadHorizontal(const int& position) {
    if(!CheckDebounce(last_dpad_time_, dpad_debounce_period_)) return;

    if(state_ == GamepadState::STANDBY ||
       state_ == GamepadState::CONTROL_TRIGGER_TBS) {
        RCLCPP_WARN(
            get_logger(),
            "Directional pad commands are not available outside PAD modes!");
        return;
    }

    control_horizontal_magnitude_ += (is_right_modifier_pressed_)
        ? shift_multiplier_ * (-position * control_horizontal_step_)
        : -position * control_horizontal_step_;

    ClampRange(control_horizontal_lower_bound_,
               control_horizontal_upper_bound_,
               control_horizontal_magnitude_);

    RCLCPP_INFO(get_logger(),
                "New horizontal value set {%0.2f}",
                control_horizontal_magnitude_);
}

void GamepadInterfaceNode::ClampRange(const double& lower_bound,
                                      const double& upper_bound,
                                      double& value) {
    value = boost::algorithm::clamp(value, lower_bound, upper_bound);
}

void GamepadInterfaceNode::HandleDirectionalPadVertical(const int& position) {
    if(!CheckDebounce(last_dpad_time_, dpad_debounce_period_)) return;

    if(state_ == GamepadState::STANDBY ||
       state_ == GamepadState::CONTROL_TRIGGER_TBS) {
        RCLCPP_WARN(
            get_logger(),
            "Directional pad commands are not available outside PAD modes!");
        return;
    }

    control_vertical_magnitude_ += (is_right_modifier_pressed_)
        ? shift_multiplier_ * (position * control_vertical_step_)
        : position * control_vertical_step_;

    ClampRange(control_vertical_lower_bound_,
               control_vertical_upper_bound_,
               control_vertical_magnitude_);

    RCLCPP_INFO(get_logger(),
                "New vertical value set {%0.2f}",
                control_vertical_magnitude_);
}

void GamepadInterfaceNode::HandleMenuButton() {
    if(!CheckDebounce(last_starter_request_time_, starter_debounce_period_))
        return;

    ScheduleRumble();

    RCLCPP_INFO(get_logger(), "Sending engine start request ...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = start_engine_client_->async_send_request(request);
}

void GamepadInterfaceNode::ZeroTrims() {
    control_vertical_magnitude_ = 0.0;
    control_horizontal_magnitude_ = 0.0;
}

void GamepadInterfaceNode::HandleShareButton() {
    if(!CheckDebounce(last_service_call_time_, service_button_debounce_period_))
        return;

    ScheduleRumble();

    ZeroTrims();

    if(is_right_modifier_pressed_) {
        auto request =
            std::make_shared<olav_interfaces::srv::SetControlMode::Request>();
        request->mode =
            olav_interfaces::srv::SetControlMode::Request::MODE_DRIVE_ACKERMANN;
        request->authority =
            olav_interfaces::srv::SetControlMode::Request::AUTHORITHY_GAMEPAD;
        const auto future =
            set_control_mode_client_->async_send_request(request);

        RCLCPP_INFO(get_logger(),
                    "Sending drive-by-wire set control mode request to: >> "
                    "MODE: DRIVE_ACKERMANN <> AUTHORITY: GAMEPAD << ...");

        // Move this to the future function.
        RCLCPP_INFO(get_logger(),
                    "Setting gamepad state to >> CONTROL_PAD_DRIVE <<");

        control_vertical_lower_bound_ = std::get<0>(speed_pad_settings_);
        control_vertical_upper_bound_ = std::get<1>(speed_pad_settings_);
        control_vertical_step_ = std::get<2>(speed_pad_settings_);

        state_ = CONTROL_PAD_DRIVE;

        return;
    }

    RCLCPP_INFO(get_logger(),
                "Sending drive-by-wire set control mode request to: >> "
                "MODE: MODE_DRIVE_TBS <> AUTHORITY: GAMEPAD << ...");

    auto request =
        std::make_shared<olav_interfaces::srv::SetControlMode::Request>();
    request->mode =
        olav_interfaces::srv::SetControlMode::Request::MODE_DRIVE_TBS;
    request->authority =
        olav_interfaces::srv::SetControlMode::Request::AUTHORITHY_GAMEPAD;
    const auto future = set_control_mode_client_->async_send_request(request);

    // Move this to the future function.
    state_ = is_left_modifier_pressed_ ? CONTROL_PAD_TBS : CONTROL_TRIGGER_TBS;

    if(is_left_modifier_pressed_) {
        RCLCPP_INFO(get_logger(),
                    "Setting gamepad state to >> CONTROL_PAD_TBS <<");

        control_vertical_lower_bound_ = std::get<0>(throttle_pad_settings_);
        control_vertical_upper_bound_ = std::get<1>(throttle_pad_settings_);
        control_vertical_step_ = std::get<2>(throttle_pad_settings_);

    } else {
        RCLCPP_INFO(get_logger(),
                    "Setting gamepad state to >> CONTROL_TRIGGER_TBS <<");
    }
}

void GamepadInterfaceNode::HandleXboxButton() {
    if(!CheckDebounce(last_service_call_time_, service_button_debounce_period_))
        return;

    RCLCPP_INFO(get_logger(), "Sending ready to run request ...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = ready_to_run_client_->async_send_request(
        request,
        std::bind(&GamepadInterfaceNode::ReadyToRunCallback,
                  this,
                  std::placeholders::_1));

    ScheduleRumble();
}

void GamepadInterfaceNode::HandleXButton() {
    if(!CheckDebounce(last_button_time_, button_debounce_period_)) return;

    RCLCPP_INFO(get_logger(), "Zeroing trims ...");

    control_horizontal_magnitude_ = 0.0;
    control_vertical_magnitude_ = 0.0;

    ScheduleRumble();
}

void GamepadInterfaceNode::HandleYButton() {
    if(!CheckDebounce(last_service_call_time_, service_button_debounce_period_))
        return;

    ScheduleRumble();

    if(!is_left_modifier_pressed_) {
        RCLCPP_INFO(get_logger(), "Sending datalogger start request ...");

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const auto future =
            start_datalogger_client_->async_send_request(request);
        return;
    }

    RCLCPP_INFO(get_logger(), "Sending datalogger stop request ...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = stop_datalogger_client_->async_send_request(request);
}

void GamepadInterfaceNode::ScheduleRumble() {
    last_rumble_time_ = get_clock()->now();
    must_rumble_ = true;
}

void GamepadInterfaceNode::HandleAButton() {
    if(!CheckDebounce(last_button_time_, button_debounce_period_)) return;

    RCLCPP_INFO(get_logger(),
                "Setting values: {%0.2f}, {%0.2f}",
                control_horizontal_magnitude_,
                control_vertical_magnitude_);

    ScheduleRumble();
}

void GamepadInterfaceNode::HandleBButton() {
    if(!CheckDebounce(last_service_call_time_, service_button_debounce_period_))
        return;

    RCLCPP_INFO(get_logger(), "Sending emergency stop request ...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = emergency_stop_client_->async_send_request(
        request,
        std::bind(&GamepadInterfaceNode::TriggerEmergencyCallback,
                  this,
                  std::placeholders::_1));

    ScheduleRumble();
}

void GamepadInterfaceNode::HandleViewButton() {
    if(!CheckDebounce(last_ignition_request_time_, shifter_debounce_period_))
        return;

    ScheduleRumble();

    RCLCPP_INFO(get_logger(), "Sending ignition state cycle request ...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = cycle_ignition_client_->async_send_request(
        request,
        std::bind(&GamepadInterfaceNode::CycleIgnitionCallback,
                  this,
                  std::placeholders::_1));
}

void GamepadInterfaceNode::FeedbackCallback() {
    // if(!must_rumble_) return;

    sensor_msgs::msg::JoyFeedback message;

    if((get_clock()->now() - last_rumble_time_).seconds() <
       force_feedback_period_) {
        message.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
        message.intensity = force_feedback_intensity_;
    } else {
        message.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
        message.intensity = 0.0;
        must_rumble_ = false;
    }

    force_feedback_publisher_->publish(message);
}

void GamepadInterfaceNode::PublishThrottleBrakeSteering(
    const double& throttle,
    const double& brake,
    const double& steering) {
    olav_interfaces::msg::ThrottleBrakeSteering tbs_message;

    tbs_message.header.frame_id = frame_id_;
    tbs_message.header.stamp = get_clock()->now();

    tbs_message.throttle = throttle;
    tbs_message.brake = brake;
    tbs_message.steering = steering;

    tbs_publisher_->publish(tbs_message);
}

void GamepadInterfaceNode::TimerCallback() {
    double throttle;
    double brake;
    double steering;

    if(state_ == GamepadState::STANDBY) {
        PublishThrottleBrakeSteering(0.0, 0.0, 0.0);
    } else if(state_ == GamepadState::CONTROL_TRIGGER_TBS) {
        {
            std::lock_guard<std::mutex> controls_lock(controls_mutex_);

            throttle = throttle_;
            brake = brake_;
            steering = steering_;
        }

        PublishThrottleBrakeSteering(throttle, brake, steering);

    } else if(state_ == GamepadState::CONTROL_PAD_TBS) {
        {
            std::lock_guard<std::mutex> controls_lock(controls_mutex_);

            throttle = (control_vertical_magnitude_ >= 0)
                ? control_vertical_magnitude_
                : 0.0;
            brake = (control_vertical_magnitude_ < 0)
                ? -control_vertical_magnitude_
                : 0.0;
            ;
            steering = control_horizontal_magnitude_;
        }

        PublishThrottleBrakeSteering(throttle, brake, steering);

    } else if(state_ == GamepadState::CONTROL_PAD_DRIVE) {
        {
            std::lock_guard<std::mutex> controls_lock(controls_mutex_);

            ackermann_msgs::msg::AckermannDriveStamped ackermann_message;
            ackermann_message.header.frame_id = frame_id_;
            ackermann_message.header.stamp = get_clock()->now();
            ackermann_message.drive.speed = control_vertical_magnitude_;
            ackermann_message.drive.steering_angle =
                control_horizontal_magnitude_;
            ackermann_drive_publisher_->publish(ackermann_message);
        }
    } else {
        // TODO: Handle error.
        return;
    }
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