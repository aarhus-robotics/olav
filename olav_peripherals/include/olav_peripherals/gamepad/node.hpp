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

#pragma once

#include <mutex>

#include <boost/algorithm/clamp.hpp>
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_interfaces/action/shift_gear.hpp>
#include <olav_interfaces/msg/throttle_brake_steering.hpp>
#include <olav_interfaces/srv/set_control_mode.hpp>

namespace OLAV {
namespace ROS {

enum GamepadState {
    STANDBY = -1,
    CONTROL_TRIGGER_TBS = 0,
    CONTROL_PAD_TBS = 1,
    CONTROL_PAD_DRIVE = 2
};

enum GamepadAxes {
    LEFT_STICK_HORIZONTAL = 0, // +1.0 full left, -1.0 full right
    LEFT_STICK_VERTICAL = 1, // +1.0 full up, -1.0 full down
    LEFT_TRIGGER = 2, // +1.0 fully release, -1.0 fully depressed
    RIGHT_STICK_HORIZONTAL = 3, // +1.0 full left, -1.0 full right
    RIGHT_STICK_VERTICAL = 4, // +1.0 full up, -1.0 full down
    RIGHT_TRIGGER = 5, // +1.0 fully released, -1.0 fully depressed
    DPAD_HORIZONTAL = 6, // +1 full left, -1 full right
    DPAD_VERTICAL = 7 // +1 full up, -1 full down
};

enum GamepadButtons {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    VIEW = 6,
    MENU = 7,
    XBOX = 8,
    LEFT_STICK = 9,
    RIGHT_STICK = 10,
    SHARE = 11
};

/**
 * @brief ROS node to interface a generic Xbox Series X-style controller with
 * the OLAV drive-by-wire node.
 */
class GamepadInterfaceNode : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new gamepad node.
     */
    GamepadInterfaceNode();

  protected:
    void Configure();

    void GetParameters();

    void Initialize();

    void Activate();

    void CreateSubscriptions();

    void CreateClients();

    void CreateTimers();

    void CreatePublishers();

    void StartTimers();

  private:
    /**
     * @brief Callback for the controls timer.
     */
    void TimerCallback();

    // Gamepad state
    // ------------------------------------------------------------------------
    /** @brief Shared pointer to the joystick subscription. */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    /**
     * @brief Callback for the ROS joystick state.
     *
     * @param joy_message Constant shared pointer to the ROS sensor_msgs/msg/Joy
     * message.
     */
    void
    JoyStateCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_message);

    // Control commands
    // ------------------------------------------------------------------------
    rclcpp::Publisher<olav_interfaces::msg::ThrottleBrakeSteering>::SharedPtr
        tbs_publisher_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_drive_publisher_;

    // Throttle control
    // ------------------------------------------------------------------------
    /** @brief Latest throttle effort. */
    double throttle_ = 0.0;

    /** @brief Throttle axis deadzone in absolute terms. */
    double throttle_deadzone_;

    /** @brief Whether or not to use cubic B-spline interpolation for throttle
     * efforts. */
    bool use_throttle_curve_;

    /** @brief Throttle effort curve basis spline evenly-spaced interpolation
     * points. */
    std::vector<double> throttle_curve_points_;

    /** @brief Throttle effort curve basis spline. */
    boost::math::interpolators::cardinal_cubic_b_spline<double> throttle_curve_;

    // Brake control
    // ------------------------------------------------------------------------
    /** @brief Latest brake effort. */
    double brake_ = 0.0;

    /** @brief Brake axis deadzone in absolute terms. */
    double brake_deadzone_;

    /** @brief Whether or not to use cubic B-spline interpolation for brake
     * efforts. */
    bool use_brake_curve_;

    /** @brief Brake effort curve basis spline evenly-spaced interpolation
     * points. */
    std::vector<double> brake_curve_points_;

    /** @brief Brake effort curve basis spline. */
    boost::math::interpolators::cardinal_cubic_b_spline<double> brake_curve_;
    // ------------------------------------------------------------------------

    // Steering control
    // ------------------------------------------------------------------------
    /** @brief Latest steering effort. */
    double steering_ = 0.0;

    /** @brief Steering axis deadzone in absolute terms and for a single
     * direction. */
    double steering_deadzone_;

    /** @brief Whether or not to use cubic B-spline interpolation for steering
     * efforts. */
    bool use_steering_curve_;

    /** @brief Steering effort curve basis spline evenly-spaced interpolation
     * points. */
    std::vector<double> steering_curve_points_;

    /** @brief Steering effort curve basis spline. */
    boost::math::interpolators::cardinal_cubic_b_spline<double> steering_curve_;
    // ------------------------------------------------------------------------

    rclcpp::Client<olav_interfaces::srv::SetControlMode>::SharedPtr
        set_control_mode_client_;

    // Start engine
    // ------------------------------------------------------------------------
    /** @brief Shared pointer to the engine start service client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_engine_client_;
    // ------------------------------------------------------------------------

    // Gear up/down shifter
    // ------------------------------------------------------------------------
    /** @brief Shared pointer to the downshift service client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shift_gear_down_client_;

    /**
     * @brief Callback for the gear downshift service response.
     *
     * @param future Shared pointer to the std_srvs/srv/Trigger request future.
     */
    void ShiftGearDownCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    /** @brief Shared pointer to the gear upshift service client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shift_gear_up_client_;

    /**
     * @brief Callback for the gear upshift service response.
     *
     * @param future Shared pointer to the std_srvs/srv/Trigger request future.
     */
    void ShiftGearUpCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    // ------------------------------------------------------------------------

    // Cycle ignition
    // ------------------------------------------------------------------------
    /** @brief Shared pointer to the cycle ignition state service client.  */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cycle_ignition_client_;

    void CycleIgnitionCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;
    // ------------------------------------------------------------------------

    // Trigger emergency stop
    // ------------------------------------------------------------------------
    void TriggerEmergencyCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    // ------------------------------------------------------------------------

    // Ready-to-run
    // ------------------------------------------------------------------------
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_to_run_client_;

    void ReadyToRunCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    // ------------------------------------------------------------------------

    // Datalogger start/stop
    // ------------------------------------------------------------------------
    /** @brief Shared pointer to the datalogger start service client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_datalogger_client_;

    /** @brief Shared pointer to the datalogger stop service client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_datalogger_client_;
    // ------------------------------------------------------------------------

    // Node parameters
    // ------------------------------------------------------------------------
    double rate_;

    std::string frame_id_ = "gamepad";

    rclcpp::TimerBase::SharedPtr controls_timer_;

    GamepadState state_ = GamepadState::STANDBY;
    // ------------------------------------------------------------------------

    // Thread safety
    // ------------------------------------------------------------------------
    std::mutex controls_mutex_;
    // ------------------------------------------------------------------------

    // Input handling
    // ------------------------------------------------------------------------

    void HandleTriggerRight(const double& position);

    void HandleTriggerLeft(const double& position);

    void HandleStickLeftHorizontal(const double& position);

    void HandleLeftBumper();

    void HandleRightBumper();

    void HandleDirectionalPadHorizontal(const int& position);

    void HandleDirectionalPadVertical(const int& position);

    void HandleXboxButton();

    void HandleMenuButton();

    void HandleShareButton();

    void HandleViewButton();

    void HandleXButton();

    void HandleYButton();

    void HandleAButton();

    void HandleBButton();

    void ZeroTrims();

    bool is_right_modifier_pressed_ = false;

    bool is_left_modifier_pressed_ = false;

    double shift_multiplier_;

    // Debouncing
    // ------------------------------------------------------------------------

    bool CheckDebounce(rclcpp::Time& last_press_time, const double& period);

    /** @brief Last parsed ignition request time stamp. */
    rclcpp::Time last_ignition_request_time_;

    /** @brief Last parsed engine start request time stamp. */
    rclcpp::Time last_starter_request_time_;

    double starter_debounce_period_;

    /** @brief Last parsed gear shift request time stamp. */
    rclcpp::Time last_shifter_request_time_;

    /** @brief Debounce interval for the gear shifting bumpers. */
    double shifter_debounce_period_;

    rclcpp::Time last_button_time_;

    double button_debounce_period_;

    rclcpp::Time last_dpad_time_;

    /** @brief Debounce interval in seconds. */
    double dpad_debounce_period_ = 0.1;

    double service_button_debounce_period_;

    rclcpp::Time last_service_call_time_;

    rclcpp::Time last_rumble_time_;

    // ------------------------------------------------------------------------

    // D-Pad controls
    // ------------------------------------------------------------------------
    double control_vertical_magnitude_ = 0.0;
    double control_vertical_lower_bound_;
    double control_vertical_upper_bound_;
    double control_vertical_step_ = 0.01;

    double control_horizontal_magnitude_ = 0.0;
    double control_horizontal_lower_bound_;
    double control_horizontal_upper_bound_;
    double control_horizontal_step_ = 0.01;

    std::tuple<double, double, double, double> throttle_pad_settings_;
    std::tuple<double, double, double, double> steering_pad_settings_;
    std::tuple<double, double, double, double> speed_pad_settings_;

    void ScheduleRumble();

    void ClampRange(const double& lower_bound,
                    const double& upper_bound,
                    double& value);

    // ------------------------------------------------------------------------

    // Force feedback
    // ------------------------------------------------------------------------
    /** @brief Shared pointer to the force feedback publisher. */
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr
        force_feedback_publisher_;

    /**
     * @brief Callback for the force feedback timer.
     */
    void FeedbackCallback();

    /** @brief Whether or not the gamepad should rumble throughout the current
     *         timer tick. */
    bool must_rumble_ = false;

    bool is_rumbling_ = false;

    /** @brief Shared pointer to the force feedback timer. */
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    double force_feedback_intensity_;

    double force_feedback_period_;
    // ------------------------------------------------------------------------

    void PublishThrottleBrakeSteering(const double& throttle,
                                      const double& brake,
                                      const double& steering);
};

} // namespace ROS
} // namespace OLAV