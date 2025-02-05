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
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_interfaces/action/shift_gear.hpp>
#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <olav_interfaces/srv/set_control_mode.hpp>

namespace OLAV {
namespace ROS {

enum GamepadAxes {
    LEFT_STICK_HORIZONTAL = 0, // +1.0 full left, -1.0 full right
    LEFT_STICK_VERTICAL = 1, // +1.0 full up, -1.0 full down
    LEFT_TRIGGER = 2, // +1.0 fully release, -1.0 fully depressed
    RIGHT_STICK_HORIZONTAL = 3, // +1.0 full left, -1.0 full right
    RIGHT_STICK_VERTICAL = 4, // +1.0 full up, -1.0 full down
    RIGHT_TRIGGER = 5, // +1.0 fully released, -1.0 fully depressed
    DPAD_HORIZONTAL = 6, // +1 full left, -1 full right
    DPAD_VERTICAL = 7 // +1 full left, -1 full right
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

    /**
     * @brief Callback for the ROS joystick state.
     *
     * @param joy_message Constant shared pointer to the ROS sensor_msgs/msg/Joy
     * message.
     */
    void
    JoyStateCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_message);

    /** @brief Shared pointer to the joystick subscription. */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_publisher_;

    // Controls
    // --------

    /** @brief Shared pointer to the throttle effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_publisher_;

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

    /** @brief Shared pointer to the brake effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_publisher_;

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

    /** @brief Shared pointer to the steering effort publisher. */
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_drive_publisher_;

    /** @brief Maximum steering angle achievable via the controller. */
    double maximum_steering_angle_;

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

    // Services
    // --------

    /** @brief Controller service buttons debounce interval in seconds. */
    double slow_debounce_time_;

    /** @brief Controller generic buttons debounce interval in seconds. */
    double fast_debounce_time_;

    /** @brief Shared pointer to the engine start service client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_engine_client_;

    /** @brief Last parsed engine start request time stamp. */
    rclcpp::Time last_starter_request_time_;

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

    /** @brief Last parsed gear shift request time stamp. */
    rclcpp::Time last_shifter_request_time_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        cycle_control_mode_client_;

    rclcpp::Time last_control_mode_cycle_time_;

    void CycleControlModeCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<olav_interfaces::srv::SetControlMode>::SharedPtr
        set_control_mode_client_;

    rclcpp::Time last_control_mode_set_time_;

    void SetControlModeCallback(
        rclcpp::Client<olav_interfaces::srv::SetControlMode>::SharedFuture
            future);

    rclcpp::TimerBase::SharedPtr controls_timer_;

    /** @brief Last parsed ignition request time stamp. */
    rclcpp::Time last_ignition_request_time_;

    /** @brief Shared pointer to the cycle ignition state service client.  */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cycle_ignition_client_;

    void CycleIgnitionCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;

    void TriggerEmergencyCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_to_run_client_;

    rclcpp::Time last_ready_to_run_time_;

    void ReadyToRunCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    std::mutex controls_mutex_;

    double rate_;

    std::string frame_id_ = "gamepad";
};

} // namespace ROS
} // namespace OLAV