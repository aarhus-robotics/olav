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

#include <boost/algorithm/clamp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <olav_interfaces/srv/set_control_mode.hpp>

namespace OLAV {
namespace ROS {

/**
 * @brief Class defining an rclcpp ROS node for control multiplexing. The ROS
 * node is repsonsible for validating the magnitude and source of the control
 * commands passed to the drive-by-wire PLC interface, and provides the system
 * and users with interfaces to change the active control authority. Finally,
 * the class provides automatic safety overrides irrespective of the active
 * control authority.
 *
 */
class ControlMultiplexerNode : public rclcpp::Node {
  public:
    /**
     * @brief Enumeration type defining the multiplexer output control mode.
     */
    enum ControlMode {
        /** @brief Control output is a triad of throttle, brake and steering
         * actuator efforts, routed to separate topics. */
        MANUAL = 0,

        /** @brief Control output is target speed (and optional acceleration and
         * jerk) and steering angle (and optional steering rate), routed to a
         * single ackermann_msgs/msg/AckermannDriveStamped topic. */
        AUTONOMOUS = 1
    };

    std::string FromControlMode(ControlMode mode) {
        if(mode == ControlMode::MANUAL) {
            return std::string("manual");
        } else if(mode == ControlMode::AUTONOMOUS) {
            return std::string("autonomous");
        } else {
            throw std::invalid_argument("Invalid control mode.");
        }
    }

    ControlMode ToControlMode(std::string mode) {
        if(mode == "manual") {
            return ControlMode::MANUAL;
        } else if(mode == "autonomous") {
            return ControlMode::AUTONOMOUS;
        } else {
            throw std::invalid_argument("Invalid control mode");
        }
    }

    /**
     * @brief Construct a new control multiplexer node.
     */
    ControlMultiplexerNode();

  private:
    void Configure();

    void GetParameters();

    void Initialize();

    void Activate();

    void CreateSubscriptions();

    void CreateServices();

    void CreateClients();

    void CreateTimers();

    void CreatePublishers();

    void StartTimers();

    void SetMode(ControlMode mode);

    void CycleControlMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void CreateSetpointSubscriptions();

    /**
     * @brief Callback for the command multiplexer node autonomous navigation
     * system throttle effort.
     *
     * @param throttle_message Autonomous navigation system throttle effort
     * message.
     */
    void ThrottleCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr throttle_message);

    /**
     * @brief Callback for the command multiplexer node autonomous navigation
     * system brake effort.
     *
     * @param brake_message Autonomous navigation system brake effort
     * message.
     */
    void BrakeCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr brake_message);

    /**
     * @brief Callback for the command multiplexer node autonomous navigation
     * system steering effort.
     *
     * @param steering_message Autonomous navigation system steering effort
     * message.
     */
    void SteeringCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr steering_message);

    void
    AckermannDriveCallback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
                               ackermann_drive_message);

    void HeartbeatCallback(
        const std_msgs::msg::Header::ConstSharedPtr heartbeat_message);

    void EmergencyStopCallback(
        const std_msgs::msg::Bool::ConstSharedPtr heartbeat_message);

    void SetControlMode(
        const std::shared_ptr<olav_interfaces::srv::SetControlMode::Request>
            request,
        std::shared_ptr<olav_interfaces::srv::SetControlMode::Response>
            response);

    /**
     * @brief Callback for the set parameters service.
     *
     * @param parameters
     * @return rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult
    SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    /** @brief Shared pointer to the autonomous navigation system throttle
     * effort subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_subscription_;

    /** @brief Shared pointer to the autonomous navigation system brake effort
     * subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_subscription_;

    /** @brief Shared pointer to the autonomous navigation system steering
     * effort subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_subscription_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_drive_subscription_;

    /** @brief Shared pointer to the autonomous navigation system heartbeat
     * signal subscription. */
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr
        heartbeat_subscription_;

    /** @brief Shared pointer to the autonomy switch subscription. */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        autonomy_switch_subscription_;

    bool autonomy_switch_ = false;

    /** @brief Shared pointer to the emergency stop subscription. */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        emergency_stop_subscription_;

    bool emergency_stop_ = false;

    /** @brief Shared pointer to the the muxed throttle effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_publisher_;

    /** @brief Shared pointer to the the muxed brake effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_publisher_;

    /** @brief Shared pointer to the the muxed steering effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_publisher_;

    /** @brief Shared pointer to the the muxed heartbeat signal publisher. */
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_publisher_;

    // Service servers
    // ---------------
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        cycle_control_mode_service_;

    rclcpp::Service<olav_interfaces::srv::SetControlMode>::SharedPtr
        set_control_mode_service_;
    // ---------------

    // Service clients
    // ---------------
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        start_speed_controller_client_;

    void StartSpeedControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        stop_speed_controller_client_;

    void StopSpeedControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        start_steering_controller_client_;

    void StartSteeringControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        stop_steering_controller_client_;

    void StopSteeringControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    // ---------------

    /** @brief Shared pointer to the set parameters callback handle. */
    OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_;

    /** @brief Manual brake effort threshold value before the autonomous
     * navigation system brake effort is overriden. */
    double brake_threshold_;

    std::atomic<ControlMode> active_control_mode_;

    bool warn_once_;

    // Thread safety
    // -------------

    /** @brief Shared pointer to a shared time mutex for reading and setting the
     * active control mode. */
    mutable std::shared_timed_mutex control_mode_mutex_;

    /** @brief Shared pointer to the mutually exclusive callback group for
     * synchronous services callbacks. */
    rclcpp::CallbackGroup::SharedPtr services_callback_group_;

    /** @brief Shared pointer to the reentrant callback group for asynchronous
     * subscriptions callback. */
    rclcpp::CallbackGroup::SharedPtr subscriptions_callback_group_;

    // Input validation
    // ----------------
    /**
     * @brief Whether or not a warning is logged when the issued controls do not
     * pass the bounds check.
     */
    bool warn_on_bounds_violations_;

    /**
     * @brief The maximum allowed throttle effort magnitude before throttle is
     * clamped.
     */
    double maximum_throttle_effort_;

    /**
     * @brief Check whether the provided throttle effort is within bounds of the
     * compact [0, max_throttle].
     *
     * @param throttle_effort Throttle effort to be checked.
     * @return true The throttle effort is within bounds.
     * @return false The throttle effort is out of bounds.
     */
    bool IsValidThrottleEffort(const double& throttle_effort);

    /**
     * @brief The maximum allowed brake effort magnitude before brake is
     * clamped.
     */
    double maximum_brake_effort_;

    /**
     * @brief Check whether the provided brake effort is within bounds of the
     * compact [0, max_brake].
     *
     * @param brake_effort Brake effort to be checked.
     * @return true The brake effort is within bounds.
     * @return false The brake effort is out of bounds.
     */
    bool IsValidBrakeEffort(const double& brake_effort);

    /**
     * @brief The maximum allowed steering effort magnitude before steering is
     * clamped.
     *
     */
    double maximum_steering_effort_;

    /**
     * @brief Check whether the provided steering effort is within bounds of the
     * compact
     *        [-max_steering_effort, max_steering_effort].
     *
     * @param steering_effort Steering effort to be checked.
     * @return true The steering effort is within bounds.
     * @return false The steering effort is out of bounds.
     */

    bool IsValidSteeringEffort(const double& steering_effort);

    /**
     * @brief The maximum allowed speed setpoint before speed is clamped.
     */
    double maximum_speed_setpoint_;

    /**
     * @brief Check whether the provided speed setpoint is below the maximum
     * allowed speed setpoint.
     *
     * @param speed_setpoint Speed setpoint to be checked.
     * @return true The speed setpoint is within bounds.
     * @return false The speed setpoint is out of bounds.
     */

    bool IsValidSpeedSetpoint(const double& speed_setpoint);

    /**
     * @brief The maximum steering angle setpoint before the steering angle
     * setpoint is clamped.
     */
    double maximum_steering_angle_setpoint_;

    /**
     * @brief Check whether the provided steering angle setpoint is within
     * bounds of the compact
     *        [-max_steering_angle, max_steering angle].
     *
     * @param steering_angle_setpoint Steering angle setpoint to be checked.
     * @return true The steering angle setpoint is within bounds.
     * @return false The steering angle setpoint is out of bounds.
     */
    bool IsValidSteeringAngleSetpoint(const double& steering_angle_setpoint);
    // ----------------

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_publisher_;

    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    void DiagnosticTimerCallback();
};

} // namespace ROS
} // namespace OLAV