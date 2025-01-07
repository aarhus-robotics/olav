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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_control/core/controllers/pid.hpp>
#include <olav_control/core/math/ramp_function.hpp>
#include <olav_control/speed/cvt_feedforward_model.hpp>
#include <olav_interfaces/msg/pid_status.hpp>
#include <olav_interfaces/msg/setpoint_stamped.hpp>

namespace OLAV {
namespace ROS {

enum FeedforwardModel { CONSTANT = 0, PROPORTIONAL = 1, CVT = 2 };

class SpeedControllerNode : public rclcpp::Node {
  public:
    SpeedControllerNode();

  private:
    void Configure();

    void Initialize();

    void GetParameters();

    void Activate();

    void CreateSubscriptions();

    void CreateTimers();

    void CreateServices();

    void CreatePublishers();

    void StartTimers();

    rcl_interfaces::msg::SetParametersResult
    SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void Reset();

    void
    TargetSpeedCallback(const olav_interfaces::msg::SetpointStamped::SharedPtr
                            target_speed_message);

    void
    OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

    void ControlTimerCallback();

    void StartServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void StopServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void ResetServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Shared pointer to the longitudinal PID controller. */
    std::shared_ptr<PIDController> controller_;

    FeedforwardModel feedforward_model_;

    double feedforward_offset_;

    std::shared_ptr<CVTFeedforwardModel> cvt_model_;

    /** @brief PID controller tick rate in Hz. */
    double control_rate_;

    /** @brief PID controller feedforward gain. */
    double feedforward_gain_;

    /** @brief PID controller proportional gain. */
    double proportional_gain_;

    /** @brief PID controller integral gain. */
    double integral_gain_;

    /** @brief PID controller derivative gain. */
    double derivative_gain_;

    /** @brief PID controller minimum output. */
    double minimum_output_;

    /** @brief PID controller maximum output. */
    double maximum_output_;

    /** @brief Whether or not the controller limits the maximum output change
     * between ticks. */
    bool use_output_change_limiter_;

    /** @brief PID controller maximum output change between ticks. */
    double maximum_output_change_;

    /** @brief Whether or not to use setpoint ramping. */
    bool use_setpoint_ramping_;

    /** @brief PID controller maximum setpoint ramp between ticks. */
    double maximum_setpoint_change_;

    /** @brief Whether or not the controller limits the integral term. */
    bool use_integral_term_limiter_;

    /** @brief PID controller maximum integral term magnitude. */
    double maximum_integral_term_;

    /** @brief Has a valid setpoint been received since the last reset? */
    bool has_setpoint_;

    /** @brief Has a valid feedback been received since the last reset? */
    bool has_feedback_;

    /** @brief Is the speed controller active? */
    bool is_stopped_;

    /** @brief Shared pointer to the odometry feedback subscription. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    /** @brief Shared pointer to the target speed subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        target_speed_subscription_;

    /** @brief Shared pointer to the control timer. */
    rclcpp::TimerBase::SharedPtr control_timer_;

    /** @brief Shared pointer to the throttle effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_publisher_;

    /** @brief Shared pointer to the brake effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_publisher_;

    /** @brief Shared pointer to the PID status publisher. */
    rclcpp::Publisher<olav_interfaces::msg::PIDStatus>::SharedPtr
        status_publisher_;

    // Service servers
    // ---------------

    /** @brief Shared pointer to the service server for resetting the speed
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_server_;

    /** @brief Shared pointer to the service server for starting the speed
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_server_;

    /** @brief Shared pointer to the service server for stopping the speed
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_server_;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    double brake_threshold_;

    double brake_limit_;

    double current_speed_;

    double minimum_speed_;

    std::unique_ptr<RampFunction> ramp_;

    void CheckDeadzone();

    void PublishThrottle(const double& throttle);

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_publisher_;

    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    void DiagnosticTimerCallback();

    std::string hardware_id_ = "olav-spd-k4mj";
};

} // namespace ROS
} // namespace OLAV