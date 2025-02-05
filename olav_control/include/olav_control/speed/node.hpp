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

#include <atomic>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <olav_core/math/cubic_spline.hpp>
#include <olav_interfaces/msg/pid_status.hpp>
#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_control/core/controllers/pid.hpp>

namespace OLAV {
namespace ROS {

class SpeedControllerNode : public rclcpp::Node {
  public:
    SpeedControllerNode();

  private:
    void Configure();

    void GetParameters();

    Eigen::RowVectorXd GetParameterVector(std::vector<double> vector);

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult
    SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void Initialize();

    void Activate();

    void CreateSubscriptions();

    void CreateTimers();

    void CreateServices();

    void CreatePublishers();

    void StartTimers();

    void Reset();

    /** @brief Shared pointer to the longitudinal PID controller. */
    std::shared_ptr<PIDController> controller_;

    /** @brief Has a valid setpoint been received since the last reset? */
    std::atomic<bool> has_setpoint_ = false;

    /** @brief Has a valid feedback been received since the last reset? */
    std::atomic<bool> has_feedback_ = false;

    /** @brief Shared pointer to the odometry feedback subscription. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    void
    OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

    /** @brief Shared pointer to the target speed subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        target_speed_subscription_;

    void
    TargetSpeedCallback(const olav_interfaces::msg::SetpointStamped::SharedPtr
                            target_speed_message);

    /** @brief PID controller tick rate in Hz. */
    double control_rate_;

    /** @brief Shared pointer to the control timer. */
    rclcpp::TimerBase::SharedPtr control_timer_;

    void ControlTimerCallback();

    /** @brief Shared pointer to the throttle effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_publisher_;

    /** @brief Shared pointer to the brake effort publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_publisher_;

    /** @brief Whether or not to publish the PID status. */
    bool publish_status_;

    /** @brief Shared pointer to the PID status publisher. */
    rclcpp::Publisher<olav_interfaces::msg::PIDStatus>::SharedPtr
        status_publisher_;

    // Service servers
    // ---------------

    /** @brief Shared pointer to the service server for resetting the speed
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_server_;

    void ResetServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Whether or not the controller will allow for negative output
     * efforts and publish brake actuation efforts accordingly. */
    bool enable_brake_;

    /** @brief Threshold effort before a positive brake actuation effort is
     * published. */
    double brake_threshold_;

    /** @brief Maximum brake actuation effort. */
    double brake_limit_;

    /** @brief Latest received speed. */
    double current_speed_;

    // Diagnostic
    // ----------

    /** @brief Whether or not to publish diagnostics. */
    bool publish_diagnostic_;

    /** @brief Diagnostic publisher. */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_publisher_;

    /** @brief Diagnostic callback execution timer.  */
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    /** @brief Diagnostic callback function. */
    void DiagnosticTimerCallback();

    /** @brief Diagnostic hardware ID. */
    std::string hardware_id_ = "olav-spd-k4mj";

    double setpoint_threshold_;

    mutable std::mutex controller_mutex_;

    mutable std::mutex parameters_mutex_;

    std::shared_ptr<CubicSpline> feedforward_spline_;

    double current_setpoint_;
};

} // namespace ROS
} // namespace OLAV