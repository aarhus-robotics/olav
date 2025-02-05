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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_control/core/controllers/pid.hpp>
#include <olav_control/core/filters/savitzky_golay.hpp>
#include <olav_interfaces/msg/pid_status.hpp>
#include <olav_interfaces/msg/setpoint_stamped.hpp>

namespace OLAV {
namespace ROS {

class SteeringControllerNode : public rclcpp::Node {
  public:
    SteeringControllerNode();

  private:
    void Configure();

    void GetParameters();

    void Initialize();

    void Activate();

    void CreateSubscriptions();

    void CreateTimers();

    void CreateServices();

    void CreatePublishers();

    void StartTimers();

    rcl_interfaces::msg::SetParametersResult
    SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void Reset();

    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        setpoint_subscription_;

    void SetpointCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr setpoint_message);

    std::atomic<bool> has_setpoint_ = false;

    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        feedback_subscription_;

    void FeedbackCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr feedback_message);

    std::atomic<bool> has_feedback_ = false;

    // Control timer
    // -------------

    /** @brief Control timer shared pointer. */
    rclcpp::TimerBase::SharedPtr control_timer_;

    /** @brief Controller tick rate. */
    double control_rate_;

    void ControlCallback();

    // Controller
    // ----------
    std::shared_ptr<PIDController> controller_;

    /** @brief Throttle effort publisher shared pointer. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        output_publisher_;

    bool publish_status_;

    /** @brief Shared pointer to the PID status publisher. */
    rclcpp::Publisher<olav_interfaces::msg::PIDStatus>::SharedPtr
        status_publisher_;

    // Service servers
    // ---------------

    /** @brief Shared pointer to the service server for resetting the steering
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_server_;

    void ResetServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Diagnostic
    // ----------
    bool publish_diagnostic_;

    /** @brief Diagnostic publisher. */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_publisher_;

    /** @brief Diagnostic callback execution timer.  */
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    /** @brief Diagnostic callback function. */
    void DiagnosticTimerCallback();

    /** @brief Diagnostic hardware ID. */
    std::string hardware_id_ = "olav-str-v8f3";
};

} // namespace ROS
} // namespace OLAV