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

#include <boost/circular_buffer.hpp>

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

    rcl_interfaces::msg::SetParametersResult
    SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void Reset();

    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        setpoint_subscription_;

    void SetpointCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr setpoint_message);

    std::atomic<bool> has_setpoint_;

    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        feedback_subscription_;

    void FeedbackCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr feedback_message);

    std::atomic<bool> has_feedback_;

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

    /** @brief Controller feedforward gain. */
    double feedforward_gain_;

    /** @brief Controller proportional gain. */
    double proportional_gain_;

    /** @brief Controller integral gain. */
    double integral_gain_;

    /** @brief Controller derivative gain. */
    double derivative_gain_;

    /** @brief Controller minimum output. */
    double minimum_output_;

    /** @brief Controller maximum output. */
    double maximum_output_;

    /** @brief Whether or not the controller limits the maximum output change
     * between ticks. */
    bool use_output_change_limiter_;

    /** @brief Controller maximum output change. */
    double maximum_output_change_;

    /** @brief Whether or not to use setpoint ramping. */
    bool use_setpoint_ramping_;

    /** @brief PID controller maximum setpoint ramp between ticks. */
    double maximum_setpoint_change_;

    /** @brief Whether or not the controller limits the integral term. */
    bool use_integral_term_limiter_;

    /** @brief PID controller maximum integral term magnitude. */
    double maximum_integral_term_;
    // ----------

    /** @brief Throttle effort publisher shared pointer. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        output_publisher_;

    /** @brief Shared pointer to the PID status publisher. */
    rclcpp::Publisher<olav_interfaces::msg::PIDStatus>::SharedPtr
        status_publisher_;

    // Filtering
    // ---------

    /** @brief Whether or not to use the steering angle setpoint filter. */
    bool use_steering_angle_setpoint_filter_;

    /** @brief Savitzky-Golay filter to the steering angle setpoint
     * measurements. */
    std::shared_ptr<SavitzkyGolayFilter> steering_angle_setpoint_filter_;

    /** @brief Window size for the steering angle setpoint Savitzky-Golay
     * filter. */
    double steering_angle_setpoint_filter_window_size_;

    /** @brief Regression polynomial order for the steering angle setpoint
     * Savitzky-Golay filter. */
    int steering_angle_setpoint_filter_order_;

    /** @brief Shared pointer to the circular buffer holding the latest steering
     * angle setpoint. */
    std::shared_ptr<boost::circular_buffer<double>>
        steering_angle_setpoint_buffer_;

    /** @brief Whether or not to use the steering angle feedback filter. */
    bool use_steering_angle_feedback_filter_;

    /** @brief Savitzky-Golay filter to the steering angle feedback
     * measurements. */
    std::shared_ptr<SavitzkyGolayFilter> steering_angle_feedback_filter_;

    /** @brief Window size for the steering angle feedback Savitzky-Golay
     * filter. */
    double steering_angle_feedback_filter_window_size_;

    /** @brief Regression polynomial order for the steering angle feedback
     * Savitzky-Golay filter. */
    int steering_angle_feedback_filter_order_;

    /** @brief Shared pointer to the circular buffer holding the latest steering
     * angle feedback. */
    std::shared_ptr<boost::circular_buffer<double>>
        steering_angle_feedback_buffer_;

    // Service servers
    // ---------------

    /** @brief Shared pointer to the service server for starting the steering
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_server_;

    void StartServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Shared pointer to the service server for stopping the steering
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_server_;

    void StopServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::atomic<bool> is_stopped_;

    /** @brief Shared pointer to the service server for resetting the steering
     * controller. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_server_;

    void ResetServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    /** @brief Whether or not to use the deadband filter. */
    bool use_deadband_filter_;

    /** @brief Deadband filter upper threshold. */
    double deadband_lower_threshold_;

    /** @brief Deadband filter upper threshold. */
    double deadband_upper_threshold_;
};

} // namespace ROS
} // namespace OLAV