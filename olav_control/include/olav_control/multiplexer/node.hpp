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
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <olav_interfaces/srv/set_control_mode.hpp>

namespace OLAV {
namespace ROS {

class ControlMultiplexerNode : public rclcpp::Node {
  public:
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

    void Reset();

    // Thread safety
    // -------------

    /** @brief Subscriptions callback group. */
    rclcpp::CallbackGroup::SharedPtr subscriptions_callback_group_;

    /** @brief Services callback group. */
    rclcpp::CallbackGroup::SharedPtr services_callback_group_;

    // Muxed signals
    // -------------

    /** @brief Heartbeat subscription. */
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr
        heartbeat_subscription_;

    /**
     * @brief Heartbeat subscription callback.
     *
     * @param heartbeat_message Heartbeat message.
     */
    void HeartbeatCallback(
        const std_msgs::msg::Header::ConstSharedPtr heartbeat_message);

    /** @brief Throttle subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_subscription_;

    /**
     * @brief Throttle subscription callback.
     *
     * @param throttle_message Throttle message.
     */
    void ThrottleCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr throttle_message);

    /** @brief Throttle publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_publisher_;

    /** @brief Brake subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_subscription_;

    /**
     * @brief Brake subscription callback.
     *
     * @param brake_message Brake message.
     */
    void BrakeCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr brake_message);

    /** @brief Brake publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_publisher_;

    /** @brief Steering subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_subscription_;

    /**
     * @brief Steering subscription callback.
     *
     * @param steering_message Steering message.
     */
    void SteeringCallback(
        olav_interfaces::msg::SetpointStamped::SharedPtr steering_message);

    /** @brief Steering publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_publisher_;

    /** @brief Heartbeat publisher. */
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_publisher_;

    /** @brief Ackermann drive subscription. */
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_drive_subscription_;

    /**
     * @brief Ackermann drive subscription callback.
     *
     * @param ackermann_drive_message Ackermann drive message.
     */
    void
    AckermannDriveCallback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
                               ackermann_drive_message);

    /** @brief Speed setpoint publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        speed_setpoint_publisher_;

    /** @brief Steering angle setpoint publisher. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_angle_setpoint_publisher_;

    // Speed controller client
    // -----------------------

    /**
     * @brief Start speed controller.
     */
    void StartSpeedController();

    /** @brief Start speed controller client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        start_speed_controller_client_;

    /**
     * @brief Start speed controller client callback.
     *
     * @param future Start speed controller request future.
     */
    void StartSpeedControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    /**
     * @brief Stop speed controller.
     */
    void StopSpeedController();

    /** @brief Stop speed controller client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        stop_speed_controller_client_;

    /**
     * @brief Stop speed controller client callback.
     *
     * @param future Stop speed controller request future.
     */
    void StopSpeedControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    // Steering controller client
    // --------------------------

    /**
     * @brief Start steering controller.
     */
    void StartSteeringController();

    /** @brief Start steering controller client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        start_steering_controller_client_;

    /**
     * @brief Start steering controller client callback.
     *
     * @param future Start steering controller request future.
     */
    void StartSteeringControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    /**
     * @brief Stop steering controller.
     */
    void StopSteeringController();

    /** @brief Stop steering controller client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        stop_steering_controller_client_;

    /**
     * @brief Stop steering controller client callback.
     *
     * @param future Stop steering controller request future.
     */
    void StopSteeringControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    // ---------------

    // Mode
    // ----

    /**
     * @brief Set multiplexer control mode.
     *
     * @param mode Multiplexer control mode.
     */
    void SetMode(const std::string& mode);

    /** @brief Cycle control mode service. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        cycle_control_mode_service_;

    /**
     * @brief Cycle control mode service callback.
     *
     * @param request Cycle control mode request.
     * @param response Cycle control mode response.
     */
    void CycleControlMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Set control mode service. */
    rclcpp::Service<olav_interfaces::srv::SetControlMode>::SharedPtr
        set_control_mode_service_;

    /**
     * @brief Set control mode service callback.
     *
     * @param request Set control mode request.
     * @param response Set control mode response.
     */
    void SetControlMode(
        const std::shared_ptr<olav_interfaces::srv::SetControlMode::Request>
            request,
        std::shared_ptr<olav_interfaces::srv::SetControlMode::Response>
            response);

    /** @brief Active control mode. */
    std::string active_control_mode_;

    // Error handling
    // --------------

    /** @brief Is the node in a faulty state? */
    bool is_faulty_ = false;

    /**
     * @brief Is the multiplexer active?
     *
     * @return true The multiplexer is active.
     * @return false The multiplexer is not active.
     */
    bool IsActive();

    /** @brief Emergency stop state. */
    bool emergency_stop_ = false;

    /** @brief Emergency stop subscription. */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        emergency_stop_subscription_;

    /**
     * @brief Emergency stop subscription callback.
     *
     * @param emergency_stop_message Emergency stop message.
     */
    void EmergencyStopCallback(
        const std_msgs::msg::Bool::ConstSharedPtr emergency_stop_message);

    // Diagnostic
    // ----------

    /** @brief Diagnostic publisher. */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_publisher_;

    /** @brief Diagnostic callback execution timer.  */
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    /** @brief Diagnostic callback function. */
    void DiagnosticTimerCallback();
};

} // namespace ROS
} // namespace OLAV