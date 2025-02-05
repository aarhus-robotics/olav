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

#include <boost/algorithm/clamp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <olav_interfaces/srv/set_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <olav_control/multiplexer/control_mode.hpp>

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

    void UpdateMode();

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
     * @brief Reset speed controller.
     */
    void ResetSpeedController();

    /** @brief Reset speed controller client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        reset_speed_controller_client_;

    /**
     * @brief Reset speed controller client callback.
     *
     * @param future Reset speed controller request future.
     */
    void ResetSpeedControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

    // Steering controller client
    // --------------------------

    /**
     * @brief Reset steering controller.
     */
    void ResetSteeringController();

    /** @brief Reset steering controller client. */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        reset_steering_controller_client_;

    /**
     * @brief Reset steering controller client callback.
     *
     * @param future Reset steering controller request future.
     */
    void ResetSteeringControllerCallback(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

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

    /** @brief Target control mode. */
    ControlMode target_control_mode_;

    /** @brief Active control mode. */
    ControlMode active_control_mode_;

    // Error handling
    // --------------

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

    /** @brief Drive-by-wire switch state. */
    std::atomic<bool> is_drive_by_wire_enabled_ = false;

    /** @brief Drive-by-wire switch subscription subscription. */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        drive_by_wire_switch_subscription_;

    /**
     * @brief Drive-by-wire switch subscription callback.
     *
     * @param drive_by_wire_switch_message Drive-by-wire switch
     * message.
     */
    void DriveByWireSwitchCallback(
        const std_msgs::msg::Bool::ConstSharedPtr drive_by_wire_switch_message);

    // Diagnostic
    // ----------

    /** @brief Diagnostic publisher. */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_publisher_;

    /** @brief Diagnostic callback execution timer.  */
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    /** @brief Diagnostic callback function. */
    void DiagnosticTimerCallback();

    std::atomic<bool> is_emergency_stop_triggered_ = false;

    bool last_drive_by_wire_state_ = false;

    std::atomic<bool> is_autonomous_mode_on_ = false;

    std::atomic<bool> is_speed_controller_ready_ = false;

    std::atomic<bool> is_steering_controller_ready_ = false;

    std::vector<std::string> generic_ids_;

    std::string gamepad_id_ = "gamepad";

    std::vector<std::string> autonomy_ids_;

    std::string speed_controller_id_ = "olav-spd-k4mj";

    std::string steering_controller_id_ = "olav-str-v8f3";

    bool IsEnabled();

    bool ValidateHeader(const std_msgs::msg::Header& header,
                        const std::string& authority);

    bool ValidateHeader(const std_msgs::msg::Header& header,
                        const std::vector<std::string>& authorities);

    void PublishSafetyThrottle();

    void PublishSafetyBrake();

    void PublishSafetySteering();

    void
    PublishSpeedSetpoint(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
                             ackermann_drive_message);

    void PublishSteeringAngleSetpoint(
        ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
            ackermann_drive_message);
};

} // namespace ROS
} // namespace OLAV