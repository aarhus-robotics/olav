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
#include <limits>
#include <shared_mutex>
#include <thread>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/algorithm/clamp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <olav_interfaces/msg/drive_by_wire_plc_status.hpp>
#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <olav_interfaces/srv/set_pid_gains.hpp>

#include <olav_control/drive_by_wire/feedback.hpp>
#include <olav_control/drive_by_wire/interface.hpp>
#include <olav_control/drive_by_wire/setpoint.hpp>


namespace OLAV {
namespace ROS {

class DriveByWireNode : public rclcpp::Node {
  public:
    DriveByWireNode();

  protected:
    void Configure();
    void GetParameters();
    void Initialize();
    void Activate();
    void CreateSubscriptions();
    void CreateTimers();
    void CreatePublishers();
    void CreateServices();
    void StartTimers();

  private:
    // Initialization
    // --------------
    void InitializeRegisters();

    // Connection
    // ----------
    rclcpp::TimerBase::SharedPtr connect_timer_;
    double connect_period_;
    void ConnectCallback();
    void Disconnect();

    // Health checks
    // -------------
    void HealthCheckCallback();

    void ThrottleCallback(
        olav_interfaces::msg::SetpointStamped::ConstSharedPtr throttle_message);

    void BrakeCallback(
        olav_interfaces::msg::SetpointStamped::ConstSharedPtr brake_message);

    void SteeringCallback(
        olav_interfaces::msg::SetpointStamped::ConstSharedPtr steering_message);

    void OdometryCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr odometry_message);

    double Smoothstep(double x, double edge0, double edge1);

    // Drive-by-wire interface
    // -----------------------

    /** @brief Drive-by-wire interface shared pointer. */
    std::shared_ptr<DriveByWireInterface> interface_;

    /** @brief Drive-by-wire interface Modbus TCP server address. */
    std::string connection_address_;

    /** @brief Drive-by-wire interface Modbus TCP server port. */
    int connection_port_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;

    /**
     * @brief Clear the drive-by-wire system to receive contro lcommands,
     * provided the system is ready to run.
     *
     * @param request All clear signal - empty, as this is a
     * std_srvs/srv/Trigger service.
     * @param response System response - contains the outcome of the system
     * activation and a comment.
     */
    void Ready(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Atomic variable defining whether the drive-by-wire Modbus TCP
     * server is connected. */
    std::atomic<bool> is_connected_ = false;

    /** @brief Atomic variable defining whether the drive-by-wire system has
     * been cleared by a human to receive commands. */
    std::atomic<bool> is_ready_ = false;

    /** @brief Shared pointer to the PLC register writer timer. */
    rclcpp::TimerBase::SharedPtr writer_timer_;

    double writer_period_;

    /**
     * @brief Callback for the drive-by-wire PLC registers writer timer.
     */
    void WriterCallback();

    /** @brief Shared pointer to the PLC register reader timer. */
    rclcpp::TimerBase::SharedPtr reader_timer_;

    double reader_period_;

    /**
     * @brief Callback for the drive-by-wire PLC registers reader timer.
     */
    void ReaderCallback();

    /**
     * @brief Publish the steering angle.
     */
    void PublishSteeringAngle();
    // -----------------------

    // Debugging
    // ---------
    /** @brief Shared pointer to the PLC debug message publisher timer. */
    rclcpp::TimerBase::SharedPtr debug_timer_;

    double debug_period_;

    /**
     * @brief Callback for the drive-by-wire PLC debug message publisher timer.
     */
    void DebugCallback();

    /**
     * @brief Publish the register values of the WAGO PLC.
     */
    void PublishPLCStatus();

    // ---------

    // Drive-by-wire commands subscriptions
    // ------------------------------------
    //** @brief Commanded throttle effort subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        throttle_subscription_;

    //** @brief Commanded brake effort subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        brake_subscription_;

    //** @brief Commanded steering effort subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_subscription_;

    // Drive-by-wire services
    // ----------------------
    /** @brief Shared pointer to the service to cycle the ignition state. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cycle_ignition_service_;

    void CycleIgnition(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /** @brief Shared pointer to the service to set the ignition state. */
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ignition_service_;

    void
    SetIgnition(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /** @brief Shared pointer to the service to trigger the emergency stop.. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;

    void EmergencyStop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void TriggerEmergencyStop();

    /** @brief Shared pointer to the service to start the engine. */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_engine_service_;

    void
    StartEngine(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    double engine_starter_duration_;

    int engine_speed_window_size_;

    double engine_speed_check_window_;

    double engine_speed_threshold_;

    /** @brief Shared pointer to the service to set the gains of the internal
     * steering PID. */
    rclcpp::Service<olav_interfaces::srv::SetPIDGains>::SharedPtr
        set_steering_pid_gains_service_;

    void SetSteeringPIDGains(
        const std::shared_ptr<olav_interfaces::srv::SetPIDGains::Request>
            request,
        std::shared_ptr<olav_interfaces::srv::SetPIDGains::Response> response);

    // ------------

    // Engine speed check
    // ------------------

    void EngineSpeedCallback(
        const olav_interfaces::msg::SetpointStamped::ConstSharedPtr
            engine_speed_message);

    /** @brief Shared pointer to the engine speed subscription. */
    rclcpp::Subscription<olav_interfaces::msg::SetpointStamped>::SharedPtr
        engine_speed_subscription_;

    /** @brief Time stamp of the last triggered engine speed callback. */
    rclcpp::Time last_engine_speed_time_;

    /** @brief Whether or not an engine speed message has been received since
     * the last node reset. */
    std::atomic<bool> has_engine_speed_ = false;

    /** @brief Last received engine speed in revolutions per minute. */
    double engine_speed_ = std::numeric_limits<double>::signaling_NaN();

    /** @brief Maximum allowed engine speed in revolutions per minute before the
     * emergency stop is triggered.*/
    double maximum_engine_speed_;
    // ------------------

    // Vehicle speed check
    // -------------------

    /** @brief Shared pointer to the vehicle odometry subscription. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    /** @brief Time stamp of the last triggered vehicle odometry callback. */
    rclcpp::Time last_odometry_time_;

    /** @brief Whether or not a vehicle odometry message has been received since
     * the last node reset. */
    std::atomic<bool> has_odometry_ = false;

    /** @brief Last received vehicle longitudinal speed in meters per second.*/
    double vehicle_speed_ = std::numeric_limits<double>::signaling_NaN();

    /** @brief Maximum allowed vehicle longitudinal speed in meters per second
     * before the emergency stop is triggered. */
    double maximum_longitudinal_speed_ =
        std::numeric_limits<double>::signaling_NaN();
    // -------------------

    // Hearbeat check
    // --------------

    /** @brief Shared pointer to the controller heartbeat subscription. */
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr
        heartbeat_subscription_;

    void HeartbeatCallback(
        const std_msgs::msg::Header::ConstSharedPtr heartbeat_message);

    /** @brief Time stamp of the last triggered controller heartbeat cabllack.
     * */
    rclcpp::Time last_heartbeat_time_;

    /** @brief Whether or not a controller heartbeat message has been received
     * since the last node reset. */
    std::atomic<bool> has_heartbeat_ = false;

    // Health check
    // ------------

    /** @brief Shared pointer to the health check timer. */
    rclcpp::TimerBase::SharedPtr health_check_timer_;

    double health_check_period_;

    // ------------

    // Thread safety
    // -------------
    mutable std::mutex modbus_mutex_;
    mutable std::shared_mutex setpoint_mutex_;
    mutable std::shared_mutex feedback_mutex_;

    rclcpp::CallbackGroup::SharedPtr subscriptions_callback_group_;
    rclcpp::CallbackGroup::SharedPtr writer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr reader_callback_group_;
    rclcpp::CallbackGroup::SharedPtr debug_callback_group_;
    rclcpp::CallbackGroup::SharedPtr services_callback_group_;
    // -------------

    // Gear shifting
    // -------------

    /** @brief Shared pointer to the gear upshift service.  */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shift_gear_up_service_;

    // Upshift service
    // ---------------
    void
    ShiftGearUp(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shift_gear_down_service_;

    // Downshift service
    // ---------------
    void ShiftGearDown(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // -------------

    // Setpoint
    // --------

    /** @brief Setpoint structure. */
    std::shared_ptr<DriveByWireSetpoint> drive_by_wire_setpoint_;

    // Feedback
    // --------
    std::shared_ptr<DriveByWireFeedback> drive_by_wire_feedback_;
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        steering_angle_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;
    rclcpp::Publisher<olav_interfaces::msg::DriveByWirePLCStatus>::SharedPtr
        plc_status_publisher_;

    // Logging
    // -------

    /** @brief Seconds between throttled log messages. */
    double log_throttle_delay_;

    template <typename T>
    void SetResponseFailError(std::shared_ptr<T> response,
                              const std::string& message) {
        RCLCPP_WARN(get_logger(), message.c_str());
        response->success = false;
        response->message = message;
    }

    template <typename T>
    void SetResponseFailWarning(std::shared_ptr<T> response,
                                const std::string& message) {
        RCLCPP_WARN(get_logger(), message.c_str());
        response->success = false;
        response->message = message;
    }

    template <typename T>
    void SetResponseSuccessInfo(std::shared_ptr<T> response,
                                const std::string& message) {
        RCLCPP_INFO(get_logger(), message.c_str());
        response->success = true;
        response->message = message;
    }

    template <typename T>
    void SetResponseSuccessWarning(std::shared_ptr<T> response,
                                   const std::string& message) {
        RCLCPP_WARN(get_logger(), message.c_str());
        response->success = true;
        response->message = message;
    }

    // -------

    bool emergency_stop_ = false;

    /** @brief Shared pointer to the emergency stop publisher. */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;

    /** @brief Shared pointer to the ready state publisher. */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;

    bool IsHealthy();

    // Diagnostics
    // -----------
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    void DiagnosticsCallback();

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostics_publisher_;

    std::string hardware_id_ = "olav-dbw-894d";

    void GetDiagnostics(diagnostic_msgs::msg::DiagnosticArray::SharedPtr
                            diagnostic_array_message);

    void
    GetEngineSpeedDiagnostics(diagnostic_msgs::msg::DiagnosticArray::SharedPtr
                                  diagnostic_array_message);

    void
    GetConnectionDiagnostics(diagnostic_msgs::msg::DiagnosticArray::SharedPtr
                                 diagnostic_array_message);
    void GetOdometryDiagnostics(diagnostic_msgs::msg::DiagnosticArray::SharedPtr
                                    diagnostic_array_message);
    void
    GetHeartbeatDiagnostics(diagnostic_msgs::msg::DiagnosticArray::SharedPtr
                                diagnostic_array_message);
    void GetReadyDiagnostics(diagnostic_msgs::msg::DiagnosticArray::SharedPtr
                                 diagnostic_array_message);
};

} // namespace ROS
} // namespace OLAV
