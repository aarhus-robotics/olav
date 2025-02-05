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

#include <olav_control/speed/node.hpp>

namespace OLAV {
namespace ROS {

SpeedControllerNode::SpeedControllerNode() : rclcpp::Node("speed_controller") {
    Configure();
    Activate();
}

void SpeedControllerNode::Configure() {
    GetParameters();
    Initialize();
}

void SpeedControllerNode::GetParameters() {
    declare_parameter("rate", 100.0);
    control_rate_ = get_parameter("rate").as_double();

    declare_parameter("pid.gains.feedforward", 0.0);
    declare_parameter("pid.gains.proportional", 1.0);
    declare_parameter("pid.gains.integral", 0.1);
    declare_parameter("pid.gains.derivative", 0.0);
    declare_parameter("pid.setpoint.ramp.enabled", true);
    declare_parameter("pid.setpoint.ramp.magnitude", 0.001);
    declare_parameter("pid.output.change.enabled", true);
    declare_parameter("pid.output.change.magnitude", 0.001);
    declare_parameter("pid.limit.integral.enabled", true);
    declare_parameter("pid.limit.integral.magnitude", 40.0);
    declare_parameter("pid.feedforward.offset.positive", 1.0);
    declare_parameter("pid.feedforward.offset.negative", 1.0);
    declare_parameter("pid.deadband.filter.enabled", true);
    declare_parameter("pid.deadband.filter.thresholds.lower", -0.18);
    declare_parameter("pid.deadband.filter.thresholds.upper", 0.3);
    declare_parameter("brake.enabled", false);
    declare_parameter("brake.threshold", 0.3);
    declare_parameter("brake.limit", 0.5);
    declare_parameter("feedforward.curve.knots",
                      std::vector<double>{0.0, 0.0, 0.0, 0.0});
    declare_parameter("feedforward.curve.values",
                      std::vector<double>{0.0, 0.0, 0.0, 0.0});
    declare_parameter("feedforward.curve.degree", 3);
    declare_parameter("debug.diagnostic", false);
    declare_parameter("debug.status", false);

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&SpeedControllerNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void SpeedControllerNode::Initialize() {
    // Initialize the PID controller.
    controller_ = std::make_shared<PIDController>();
    controller_->SetFeedforwardGain(
        get_parameter("pid.gains.feedforward").as_double());
    controller_->SetProportionalGain(
        get_parameter("pid.gains.proportional").as_double());
    controller_->SetIntegralGain(
        get_parameter("pid.gains.integral").as_double());
    controller_->SetDerivativeGain(
        get_parameter("pid.gains.derivative").as_double());
    controller_->UseSetpointRamping(
        get_parameter("pid.setpoint.ramp.enabled").as_bool());
    controller_->SetMaximumSetpointChange(
        get_parameter("pid.setpoint.ramp.magnitude").as_double());
    controller_->UseOutputLimiter(true);
    double minimum_output = enable_brake_ ? -1.0 : 0.0;
    controller_->SetMinimumOutput(minimum_output);
    controller_->SetMaximumOutput(1.0);
    controller_->UseOutputChangeLimiter(
        get_parameter("pid.output.change.enabled").as_bool());
    controller_->SetMaximumOutputChange(
        get_parameter("pid.output.change.magnitude").as_double());
    controller_->UseIntegralTermLimiter(
        get_parameter("pid.limit.integral.enabled").as_bool());
    controller_->SetMaximumIntegralTerm(
        get_parameter("pid.limit.integral.magnitude").as_double());

    controller_->UseDeadbandFilter(
        get_parameter("pid.deadband.filter.enabled").as_bool());
    controller_->SetDeadbandLowerThreshold(
        get_parameter("pid.deadband.filter.thresholds.lower").as_double());
    controller_->SetDeadbandUpperThreshold(
        get_parameter("pid.deadband.filter.thresholds.upper").as_double());

    feedforward_spline_ = std::make_shared<CubicSpline>(
        GetParameterVector(
            get_parameter("feedforward.curve.knots").as_double_array()),
        GetParameterVector(
            get_parameter("feedforward.curve.values").as_double_array()),
        get_parameter("feedforward.curve.degree").as_int());

    enable_brake_ = get_parameter("brake.enabled").as_bool();
    brake_threshold_ = get_parameter("brake.threshold").as_double();
    brake_limit_ = get_parameter("brake.limit").as_double();

    publish_diagnostic_ = get_parameter("debug.diagnostic").as_bool();
    publish_status_ = get_parameter("debug.status").as_bool();

    RCLCPP_DEBUG(get_logger(), "Speed controller ready!");
}

Eigen::RowVectorXd
SpeedControllerNode::GetParameterVector(std::vector<double> vector) {
    Eigen::RowVectorXd row_vector =
        Eigen::Map<Eigen::VectorXd>(vector.data(), vector.size());

    return row_vector;
}

void SpeedControllerNode::Activate() {
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();
    StartTimers();
}

void SpeedControllerNode::CreateSubscriptions() {
    target_speed_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&SpeedControllerNode::TargetSpeedCallback,
                      this,
                      std::placeholders::_1));

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&SpeedControllerNode::OdometryCallback,
                  this,
                  std::placeholders::_1));
}

void SpeedControllerNode::CreateServices() {
    reset_service_server_ = create_service<std_srvs::srv::Trigger>(
        "reset",
        std::bind(&SpeedControllerNode::ResetServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
}

void SpeedControllerNode::CreateTimers() {
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&SpeedControllerNode::ControlTimerCallback, this));
    control_timer_->cancel();

    if(publish_diagnostic_) {
        diagnostic_timer_ = create_wall_timer(
            std::chrono::duration<double>(0.5),
            std::bind(&SpeedControllerNode::DiagnosticTimerCallback, this));
        diagnostic_timer_->cancel();
    }
}

void SpeedControllerNode::StartTimers() {
    control_timer_->reset();

    if(publish_diagnostic_) { diagnostic_timer_->reset(); }
}

void SpeedControllerNode::CreatePublishers() {
    throttle_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "throttle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    if(enable_brake_) {
        brake_publisher_ =
            create_publisher<olav_interfaces::msg::SetpointStamped>(
                "brake",
                RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    }

    if(publish_status_) {
        status_publisher_ = create_publisher<olav_interfaces::msg::PIDStatus>(
            "status",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    }

    if(publish_diagnostic_) {
        diagnostic_publisher_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics",
                RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    }
}

rcl_interfaces::msg::SetParametersResult
SpeedControllerNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    for(const auto& parameter : parameters) {
        if(parameter.get_name() == "pid.gains.feedforward") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetFeedforwardGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.gains.proportional") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetProportionalGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.gains.integral") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetIntegralGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.gains.derivative") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetDerivativeGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.setpoint.ramp.enabled") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->UseSetpointRamping(parameter.as_bool());
        } else if(parameter.get_name() == "pid.setpoint.ramp.magnitude") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetMaximumSetpointChange(parameter.as_double());
        } else if(parameter.get_name() == "pid.output.change.enabled") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->UseOutputChangeLimiter(parameter.as_bool());
        } else if(parameter.get_name() == "pid.output.change.magnitude") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetMaximumOutputChange(parameter.as_double());
        } else if(parameter.get_name() == "pid.limit.integral.enabled") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->UseIntegralTermLimiter(parameter.as_bool());
        } else if(parameter.get_name() == "pid.limit.integral.magnitude") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetMaximumIntegralTerm(parameter.as_double());
        } else if(parameter.get_name() == "pid.deadband.filter.enabled") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->UseDeadbandFilter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "pid.deadband.filter.thresholds.lower") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetDeadbandLowerThreshold(parameter.as_double());
        } else if(parameter.get_name() ==
                  "pid.deadband.filter.thresholds.upper") {
            const std::lock_guard<std::mutex> lock(controller_mutex_);
            controller_->SetDeadbandUpperThreshold(parameter.as_double());
        } else if(parameter.get_name() == "brake.enabled") {
            const std::lock_guard<std::mutex> lock(parameters_mutex_);
            if(parameter.as_bool()) {
                brake_publisher_ =
                    create_publisher<olav_interfaces::msg::SetpointStamped>(
                        "brake",
                        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
            } else {
                brake_publisher_.reset();
            }

            enable_brake_ = parameter.as_bool();
        } else if(parameter.get_name() == "brake.limit") {
            const std::lock_guard<std::mutex> lock(parameters_mutex_);
            brake_limit_ = parameter.as_double();
        } else {
            RCLCPP_WARN(get_logger(),
                        "Parameter \"%s\" does not support reconfigure!",
                        parameter.get_name().c_str());
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            return result;
        }

        Reset();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void SpeedControllerNode::Reset() {
    has_feedback_ = false;
    has_setpoint_ = false;

    controller_->Reset();
}

void SpeedControllerNode::TargetSpeedCallback(
    const olav_interfaces::msg::SetpointStamped::SharedPtr
        target_speed_message) {
    controller_->SetSetpoint(target_speed_message->setpoint);
    has_setpoint_ = true;
}

void SpeedControllerNode::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    controller_->SetFeedback(odometry_message->twist.twist.linear.x);
    has_feedback_ = true;
}

void SpeedControllerNode::ControlTimerCallback() {
    if((!has_setpoint_) || (!has_feedback_)) { return; }

    // Compute the current feedforward value to be fed to the controller.
    double feedforward_offset =
        feedforward_spline_->Evaluate(current_setpoint_);

    double output;
    {
        const std::lock_guard<std::mutex> controller_lock(controller_mutex_);
        // Tick the controller.
        // controller_->SetFeedforwardOffset(feedforward_offset);
        controller_->Tick();
        output = controller_->GetOutput();
    }

    // Publish the throttle effort.
    olav_interfaces::msg::SetpointStamped throttle_message;
    throttle_message.header.frame_id = hardware_id_;
    throttle_message.header.stamp = get_clock()->now();
    {
        const std::lock_guard<std::mutex> lock(parameters_mutex_);
        throttle_message.setpoint = (output >= 0) ? output : 0.0;
    }
    throttle_publisher_->publish(throttle_message);

    // Publish the brake effort.
    {
        const std::lock_guard<std::mutex> lock(parameters_mutex_);
        if(enable_brake_) {
            olav_interfaces::msg::SetpointStamped brake_message;
            brake_message.header.frame_id = hardware_id_;
            brake_message.header.stamp = get_clock()->now();
            brake_message.setpoint = (output < -brake_threshold_)
                ? -boost::algorithm::clamp(output + brake_threshold_,
                                           -brake_limit_,
                                           0.0)
                : 0.0;
            brake_publisher_->publish(brake_message);
        }
    }

    if(publish_status_) {
        const std::lock_guard<std::mutex> controller_lock(controller_mutex_);
        olav_interfaces::msg::PIDStatus pid_status_message;
        pid_status_message.setpoint = controller_->GetSetpoint();
        pid_status_message.feedback = controller_->GetFeedback();
        pid_status_message.output = controller_->GetOutput();
        pid_status_message.feedforward_term = controller_->GetFeedforwardTerm();
        pid_status_message.proportional_term =
            controller_->GetProportionalTerm();
        pid_status_message.integral_term = controller_->GetIntegralTerm();
        pid_status_message.derivative_term = controller_->GetDerivativeTerm();
        status_publisher_->publish(pid_status_message);
    }
}

void SpeedControllerNode::ResetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Suppress unused variable compiler warning.
    (void)request;

    Reset();

    std::string message("Speed controller reset.");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
}

void SpeedControllerNode::DiagnosticTimerCallback() {
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array_message;
    diagnostic_array_message.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status_message;
    diagnostic_status_message.name = "olav/speed_controller/enabled";
    diagnostic_status_message.message = true;
    diagnostic_status_message.hardware_id = hardware_id_;
    diagnostic_array_message.status.push_back(diagnostic_status_message);
    diagnostic_publisher_->publish(diagnostic_array_message);
}

} // namespace ROS
} // namespace OLAV