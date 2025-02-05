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
    // Define a floating point range suitable for controller gains.
    rcl_interfaces::msg::FloatingPointRange gain_range;
    gain_range.set__from_value(0.0).set__to_value(100.0).set__step(0.001);
    rcl_interfaces::msg::ParameterDescriptor gain_descriptor;
    gain_descriptor.floating_point_range = {gain_range};

    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    declare_parameter("rate", 100.0, read_only_descriptor);
    control_rate_ = get_parameter("rate").as_double();

    rcl_interfaces::msg::FloatingPointRange offset_range;
    offset_range.set__from_value(0.0).set__to_value(1.0).set__step(0.001);
    rcl_interfaces::msg::ParameterDescriptor offset_descriptor;
    offset_descriptor.floating_point_range = {offset_range};

    declare_parameter("pid.gains.feedforward", 1.0, gain_descriptor);
    feedforward_gain_ = get_parameter("pid.gains.feedforward").as_double();

    declare_parameter("pid.gains.proportional", 1.0, gain_descriptor);
    proportional_gain_ = get_parameter("pid.gains.proportional").as_double();

    declare_parameter("pid.gains.integral", 0.1, gain_descriptor);
    integral_gain_ = get_parameter("pid.gains.integral").as_double();

    declare_parameter("pid.gains.derivative", 0.0, gain_descriptor);
    derivative_gain_ = get_parameter("pid.gains.derivative").as_double();

    declare_parameter("pid.setpoint.ramp.enabled", true);
    use_setpoint_ramping_ =
        get_parameter("pid.setpoint.ramp.enabled").as_bool();

    rcl_interfaces::msg::FloatingPointRange setpoint_ramp_range;
    setpoint_ramp_range.set__from_value(0.000).set__to_value(0.100).set__step(
        0.001);
    rcl_interfaces::msg::ParameterDescriptor setpoint_ramp_descriptor;
    setpoint_ramp_descriptor.floating_point_range = {setpoint_ramp_range};
    declare_parameter("pid.setpoint.ramp.magnitude",
                      0.001,
                      setpoint_ramp_descriptor);
    maximum_setpoint_change_ =
        get_parameter("pid.setpoint.ramp.magnitude").as_double();

    declare_parameter("pid.output.change.enabled", true);
    use_output_change_limiter_ =
        get_parameter("pid.output.change.enabled").as_bool();

    declare_parameter("pid.output.change.magnitude", 0.001);
    maximum_output_change_ =
        get_parameter("pid.output.change.magnitude").as_double();

    declare_parameter("pid.limit.integral.enabled", true);
    use_integral_term_limiter_ =
        get_parameter("pid.limit.integral.enabled").as_bool();

    rcl_interfaces::msg::FloatingPointRange integral_term_range;
    integral_term_range.set__from_value(0.000).set__to_value(100.0).set__step(
        0.001);
    rcl_interfaces::msg::ParameterDescriptor integral_term_descriptor;
    integral_term_descriptor.floating_point_range = {integral_term_range};
    declare_parameter("pid.limit.integral.magnitude",
                      40.0,
                      integral_term_descriptor);
    maximum_integral_term_ =
        get_parameter("pid.limit.integral.magnitude").as_double();

    declare_parameter("pid.feedforward.offset.positive", 1.0);
    positive_feedforward_offset_ =
        get_parameter("pid.feedforward.offset.positive").as_double();

    declare_parameter("pid.feedforward.offset.negative", 1.0);
    negative_feedforward_offset_ =
        get_parameter("pid.feedforward.offset.negative").as_double();

    declare_parameter("pid.deadband.filter.enabled", true);
    use_deadband_filter_ =
        get_parameter("pid.deadband.filter.enabled").as_bool();

    declare_parameter("pid.deadband.filter.thresholds.lower", -0.18);
    deadband_lower_threshold_ =
        get_parameter("pid.deadband.filter.thresholds.lower").as_double();

    declare_parameter("pid.deadband.filter.thresholds.upper", 0.3);
    deadband_upper_threshold_ =
        get_parameter("pid.deadband.filter.thresholds.upper").as_double();

    declare_parameter("brake.enabled", false);
    enable_brake_ = get_parameter("brake.enabled").as_bool();

    declare_parameter("brake.threshold", 0.3);
    brake_threshold_ = get_parameter("brake.threshold").as_double();

    declare_parameter("brake.limit", 0.5);
    brake_limit_ = get_parameter("brake.limit").as_double();

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&SpeedControllerNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void SpeedControllerNode::Initialize() {
    // Initialize the PID controller.
    controller_ = std::make_shared<PIDController>();
    controller_->SetProportionalGain(proportional_gain_);
    controller_->SetIntegralGain(integral_gain_);
    controller_->SetDerivativeGain(derivative_gain_);
    controller_->UseSetpointRamping(use_setpoint_ramping_);
    controller_->SetMaximumSetpointChange(maximum_setpoint_change_);
    controller_->UseOutputLimiter(true);
    double minimum_output = enable_brake_ ? -1.0 : 0.0;
    controller_->SetMinimumOutput(minimum_output);
    controller_->SetMaximumOutput(1.0);
    controller_->UseOutputChangeLimiter(use_output_change_limiter_);
    controller_->SetMaximumOutputChange(maximum_output_change_);
    controller_->UseIntegralTermLimiter(use_integral_term_limiter_);
    controller_->SetMaximumIntegralTerm(maximum_integral_term_);

    // Initialize the atomic variables.
    has_feedback_ = false;
    has_setpoint_ = false;

    RCLCPP_DEBUG(get_logger(), "Speed controller ready!");
}

void SpeedControllerNode::Activate() {
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();
    StartTimers();

    // Create the subscriptions and publishers, restart the timer.
    CreateSubscriptions();
    CreatePublishers();
    control_timer_->reset();
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

    diagnostic_timer_ = create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&SpeedControllerNode::DiagnosticTimerCallback, this));
    diagnostic_timer_->cancel();
}

void SpeedControllerNode::StartTimers() { diagnostic_timer_->reset(); }

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
    } else {
        RCLCPP_INFO(get_logger(),
                    "Brake actuation is not enabled: the speed controller will "
                    "rely on throttle position only for speed regulation.");
    }

    status_publisher_ = create_publisher<olav_interfaces::msg::PIDStatus>(
        "status",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    diagnostic_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

rcl_interfaces::msg::SetParametersResult
SpeedControllerNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    {
        const std::lock_guard<std::mutex> controller_lock(controller_mutex_);

        for(const auto& parameter : parameters) {
            if(parameter.get_name() == "pid.gains.feedforward") {
                controller_->SetFeedforwardGain(parameter.as_double());
            } else if(parameter.get_name() == "pid.gains.proportional") {
                controller_->SetProportionalGain(parameter.as_double());
            } else if(parameter.get_name() == "pid.gains.integral") {
                controller_->SetIntegralGain(parameter.as_double());
            } else if(parameter.get_name() == "pid.gains.derivative") {
                controller_->SetDerivativeGain(parameter.as_double());
            } else if(parameter.get_name() == "pid.setpoint.ramp.enabled") {
                controller_->UseSetpointRamping(parameter.as_bool());
            } else if(parameter.get_name() == "pid.setpoint.ramp.magnitude") {
                controller_->SetMaximumSetpointChange(parameter.as_double());
            } else if(parameter.get_name() == "pid.output.change.enabled") {
                controller_->UseOutputChangeLimiter(parameter.as_bool());
            } else if(parameter.get_name() == "pid.output.change.magnitude") {
                controller_->SetMaximumOutputChange(parameter.as_double());
            } else if(parameter.get_name() == "pid.limit.integral.enabled") {
                controller_->UseIntegralTermLimiter(parameter.as_bool());
            } else if(parameter.get_name() == "pid.limit.integral.magnitude") {
                controller_->SetMaximumIntegralTerm(parameter.as_double());

            } else if(parameter.get_name() ==
                      "pid.feedforward.offset.positive") {
                controller_->SetPositiveFeedforwardOffset(
                    parameter.as_double());
            } else if(parameter.get_name() ==
                      "pid.feedforward.offset.negative") {
                controller_->SetNegativeFeedforwardOffset(
                    parameter.as_double());
            } else if(parameter.get_name() == "pid.deadband.filter.enabled") {
                controller_->UseDeadbandFilter(parameter.as_bool());
            } else if(parameter.get_name() ==
                      "pid.deadband.filter.thresholds.lower") {
                controller_->SetDeadbandLowerThreshold(parameter.as_double());
            } else if(parameter.get_name() ==
                      "pid.deadband.filter.thresholds.upper") {
                controller_->SetDeadbandUpperThreshold(parameter.as_double());
            } else if(parameter.get_name() == "brake.enabled") {
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
                brake_limit_ = parameter.as_double();
            } else {
                RCLCPP_WARN(get_logger(),
                            "Parameter \"%s\" does not support reconfigure!",
                            parameter.get_name().c_str());
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = false;
                return result;
            }
        }

        Reset();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void SpeedControllerNode::Reset() {
    // Reset the controller atomic flags to uninitialized state.
    has_feedback_ = false;
    has_setpoint_ = false;

    controller_->Reset();

    RCLCPP_DEBUG(get_logger(), "Speed controller reset!");
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

    {
        const std::lock_guard<std::mutex> controller_lock(controller_mutex_);
        // Tick the controller.
        controller_->Tick();
        double output = controller_->GetOutput();

        // Publish the throttle effort.
        olav_interfaces::msg::SetpointStamped throttle_message;
        throttle_message.header.frame_id = "olav-spd-k4mj";
        throttle_message.header.stamp = get_clock()->now();
        throttle_message.setpoint = (output >= 0) ? output : 0.0;
        throttle_publisher_->publish(throttle_message);

        // Publish the brake effort.
        if(enable_brake_) {
            olav_interfaces::msg::SetpointStamped brake_message;
            brake_message.header.frame_id = "olav-spd-k4mj";
            brake_message.header.stamp = get_clock()->now();
            brake_message.setpoint = (output < -brake_threshold_)
                ? -boost::algorithm::clamp(output + brake_threshold_,
                                           -brake_limit_,
                                           0.0)
                : 0.0;
            brake_publisher_->publish(brake_message);
        }
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