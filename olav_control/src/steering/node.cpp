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

#include <olav_control/steering/node.hpp>

namespace OLAV {
namespace ROS {

SteeringControllerNode::SteeringControllerNode()
    : rclcpp::Node("steering_controller") {
    Configure();
    Activate();
}

void SteeringControllerNode::Configure() {
    GetParameters();
    Initialize();
}

void SteeringControllerNode::GetParameters() {
    declare_parameter("rate", 100.0);
    declare_parameter("pid.feedforward.offset", 0.3);
    declare_parameter("pid.gains.proportional", 1.0);
    declare_parameter("pid.gains.integral", 0.1);
    declare_parameter("pid.gains.derivative", 0.01);
    declare_parameter("pid.setpoint.ramp.enabled", true);
    declare_parameter("pid.setpoint.ramp.magnitude", 0.001);
    declare_parameter("pid.output.change.enabled", true);
    declare_parameter("pid.output.change.magnitude", 0.001);
    declare_parameter("pid.limit.integral.enabled", true);
    declare_parameter("pid.limit.integral.magnitude", 50.0);
    declare_parameter("pid.deadband.filter.enabled", true);
    declare_parameter("pid.deadband.filter.thresholds.lower", -0.3);
    declare_parameter("pid.deadband.filter.thresholds.upper", 0.3);
    declare_parameter("pid.limit.error.enabled", true);
    declare_parameter("pid.limit.error.magnitude", 0.04);
    declare_parameter("debug.diagnostic", true);
    declare_parameter("debug.status", false);

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&SteeringControllerNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void SteeringControllerNode::Initialize() {
    // Initialize the steering PID controller.
    controller_ = std::make_shared<PIDController>();
    controller_->SetFeedforwardOffset(
        get_parameter("pid.feedforward.offset").as_double());
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
    controller_->UseOutputChangeLimiter(
        get_parameter("pid.output.change.enabled").as_bool());
    controller_->SetMaximumOutputChange(
        get_parameter("pid.output.change.magnitude").as_double());
    controller_->SetMinimumOutput(-1.0);
    controller_->SetMaximumOutput(1.0);
    controller_->UseDeadbandFilter(
        get_parameter("pid.deadband.filter.enabled").as_bool());
    controller_->SetDeadbandLowerThreshold(
        get_parameter("pid.deadband.filter.thresholds.lower").as_double());
    controller_->SetDeadbandUpperThreshold(
        get_parameter("pid.deadband.filter.thresholds.upper").as_double());
    controller_->UseErrorThreshold(
        get_parameter("pid.limit.error.enabled").as_bool());
    controller_->SetErrorThreshold(
        get_parameter("pid.limit.error.magnitude").as_double());
    controller_->UseIntegralTermLimiter(
        get_parameter("pid.limit.integral.enabled").as_bool());
    controller_->SetMaximumIntegralTerm(
        get_parameter("pid.limit.integral.magnitude").as_double());

    publish_diagnostic_ = get_parameter("debug.diagnostic").as_bool();
    publish_status_ = get_parameter("debug.status").as_bool();
}

void SteeringControllerNode::Activate() {
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();
    StartTimers();
}

void SteeringControllerNode::Reset() {
    has_feedback_ = false;
    has_setpoint_ = false;

    controller_->Reset();
}

void SteeringControllerNode::CreateSubscriptions() {
    setpoint_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "setpoint",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&SteeringControllerNode::SetpointCallback,
                      this,
                      std::placeholders::_1));

    // Create the subscription for the steering controller feedback.
    feedback_subscription_ =
        create_subscription<olav_interfaces::msg::SetpointStamped>(
            "feedback",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&SteeringControllerNode::FeedbackCallback,
                      this,
                      std::placeholders::_1));
}

void SteeringControllerNode::CreateTimers() {
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(
            1.0 / get_parameter("rate").as_double()),
        std::bind(&SteeringControllerNode::ControlCallback, this));
    control_timer_->cancel();

    if(publish_diagnostic_) {
        diagnostic_timer_ = create_wall_timer(
            std::chrono::duration<double>(0.5),
            std::bind(&SteeringControllerNode::DiagnosticTimerCallback, this));
        diagnostic_timer_->cancel();
    }
}

void SteeringControllerNode::CreateServices() {
    reset_service_server_ = create_service<std_srvs::srv::Trigger>(
        "reset",
        std::bind(&SteeringControllerNode::ResetServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
}

void SteeringControllerNode::CreatePublishers() {
    output_publisher_ = create_publisher<olav_interfaces::msg::SetpointStamped>(
        "output",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    status_publisher_ = create_publisher<olav_interfaces::msg::PIDStatus>(
        "status",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    if(publish_diagnostic_) {
        diagnostic_publisher_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics",
                RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    }

    if(publish_status_) {
        status_publisher_ = create_publisher<olav_interfaces::msg::PIDStatus>(
            "status",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    }
}

void SteeringControllerNode::StartTimers() {
    control_timer_->reset();

    if(publish_diagnostic_) { diagnostic_timer_->reset(); }
}

void SteeringControllerNode::SetpointCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr setpoint_message) {
    // Update the PID controller setpoint.
    controller_->SetSetpoint(setpoint_message->setpoint);
    has_setpoint_ = true;
}

void SteeringControllerNode::FeedbackCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr feedback_message) {
    // Filter the feedback.
    controller_->SetFeedback(feedback_message->setpoint);
    has_feedback_ = true;
}

void SteeringControllerNode::ControlCallback() {
    // Check for a valid speed setpoint.
    if((!has_setpoint_) || (!has_feedback_)) { return; }

    // Tick the controller.
    controller_->Tick();
    double output = controller_->GetOutput();

    // Publish the controller steering output.
    olav_interfaces::msg::SetpointStamped steering_effort_message;
    steering_effort_message.header.frame_id = "olav-str-v8f3";
    steering_effort_message.setpoint = output;
    output_publisher_->publish(steering_effort_message);

    if(publish_status_) {
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

void SteeringControllerNode::ResetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Suppress unused variable compiler warning.
    (void)request;

    Reset();

    std::string message("Steering controller reset.");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
}

rcl_interfaces::msg::SetParametersResult
SteeringControllerNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    for(const auto& parameter : parameters) {
        if(parameter.get_name() == "pid.gains.proportional") {
            controller_->SetProportionalGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.gains.integral") {
            controller_->SetIntegralGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.gains.derivative") {
            controller_->SetDerivativeGain(parameter.as_double());
        } else if(parameter.get_name() == "pid.output.change.enabled") {
            controller_->UseOutputChangeLimiter(parameter.as_bool());
        } else if(parameter.get_name() == "pid.output.change.magnitude") {
            controller_->SetMaximumOutputChange(parameter.as_double());
        } else if(parameter.get_name() == "pid.setpoint.ramp.enabled") {
            controller_->UseSetpointRamping(parameter.as_bool());
        } else if(parameter.get_name() == "pid.setpoint.ramp.magnitude") {
            controller_->SetMaximumSetpointChange(parameter.as_double());
        } else if(parameter.get_name() == "pid.limit.integral.enabled") {
            controller_->UseIntegralTermLimiter(parameter.as_bool());
        } else if(parameter.get_name() == "pid.limit.integral.magnitude") {
            controller_->SetMaximumIntegralTerm(parameter.as_double());
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

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void SteeringControllerNode::DiagnosticTimerCallback() {
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array_message;
    diagnostic_array_message.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status_message;
    diagnostic_status_message.name = "olav/steering_controller/enabled";
    diagnostic_status_message.message = true;
    diagnostic_status_message.hardware_id = hardware_id_;
    diagnostic_array_message.status.push_back(diagnostic_status_message);
    diagnostic_publisher_->publish(diagnostic_array_message);
}

} // namespace ROS
} // namespace OLAV