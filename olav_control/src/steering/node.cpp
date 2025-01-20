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
    // Define a floating point range suitable for controller gains.
    rcl_interfaces::msg::FloatingPointRange gain_range;
    gain_range.set__from_value(0.0).set__to_value(100.0).set__step(0.001);
    rcl_interfaces::msg::ParameterDescriptor gain_descriptor;
    gain_descriptor.floating_point_range = {gain_range};

    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    // Declare and retrieve the controller rate.
    declare_parameter("controller.rate", 100.0, read_only_descriptor);
    control_rate_ = get_parameter("controller.rate").as_double();

    // Declare and retrieve the steering controller proportional gain.
    declare_parameter("controller.gains.proportional", 1.0, gain_descriptor);
    proportional_gain_ =
        get_parameter("controller.gains.proportional").as_double();

    // Declare and retrieve the steering controller integral gain.
    declare_parameter("controller.gains.integral", 0.1, gain_descriptor);
    integral_gain_ = get_parameter("controller.gains.integral").as_double();

    // Declare and retrieve the steering controller derivative gain.
    declare_parameter("controller.gains.derivative", 0.01, gain_descriptor);
    derivative_gain_ = get_parameter("controller.gains.derivative").as_double();

    // Declare and retrieve the steering controller setpoint filter switch.
    declare_parameter("controller.setpoint.filter.enabled",
                      true,
                      read_only_descriptor);
    use_steering_angle_setpoint_filter_ =
        get_parameter("controller.setpoint.filter.enabled").as_bool();

    // Declare and retrieve the steering controller setpoint filter window size.
    declare_parameter("controller.setpoint.filter.window",
                      10,
                      read_only_descriptor);
    steering_angle_setpoint_filter_window_size_ =
        get_parameter("controller.setpoint.filter.window").as_int();

    // Declare and retrieve the steering controller setpoint filter order.
    declare_parameter("controller.setpoint.filter.order",
                      3,
                      read_only_descriptor);
    steering_angle_setpoint_filter_order_ =
        get_parameter("controller.setpoint.filter.order").as_int();

    declare_parameter("controller.setpoint.ramp.enabled", true);
    use_setpoint_ramping_ =
        get_parameter("controller.setpoint.ramp.enabled").as_bool();

    rcl_interfaces::msg::FloatingPointRange setpoint_ramp_range;
    setpoint_ramp_range.set__from_value(0.000).set__to_value(0.100).set__step(
        0.001);
    rcl_interfaces::msg::ParameterDescriptor setpoint_ramp_descriptor;
    setpoint_ramp_descriptor.floating_point_range = {setpoint_ramp_range};
    declare_parameter("controller.setpoint.ramp.magnitude",
                      0.001,
                      setpoint_ramp_descriptor);
    maximum_setpoint_change_ =
        get_parameter("controller.setpoint.ramp.magnitude").as_double();

    // Declare and retrieve the steering controller feedback filter switch.
    declare_parameter("controller.feedback.filter.enabled",
                      true,
                      read_only_descriptor);
    use_steering_angle_feedback_filter_ =
        get_parameter("controller.feedback.filter.enabled").as_bool();

    // Declare and retrieve the steering controller feedback filter window size.
    declare_parameter("controller.feedback.filter.window",
                      10,
                      read_only_descriptor);
    steering_angle_feedback_filter_window_size_ =
        get_parameter("controller.feedback.filter.window").as_int();

    // Declare and retrieve the steering controller feedback filter order.
    declare_parameter("controller.feedback.filter.order",
                      3,
                      read_only_descriptor);
    steering_angle_feedback_filter_order_ =
        get_parameter("controller.feedback.filter.order").as_int();

    // Declare and retrieve the steering controller minimum output.
    declare_parameter("controller.output.minimum", -1.0);
    minimum_output_ = get_parameter("controller.output.minimum").as_double();

    // Declare and retrieve the steering controller maximum output.
    declare_parameter("controller.output.maximum", 1.0);
    maximum_output_ = get_parameter("controller.output.maximum").as_double();

    // Declare and retrieve the steering controller output change limiter
    // switch.
    declare_parameter("controller.output.change.enabled", true);
    use_output_change_limiter_ =
        get_parameter("controller.output.change.enabled").as_bool();

    // Declare and retrieve the steering controller output change maximum
    // magnitude.
    rcl_interfaces::msg::FloatingPointRange output_change_range;
    output_change_range.set__from_value(0.000).set__to_value(0.100).set__step(
        0.001);
    rcl_interfaces::msg::ParameterDescriptor output_change_descriptor;
    output_change_descriptor.floating_point_range = {output_change_range};
    declare_parameter("controller.output.change.magnitude",
                      0.001,
                      output_change_descriptor);
    maximum_output_change_ =
        get_parameter("controller.output.change.magnitude").as_double();

    declare_parameter("controller.limit.integral.enabled", true);
    use_integral_term_limiter_ =
        get_parameter("controller.limit.integral.enabled").as_bool();

    rcl_interfaces::msg::FloatingPointRange integral_term_range;
    integral_term_range.set__from_value(0.000).set__to_value(100.0).set__step(
        0.001);
    rcl_interfaces::msg::ParameterDescriptor integral_term_descriptor;
    integral_term_descriptor.floating_point_range = {integral_term_range};
    declare_parameter("controller.limit.integral.magnitude",
                      50.0,
                      integral_term_descriptor);
    maximum_integral_term_ =
        get_parameter("controller.limit.integral.magnitude").as_double();

    declare_parameter("controller.deadband.filter.enabled", true);
    use_deadband_filter_ =
        get_parameter("controller.deadband.filter.enabled").as_bool();

    declare_parameter("controller.deadband.filter.thresholds.lower", -0.3);
    deadband_lower_threshold_ =
        get_parameter("controller.deadband.filter.thresholds.lower")
            .as_double();

    declare_parameter("controller.deadband.filter.thresholds.upper", 0.3);
    deadband_upper_threshold_ =
        get_parameter("controller.deadband.filter.thresholds.upper")
            .as_double();

    // Create the parameters callback handle.
    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&SteeringControllerNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void SteeringControllerNode::Initialize() {
    // Initialize the steering PID controller.
    controller_ = std::make_shared<PIDController>();
    controller_->SetProportionalGain(proportional_gain_);
    controller_->SetIntegralGain(integral_gain_);
    controller_->SetDerivativeGain(derivative_gain_);
    controller_->UseSetpointRamping(use_setpoint_ramping_);
    controller_->SetMaximumSetpointChange(maximum_setpoint_change_);
    controller_->UseOutputLimiter(true);
    controller_->UseOutputChangeLimiter(use_output_change_limiter_);
    controller_->SetMaximumOutputChange(maximum_output_change_);
    controller_->SetMinimumOutput(minimum_output_);
    controller_->SetMaximumOutput(maximum_output_);
    controller_->UseDeadbandFilter(use_deadband_filter_);
    controller_->SetDeadbandLowerThreshold(deadband_lower_threshold_);
    controller_->SetDeadbandUpperThreshold(deadband_upper_threshold_);

    // Initialize the steering angle filter.
    if(use_steering_angle_feedback_filter_) {
        // Check that the specified window size is sufficiently large for the
        // chosen fitting polynomial order.
        size_t buffer_size = (2 * steering_angle_feedback_filter_window_size_ +
                              1) > steering_angle_feedback_filter_order_ + 1
            ? 2 * steering_angle_feedback_filter_window_size_ + 1
            : steering_angle_feedback_filter_order_ + 1;

        // Initialize the steering angle buffer with null values.
        steering_angle_feedback_buffer_ =
            std::make_shared<boost::circular_buffer<double>>(buffer_size, 0.0);

        // Initialize the Savitzky-Golay filter settings - here we specify the
        // filter is evaluated at the end of the buffer and no derivative
        // information is used.
        auto savitzky_golay_filter_settings =
            SavitzkyGolayFilterSettings(std::floor(buffer_size / 2) - 1,
                                        std::floor(buffer_size / 2) - 1,
                                        steering_angle_feedback_filter_order_,
                                        0,
                                        1.0);

        // Instantiate the steering angle Savitzky-Golay filter.
        steering_angle_feedback_filter_ = std::make_shared<SavitzkyGolayFilter>(
            savitzky_golay_filter_settings);
    }

    // Initialize the controller atomic flags.
    has_feedback_ = false;
    has_setpoint_ = false;
    is_stopped_ = true;
}

void SteeringControllerNode::Activate() {
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();
}

void SteeringControllerNode::Reset() {
    // Reset the controller atomic flags to uninitialized state.
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
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&SteeringControllerNode::ControlCallback, this));
    control_timer_->cancel();
}

void SteeringControllerNode::CreateServices() {
    start_service_server_ = create_service<std_srvs::srv::Trigger>(
        "start",
        std::bind(&SteeringControllerNode::StartServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    stop_service_server_ = create_service<std_srvs::srv::Trigger>(
        "stop",
        std::bind(&SteeringControllerNode::StopServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

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
}

void SteeringControllerNode::SetpointCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr setpoint_message) {
    // TODO: Check setpoint units.
    if(is_stopped_) { return; }

    // Update the PID controller setpoint.
    controller_->SetSetpoint(setpoint_message->setpoint);
    // TODO: Handle setpoint rate.

    if(!has_setpoint_) {
        has_setpoint_ = true;
        RCLCPP_INFO(get_logger(),
                    "Received first setpoint message since initialization: "
                    "%0.2lf rad",
                    setpoint_message->setpoint);
    }
}

void SteeringControllerNode::FeedbackCallback(
    olav_interfaces::msg::SetpointStamped::SharedPtr feedback_message) {
    if(is_stopped_) { return; }

    // Filter the feedback.
    auto feedback = std::numeric_limits<double>::quiet_NaN();
    if(use_steering_angle_feedback_filter_) {
        steering_angle_feedback_buffer_->pop_front();
        steering_angle_feedback_buffer_->push_back(feedback_message->setpoint);
        feedback = steering_angle_feedback_filter_
                       ->filter<boost::circular_buffer<double>>(
                           *steering_angle_feedback_buffer_);
    } else {
        feedback = feedback_message->setpoint;
    }

    controller_->SetFeedback(feedback);

    if(!has_feedback_) {
        has_feedback_ = true;
        RCLCPP_INFO(get_logger(),
                    "Received first feedback message since initialization: "
                    "%0.2lf rad",
                    feedback_message->setpoint);
    }
}

void SteeringControllerNode::ControlCallback() {
    // Check for a valid speed setpoint.
    if((is_stopped_) || (!has_setpoint_) || (!has_feedback_)) { return; }

    // Tick the controller.
    controller_->Tick();

    // Publish the controller throttle output.
    auto message = std::make_shared<olav_interfaces::msg::SetpointStamped>();
    message->header.frame_id = "controller";
    message->setpoint = controller_->GetOutput();
    output_publisher_->publish(*message);
}

void SteeringControllerNode::StartServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Suppress unused variable compiler warning.
    (void)request;

    // Create the subscriptions and publishers, restart the timer.
    CreateSubscriptions();
    CreatePublishers();
    control_timer_->reset();

    // Reset the controller. This is not strictly necessary, but it is an
    // inexpensive operation that can help you get some better sleep at night.
    Reset();

    // Mark the controller as active.
    is_stopped_ = false;

    std::string message("Steering controller started.");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
}

void SteeringControllerNode::StopServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Suppress unused variable compiler warning.
    (void)request;

    // Delete the subscriptions and publishers, stop the timer.
    setpoint_subscription_.reset();
    feedback_subscription_.reset();
    control_timer_->cancel();

    // Reset the PID controller.
    controller_->Reset();

    // Mark the controller as stopped.
    is_stopped_ = true;

    std::string message("Steering controller stopped.");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
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
        if(parameter.get_name() == "controller.gains.proportional") {
            controller_->SetProportionalGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.gains.integral") {
            controller_->SetIntegralGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.gains.derivative") {
            controller_->SetDerivativeGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.output.minimum") {
            controller_->SetMinimumOutput(parameter.as_double());
        } else if(parameter.get_name() == "controller.output.maximum") {
            controller_->SetMaximumOutput(parameter.as_double());
        } else if(parameter.get_name() == "controller.output.change.enabled") {
            controller_->UseOutputChangeLimiter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controller.output.change.magnitude") {
            controller_->SetMaximumOutputChange(parameter.as_double());
        } else if(parameter.get_name() == "controller.setpoint.ramp.enabled") {
            controller_->UseSetpointRamping(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controller.setpoint.ramp.magnitude") {
            controller_->SetMaximumSetpointChange(parameter.as_double());
        } else if(parameter.get_name() == "controller.limit.integral.enabled") {
            controller_->UseIntegralTermLimiter(parameter.as_bool());
        } else if(parameter.get_name() ==
                  "controller.limit.integral.magnitude") {
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

} // namespace ROS
} // namespace OLAV