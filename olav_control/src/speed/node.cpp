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

    declare_parameter("controller.rate", 100.0, read_only_descriptor);
    control_rate_ = get_parameter("controller.rate").as_double();

    declare_parameter("controller.gains.feedforward", 0.0, gain_descriptor);
    feedforward_gain_ =
        get_parameter("controller.gains.feedforward").as_double();

    declare_parameter("controller.feedforward.model",
                      "constant",
                      read_only_descriptor);
    auto feedforward_model_selection =
        get_parameter("controller.feedforward.model").as_string();
    if(feedforward_model_selection == "constant") {
        feedforward_model_ = FeedforwardModel::CONSTANT;
    } else if(feedforward_model_selection == "proportional") {
        feedforward_model_ = FeedforwardModel::PROPORTIONAL;
    } else if(feedforward_model_selection == "cvt") {
        feedforward_model_ = FeedforwardModel::CVT;
    } else {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Invalid feedforward model selected.");
    }

    rcl_interfaces::msg::FloatingPointRange offset_range;
    offset_range.set__from_value(0.0).set__to_value(1.0).set__step(0.001);
    rcl_interfaces::msg::ParameterDescriptor offset_descriptor;
    offset_descriptor.floating_point_range = {offset_range};

    declare_parameter("controller.feedforward.offset", 0.0, offset_descriptor);
    feedforward_offset_ =
        get_parameter("controller.feedforward.offset").as_double();

    declare_parameter("controller.gains.proportional", 1.0, gain_descriptor);
    proportional_gain_ =
        get_parameter("controller.gains.proportional").as_double();

    declare_parameter("controller.gains.integral", 0.1, gain_descriptor);
    integral_gain_ = get_parameter("controller.gains.integral").as_double();

    declare_parameter("controller.gains.derivative", 0.0, gain_descriptor);
    derivative_gain_ = get_parameter("controller.gains.derivative").as_double();

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

    declare_parameter("controller.output.minimum", 0.0, read_only_descriptor);
    minimum_output_ = get_parameter("controller.output.minimum").as_double();

    declare_parameter("controller.output.maximum", 1.0, read_only_descriptor);
    maximum_output_ = get_parameter("controller.output.maximum").as_double();

    declare_parameter("controller.output.change.enabled", true);
    use_output_change_limiter_ =
        get_parameter("controller.output.change.enabled").as_bool();

    declare_parameter("controller.output.change.magnitude", 0.001);
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
                      40.0,
                      integral_term_descriptor);
    maximum_integral_term_ =
        get_parameter("controller.limit.integral.magnitude").as_double();

    declare_parameter("brake.limit", 0.4);
    brake_limit_ = get_parameter("brake.limit").as_double();

    declare_parameter("brake.threshold", 0.2);
    brake_threshold_ = get_parameter("brake.threshold").as_double();

    // Create the parameters callback handle.
    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&SpeedControllerNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void SpeedControllerNode::Initialize() {
    // Initialize the PID controller.
    controller_ = std::make_shared<PIDController>();
    controller_->SetFeedforwardGain(feedforward_gain_);
    controller_->SetProportionalGain(proportional_gain_);
    controller_->SetIntegralGain(integral_gain_);
    controller_->SetDerivativeGain(derivative_gain_);
    controller_->UseSetpointRamping(use_setpoint_ramping_);
    controller_->SetMaximumSetpointChange(maximum_setpoint_change_);
    controller_->UseOutputLimiter(true);
    controller_->SetMinimumOutput(minimum_output_);
    controller_->SetMaximumOutput(maximum_output_);
    controller_->UseOutputChangeLimiter(use_output_change_limiter_);
    controller_->SetMaximumOutputChange(maximum_output_change_);
    controller_->UseIntegralTermLimiter(use_integral_term_limiter_);
    controller_->SetMaximumIntegralTerm(maximum_integral_term_);

    // Initialize the throttle ramp function.
    // TODO: Make the ramp time a parameter
    ramp_ = std::make_unique<RampFunction>(1.0);

    // Initialize the feedforward model.
    cvt_model_ = std::make_shared<CVTFeedforwardModel>();

    // Initialize the atomic variables.
    has_feedback_ = false;
    has_setpoint_ = false;
    is_stopped_ = true;

    RCLCPP_DEBUG(get_logger(), "Speed controller ready!");
}

void SpeedControllerNode::Activate() {
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();
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
    start_service_server_ = create_service<std_srvs::srv::Trigger>(
        "start",
        std::bind(&SpeedControllerNode::StartServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    stop_service_server_ = create_service<std_srvs::srv::Trigger>(
        "stop",
        std::bind(&SpeedControllerNode::StopServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

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
}

void SpeedControllerNode::CreatePublishers() {
    throttle_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "throttle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    if(minimum_output_ < 0.0) {
        brake_publisher_ =
            create_publisher<olav_interfaces::msg::SetpointStamped>(
                "brake",
                RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Minimum output is a positive number: no brake efforts "
                    "will be published.");
    }

    status_publisher_ = create_publisher<olav_interfaces::msg::PIDStatus>(
        "status",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

rcl_interfaces::msg::SetParametersResult
SpeedControllerNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    for(const auto& parameter : parameters) {
        if(parameter.get_name() == "controller.gains.feedforward") {
            controller_->SetFeedforwardGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.gains.proportional") {
            controller_->SetProportionalGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.gains.integral") {
            controller_->SetIntegralGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.gains.derivative") {
            controller_->SetDerivativeGain(parameter.as_double());
        } else if(parameter.get_name() == "controller.feedforward.offset") {
            controller_->SetFeedforwardOffset(parameter.as_double());
        } else if(parameter.get_name() == "controller.output.minimum") {
            controller_->SetMinimumOutput(parameter.as_double());
            // TODO: Some logic to enable or disable publisher here.
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
    if(is_stopped_) { return; }

    // Update the PID controller setpoint.
    controller_->SetSetpoint(target_speed_message->setpoint);
    if(!has_setpoint_) {
        has_setpoint_ = true;
        RCLCPP_DEBUG(
            get_logger(),
            "Received first setpoint message since initialization: %0.2lf m/s",
            target_speed_message->setpoint);
    }
}

void SpeedControllerNode::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    if(is_stopped_) { return; }

    // TODO: Implement a model for CVT transmissions.
    // Update the PID controller feedback.
    /*
    cvt_model_->SetInclination(
        Eigen::Quaterniond(odometry_message->pose.pose.orientation.w,
                           odometry_message->pose.pose.orientation.x,
                           odometry_message->pose.pose.orientation.y,
                           odometry_message->pose.pose.orientation.z)
            .toRotationMatrix()
            .eulerAngles(0, 1, 2)
            .y());

    */

    current_speed_ = odometry_message->twist.twist.linear.x;

    // Update the PID controller feedback.
    controller_->SetFeedback(odometry_message->twist.twist.linear.x);
    if(!has_feedback_) {
        has_feedback_ = true;
        RCLCPP_DEBUG(
            get_logger(),
            "Received first feedback message since initialization: %0.2lf m/s",
            odometry_message->twist.twist.linear.x);
    }
}

void SpeedControllerNode::ControlTimerCallback() {
    if((is_stopped_) || (!has_setpoint_) || (!has_feedback_)) { return; }

    // TODO: Switch this to a deadzone setting.
    if(controller_->GetSetpoint() > 0.0 && current_speed_ < minimum_speed_) {
        RCLCPP_DEBUG(get_logger(), "Moving to controllable region ...");

        auto current_time = get_clock()->now().seconds();

        if(ramp_->Enabled()) {
            ramp_->Start(current_time);
            double throttle = ramp_->Tick(current_time);
            PublishThrottle(throttle);
            return;
        }

        // Get the elapsed time - it does not need to be extremely precise, so
        // seconds() is okay.
        double throttle = ramp_->Tick(current_time);
        PublishThrottle(throttle);
    } else {
        ramp_->Reset();
    }

    // Apply the feedforward offset for all strictly positive setpoints.
    if(controller_->GetSetpoint() > 0.0) {
        controller_->SetFeedforwardOffset(feedforward_offset_);
    } else {
        controller_->SetFeedforwardOffset(0.0);
    }

    // Update the unscaled feedforward term.
    switch(feedforward_model_) {
    case FeedforwardModel::CONSTANT:
        controller_->SetUnscaledFeedforwardTerm(0.0);
        break;
    case FeedforwardModel::PROPORTIONAL:
        controller_->SetUnscaledFeedforwardTerm(controller_->GetSetpoint());
        break;
    default: break;
    };

    // Tick the controller.
    controller_->Tick();

    auto output = controller_->GetOutput();

    // Publish the throttle effort.
    auto throttle_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    throttle_message->header.frame_id = "internal";
    throttle_message->setpoint = (output >= 0) ? output : 0.0;
    throttle_publisher_->publish(*throttle_message);

    // Publish the brake effort.
    if(minimum_output_ < 0.0) {
        auto brake_message =
            std::make_shared<olav_interfaces::msg::SetpointStamped>();
        brake_message->header.frame_id = "internal";
        brake_message->setpoint = (output < -brake_threshold_)
            ? boost::algorithm::clamp(-output, 0.0, brake_limit_)
            : 0.0;
        brake_publisher_->publish(*brake_message);
    }
}

void SpeedControllerNode::StartServiceCallback(
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

    std::string message("Speed controller started.");
    RCLCPP_DEBUG(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
}

void SpeedControllerNode::StopServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Suppress unused variable compiler warning.
    (void)request;

    // Delete the subscriptions and publishers, stop the timer.
    odometry_subscription_.reset();
    target_speed_subscription_.reset();
    throttle_publisher_.reset();
    brake_publisher_.reset();
    control_timer_->cancel();

    // Reset the PID controller.
    controller_->Reset();

    // Mark the controller as stopped.
    is_stopped_ = true;

    std::string message("Speed controller stopped.");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
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

void SpeedControllerNode::PublishThrottle(const double& throttle) {
    // Publish the throttle effort.
    auto throttle_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    throttle_message->header.frame_id = "internal";
    throttle_message->setpoint = throttle;
    throttle_publisher_->publish(*throttle_message);
}

} // namespace ROS
} // namespace OLAV