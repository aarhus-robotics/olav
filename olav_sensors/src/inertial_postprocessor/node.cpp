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

#include <olav_sensors/inertial_postprocessor/node.hpp>

namespace OLAV {
namespace ROS {

InertialPostprocessorNode::InertialPostprocessorNode()
    : rclcpp::Node("inertial_postprocessor") {
    Configure();
    Activate();
}

void InertialPostprocessorNode::Configure() {
    GetParameters();
    Initialize();
}

void InertialPostprocessorNode::GetParameters() {
    declare_parameter("rate", 100.0);
    DeclareFloatingPointParameter("filter.cutoff.signal", 20.0, 0.0, 100.0);
    DeclareFloatingPointParameter("filter.cutoff.derivative",
                                  100.0,
                                  0.0,
                                  100.0);
    DeclareFloatingPointParameter("filter.speed", 0.1, 0.0, 1.0);
    declare_parameter("transform.source", "base_link_3d");
    declare_parameter("transform.target", "base_link");

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&InertialPostprocessorNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
}

void InertialPostprocessorNode::Initialize() {
    speed_filter_ = std::make_shared<OneEuroFilter>(
        get_parameter("rate").as_double(),
        get_parameter("filter.cutoff.signal").as_double(),
        get_parameter("filter.cutoff.derivative").as_double(),
        get_parameter("filter.speed").as_double());

    transform_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    source_frame_ = get_parameter("transform.source").as_string();
    target_frame_ = get_parameter("transform.target").as_string();
}

rcl_interfaces::msg::SetParametersResult
InertialPostprocessorNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;

    for(const auto& parameter : parameters) {
        if(parameter.get_name() == "filter.cutoff.signal") {
            speed_filter_->SetCutoffFrequency(parameter.as_double());
        } else if(parameter.get_name() == "filter.cutoff.derivative") {
            speed_filter_->SetDerivativeCutoffFrequency(parameter.as_double());

        } else if(parameter.get_name() == "filter.speed") {
            speed_filter_->SetSpeedCoefficient(parameter.as_double());
        } else if(parameter.get_name().rfind("qos_overrides", 0) !=
                  std::string::npos) {
        } else {
            RCLCPP_WARN(get_logger(),
                        "Parameter \"%s\" does not support reconfigure!",
                        parameter.get_name().c_str());
            result.successful = false;
            return result;
        }
        RCLCPP_INFO(get_logger(),
                    "Parameter \"%s\" set!",
                    parameter.get_name().c_str());
    }

    result.successful = true;
    return result;
}

void InertialPostprocessorNode::Activate() {
    CreateSubscriptions();
    CreateTimers();
    CreatePublishers();
    StartTimers();
}

void InertialPostprocessorNode::CreateSubscriptions() {
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "in/odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&InertialPostprocessorNode::OdometryCallback,
                  this,
                  std::placeholders::_1));
}

void InertialPostprocessorNode::CreateTimers() {
    filter_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / get_parameter("rate").as_double()),
        std::bind(&InertialPostprocessorNode::FilterCallback, this));
    filter_timer_->cancel();
}

void InertialPostprocessorNode::CreatePublishers() {
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "out/odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void InertialPostprocessorNode::StartTimers() { filter_timer_->reset(); }

void InertialPostprocessorNode::FilterCallback() {
    if(!has_odometry_) return;

    auto odometry_message =
        std::make_shared<nav_msgs::msg::Odometry>(*latest_odometry_message_);

    FilterOdometry(odometry_message);

    // Project the three-dimensional odometry on the XY plane.
    FlattenOdometry(odometry_message);

    // Publish the updated odometry.
    odometry_publisher_->publish(*odometry_message);
}

void InertialPostprocessorNode::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    latest_odometry_message_ = odometry_message;
    has_odometry_ = true;

    PublishTransform(odometry_message);
}

void InertialPostprocessorNode::FilterOdometry(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    odometry_message->twist.twist.linear.x =
        speed_filter_->Filter(odometry_message->twist.twist.linear.x);
}

void InertialPostprocessorNode::FlattenOdometry(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    odometry_message->pose.pose.position.z = 0.0;
}

void InertialPostprocessorNode::PublishTransform(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    geometry_msgs::msg::TransformStamped odom_to_base_link_transform;
    odom_to_base_link_transform.header.stamp = get_clock()->now();
    odom_to_base_link_transform.header.frame_id = source_frame_;
    odom_to_base_link_transform.child_frame_id = target_frame_;
    odom_to_base_link_transform.transform.translation.z =
        -odometry_message->pose.pose.position.z;
    transform_broadcaster_->sendTransform(odom_to_base_link_transform);
}

void InertialPostprocessorNode::DeclareFloatingPointParameter(
    const std::string& name,
    const double& default_value,
    const double& minimum_value,
    const double& maximum_value) {
    // Instantiate the parameter descriptor.
    auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    parameter_descriptor.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    // Define the floating point range - note how a null step implies no step
    // constraints for the floating point parameter.
    rcl_interfaces::msg::FloatingPointRange floating_point_range;
    floating_point_range.set__from_value(minimum_value);
    floating_point_range.set__to_value(maximum_value);
    floating_point_range.set__step(0.0);
    parameter_descriptor.floating_point_range = {floating_point_range};

    // Declare the parameter paired with its matching descriptor on the ROS
    // node.
    declare_parameter(name,
                      rclcpp::ParameterValue(default_value),
                      parameter_descriptor);
}

} // namespace ROS
} // namespace OLAV