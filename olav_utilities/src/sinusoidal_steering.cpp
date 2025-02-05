#include <olav_utilities/sinusoidal_steering.hpp>

namespace OLAV {
namespace ROS {

SinusoidalSteeringNode::SinusoidalSteeringNode()
    : rclcpp::Node("sinusoidal_steering") {}

void SinusoidalSteeringNode::Configure() {
    GetParameters();
    Activate();
}

void SinusoidalSteeringNode::GetParameters() {}

void SinusoidalSteeringNode::Initialize() {}

void SinusoidalSteeringNode::Activate() {}

void SinusoidalSteeringNode::CreateSubscriptions() {
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&SinusoidalSteeringNode::OdometryCallback,
                  this,
                  std::placeholders::_1));
}

void SinusoidalSteeringNode::CreatePublishers() {
    ackermann_drive_publisher_ =
        create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_angle_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "steering_angle",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void SinusoidalSteeringNode::CreateActions() {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
        this,
        "start",
        std::bind(&SinusoidalSteeringNode::SinusoidalSteeringGoalStart,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&SinusoidalSteeringNode::SinusoidalSteeringGoalCancel,
                  this,
                  std::placeholders::_1),
        std::bind(&SinusoidalSteeringNode::SinusoidalSteeringGoalAccept,
                  this,
                  std::placeholders::_1));
}

} // namespace ROS
} // namespace OLAV