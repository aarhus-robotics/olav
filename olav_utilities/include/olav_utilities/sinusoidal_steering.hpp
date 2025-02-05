#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <olav_interfaces/action/sinuoidal_steering.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace OLAV {
namespace ROS {

class SinusoidalSteeringNode : public rclcpp::Node {
  public:
    SinusoidalSteeringNode();

  protected:
    void Configure();
    void GetParameters();
    void Initialize();
    void Activate();
    void CreateSubscriptions();
    void CreatePublishers();
    void CreateActions();

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_drive_publisher_;

    rclcpp_action::Server<
        olav_interfaces::action::SinusoidalSteering>::SharedPtr action_server_;

    rclcpp_action::GoalResponse SinusoidalSteeringGoalStart(
        const rclcpp_action::GoalUUI& uuid,
        std::shared_ptr<const olav_interfaces::action::SinusoidalSteering>
            goal) {
        (void)uuid;
        RCLCPP_INFO(get_logger(),
                    "Received sinusoidal steering start request!");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SinusoidalSteeringGoalCancel(
        const std::shared_ptr<olav_interfaces::action::SinusoidalSteering>
            goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(),
                    "Received sinusoidal steering cancel request!");

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SinusoidalSteeringGoalAccept(
        const std::shared_ptr<olav_interfaces::action::SinusoidalSteering>
            goal_handle) {
        std::thread{
            std::bind(&SinusoidalSteeringNode::SinusoidalSteeringGoalExecute,
                      this,
                      std::placeholders::_1),
            goal_handle}
            .detach();
    }

    void SinusoidalSteeringGoalExecuto(
        const std::shared_ptr<olav_interfaces::action::SinusoidalSteering>
            goal_handle) {
        RCLCPP_INFO(get_logger(), "Executing sinusoidal steering");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<
            olav_interfaces::action::SinusoidalSteering::Feedback>();

        ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_message;
        ackermann_drive_message.header.frame_id = "olav-act-sst-4fg3";
        ackermann_drive_message.header.stamp = get_clock()->now();
        ackermann_drive_message.speed = goal_handle.target_speed;
        ackermann_drive_publisher_->publish(ackermann_drive_message);

        while(vehicle_speed_ < goal_handle.target_speed) {}

        auto start_time = get_clock()->now();
        auto time = start_time;

        while(time - start_time < end_time) {
            ackermann_drive_message.header.stamp = get_clock()->now();
            ackermann_drive_message.steering_angle = GetSteeringAngle(time);
            ackermann_drive_message.speed = goal_handle.target_speed;
            ackermann_drive_publisher_->publish(ackermann_drive_message);
        }

        feedback->has_speed_;
        auto result = std::make_shared<
            olav_interfaces::action::SinusoidalSteering::Result>();

        for(int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if(goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            // Update sequence
            sequence.push_back(sequence[i] + sequence[i - 1]);
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }
    }

} // namespace ROS
} // namespace OLAV

// RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
