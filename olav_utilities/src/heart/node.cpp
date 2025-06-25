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

#include <olav_utilities/heart/node.hpp>

namespace OLAV {
namespace ROS {

HeartNode::HeartNode() : rclcpp::Node("heart") {
    Configure();
    Activate();
}

void HeartNode::Configure() {
    GetParameters();
}

void HeartNode::GetParameters() {
    declare_parameter("rate", 1.0);
    heartbeat_period_ = 1.0 / get_parameter("rate").as_double();

    declare_parameter("frame_id", "olav");
    frame_id_ = get_parameter("frame_id").as_string();
}

void HeartNode::Activate() {
    CreateTimers();
    CreatePublishers();
    StartTimers();
}

void HeartNode::CreateTimers() {
    heartbeat_timer_ =
        create_wall_timer(std::chrono::duration<double>(heartbeat_period_),
                          std::bind(&HeartNode::HeartbeatTimerCallback, this));
    heartbeat_timer_->cancel();
}

void HeartNode::CreatePublishers() {
    heartbeat_publisher_ = create_publisher<std_msgs::msg::Header>(
        "heartbeat", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void HeartNode::StartTimers() {
    RCLCPP_INFO(get_logger(), "Starting timers ...");

    heartbeat_timer_->reset();
}

void HeartNode::HeartbeatTimerCallback() {
    std_msgs::msg::Header heartbeat_message;
    heartbeat_message.stamp = get_clock()->now();
    heartbeat_message.frame_id = frame_id_;
    heartbeat_publisher_->publish(heartbeat_message);
}

}  // namespace ROS
}  // namespace OLAV
