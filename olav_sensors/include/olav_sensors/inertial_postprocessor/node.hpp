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

#include <nav_msgs/msg/odometry.hpp>
#include <olav_core/filters/one_euro_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace OLAV {
namespace ROS {

class InertialPostprocessorNode : public rclcpp::Node {
  public:
    InertialPostprocessorNode();

  private:
    void Configure();

    void DeclareFloatingPointParameter(const std::string& name,
                                       const double& default_value,
                                       const double& minimum_value,
                                       const double& maximum_value);

    void GetParameters();

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult
    SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void Initialize();

    void Activate();

    void CreateSubscriptions();

    void CreateTimers();

    void CreatePublishers();

    void StartTimers();

    // Filter

    /** @brief Odometry subscription. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    void
    OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

    bool has_odometry_ = false;

    /** @brief Filter tick timer. */
    rclcpp::TimerBase::SharedPtr filter_timer_;

    void FilterCallback();

    nav_msgs::msg::Odometry::SharedPtr latest_odometry_message_;

    void
    FilterOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

    void
    FlattenOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

    void
    PublishTransform(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

    /** @brief Odometry longitudinal speed component filter. */
    std::shared_ptr<OneEuroFilter> speed_filter_;

    /** @brief Filtered odometry publisher. */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // Transforms
    // ----------

    /** @brief Transform broadcaster. */
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    /** @brief Source frame. */
    std::string source_frame_;

    /** @brief Target frame. */
    std::string target_frame_;
};

} // namespace ROS
} // namespace OLAV