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

#include <cmath>
#include <thread>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <olav_interfaces/msg/setpoint_stamped.hpp>
#include <olav_sensors/powertrain/exceptions.hpp>
#include <olav_sensors/powertrain/interface.hpp>

namespace OLAV {
namespace ROS {

class PowertrainInterfaceNode : public rclcpp::Node {
  public:
    PowertrainInterfaceNode();

  protected:
    void Configure();
    void GetParameters();
    void Initialize();
    void Activate();
    void CreateTimers();
    void CreatePublishers();

  private:
    void TimerCallback();

    /** @brief Shared pointer to the publisher for filtered engine speed. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        filtered_engine_speed_publisher_;

    /** @brief Shared pointer to the publisher for filtered wheel speed. */
    rclcpp::Publisher<olav_interfaces::msg::SetpointStamped>::SharedPtr
        filtered_vehicle_speed_publisher_;

    /** @brief Shared pointer to the timer for message publishing. */
    rclcpp::TimerBase::SharedPtr timer_;

    std::string odometry_frame_id_;

    std::string odometry_child_frame_id_;

    /** @brief Shared pointer to the publisher for estimated vehicle odometry.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    double odometry_twist_covariance_;

    /** @brief Shared pointer to the microcontroller odometer interface. */
    std::shared_ptr<OLAV::Interfaces::PowertrainInterface> interface_;

    std::string port_;

    int baudrate_;

    int timeout_;

    double rate_;

    double wheel_radius_;

    int engine_speed_filter_samples_;

    int axle_speed_filter_samples_;

    std::shared_ptr<boost::accumulators::accumulator_set<
        double,
        boost::accumulators::stats<boost::accumulators::tag::rolling_mean>>>
        engine_speed_accumulator_;

    std::shared_ptr<boost::accumulators::accumulator_set<
        double,
        boost::accumulators::stats<boost::accumulators::tag::rolling_mean>>>
        axle_speed_accumulator_;
};

} // namespace ROS
} // namespace OLAV
