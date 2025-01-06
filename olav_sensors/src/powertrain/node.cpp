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

#include <olav_sensors/powertrain/node.hpp>

namespace OLAV {
namespace ROS {

PowertrainInterfaceNode::PowertrainInterfaceNode()
    : rclcpp::Node("powertrain_interface") {
    Configure();
    Activate();
}

void PowertrainInterfaceNode::Configure() {
    GetParameters();
    Initialize();
}

void PowertrainInterfaceNode::GetParameters() {
    declare_parameter("rate", 100.0);
    parse_period_ = 1.0 / get_parameter("rate").as_double();

    declare_parameter("connection.port", "/dev/powertrain");
    port_ = get_parameter("connection.port").as_string();

    declare_parameter("connection.baudrate", 9600);
    baudrate_ = get_parameter("connection.baudrate").as_int();

    declare_parameter("connection.timeout", 5);
    timeout_ = get_parameter("connection.timeout").as_int();

    declare_parameter("wheel_radius", 0.3175);
    wheel_radius_ = get_parameter("wheel_radius").as_double();

    declare_parameter("filter.engine.samples", 200);
    engine_speed_filter_samples_ =
        get_parameter("filter.engine.samples").as_int();

    declare_parameter("filter.axle.samples", 200);
    axle_speed_filter_samples_ = get_parameter("filter.axle.samples").as_int();

    declare_parameter("odometry.frame_id", "odom");
    odometry_frame_id_ = get_parameter("odometry.frame_id").as_string();

    declare_parameter("odometry.child_frame_id", "base_link");
    odometry_child_frame_id_ =
        get_parameter("odometry.child_frame_id").as_string();

    declare_parameter("odometry.covariance", 0.01);
    odometry_child_frame_id_ = get_parameter("odometry.covariance").as_double();
}

void PowertrainInterfaceNode::Activate() {
    CreateTimers();
    CreatePublishers();
    StartTimers();
}

void PowertrainInterfaceNode::CreateTimers() {
    connect_timer_ = create_wall_timer(
        std::chrono::duration<double>(5.0),
        std::bind(&PowertrainInterfaceNode::ConnectTimerCallback, this));
    connect_timer_->cancel();

    parse_timer_ = create_wall_timer(
        std::chrono::duration<double>(parse_period_),
        std::bind(&PowertrainInterfaceNode::ParseTimerCallback, this));
    parse_timer_->cancel();

    diagnostic_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0),
        std::bind(&PowertrainInterfaceNode::DiagnosticTimerCallback, this));
    diagnostic_timer_->cancel();
}

void PowertrainInterfaceNode::CreatePublishers() {
    engine_speed_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "engine/speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    tachometer_speed_publisher_ =
        create_publisher<olav_interfaces::msg::SetpointStamped>(
            "tachometer/speed",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "tachometer/odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    diagnostic_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
}

void PowertrainInterfaceNode::StartTimers() {
    connect_timer_->reset();
    parse_timer_->reset();
    diagnostic_timer_->reset();
}

void PowertrainInterfaceNode::ConnectTimerCallback() {
    if(interface_->IsOpen()) return;

    try {
        interface_->Open();
        RCLCPP_INFO(get_logger(),
                    "Connected to powertrain interface microcontroller!");
    } catch(OLAV::Exceptions::PowertrainInterfaceException& exception) {}
}

void PowertrainInterfaceNode::ParseTimerCallback() {
    if(!interface_->IsOpen()) return;

    auto time = get_clock()->now();

    auto engine_speed = interface_->GetEngineSpeed();

    (*engine_speed_accumulator_.get())(engine_speed);
    auto mean_engine_speed =
        boost::accumulators::rolling_mean(*engine_speed_accumulator_.get());

    auto filtered_engine_speed_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    filtered_engine_speed_message->header.stamp = time;
    filtered_engine_speed_message->setpoint = mean_engine_speed;
    engine_speed_publisher_->publish(*filtered_engine_speed_message);

    auto axle_speed = interface_->GetAxleSpeed();

    auto mean_axle_speed = axle_speed;
    (*axle_speed_accumulator_.get())(axle_speed);
    mean_axle_speed =
        boost::accumulators::rolling_mean(*axle_speed_accumulator_.get());

    auto filtered_axle_speed_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    filtered_axle_speed_message->header.stamp = time;
    filtered_axle_speed_message->setpoint =
        mean_axle_speed * 2.0 * M_PI * wheel_radius_;
    tachometer_speed_publisher_->publish(*filtered_axle_speed_message);

    auto odometry_message = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_message->header.frame_id = odometry_frame_id_;
    odometry_message->header.stamp = get_clock()->now();
    odometry_message->child_frame_id = odometry_child_frame_id_;
    odometry_message->twist.twist.linear.x =
        mean_axle_speed * 2.0 * M_PI * wheel_radius_;
    odometry_message->twist.covariance[0] = odometry_twist_covariance_;
    odometry_publisher_->publish(*odometry_message);
}

void PowertrainInterfaceNode::DiagnosticTimerCallback() {
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status_message;
    diagnostic_status_message.level = interface_->IsOpen()
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic_status_message.name = "olav/powertrain/connection";
    diagnostic_status_message.message = interface_->IsOpen()
        ? "Serial communication interface is open and transferring data."
        : "No available serial communication interface to the powertrain "
          "microcontroller. Please check the configure device address and the "
          "physical connection to the microcontroller.";
    diagnostic_status_message.hardware_id = hardware_id_;

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array_message;
    diagnostic_array_message.header.stamp = get_clock()->now();
    diagnostic_array_message.status.push_back(diagnostic_status_message);
    diagnostic_publisher_->publish(diagnostic_array_message);
}

void PowertrainInterfaceNode::Initialize() {
    engine_speed_accumulator_ =
        std::make_shared<boost::accumulators::accumulator_set<
            double,
            boost::accumulators::stats<
                boost::accumulators::tag::rolling_mean>>>(
            boost::accumulators::tag::rolling_window::window_size =
                engine_speed_filter_samples_);

    axle_speed_accumulator_ =
        std::make_shared<boost::accumulators::accumulator_set<
            double,
            boost::accumulators::stats<
                boost::accumulators::tag::rolling_mean>>>(
            boost::accumulators::tag::rolling_window::window_size =
                axle_speed_filter_samples_);

    interface_ =
        std::make_shared<OLAV::Interfaces::PowertrainInterface>(port_,
                                                                baudrate_,
                                                                timeout_);
}

} // namespace ROS
} // namespace OLAV