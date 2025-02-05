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
    parse_rate_ = get_parameter("rate").as_double();

    declare_parameter("connection.port", "/dev/powertrain");
    port_ = get_parameter("connection.port").as_string();

    declare_parameter("connection.baudrate", 9600);
    baudrate_ = get_parameter("connection.baudrate").as_int();

    declare_parameter("connection.timeout", 5);
    timeout_ = get_parameter("connection.timeout").as_int();

    declare_parameter("wheel_radius", 0.3175);
    wheel_radius_ = get_parameter("wheel_radius").as_double();

    declare_parameter("filter.engine.minimum", 0.0);
    declare_parameter("filter.engine.maximum", 6350.0);
    declare_parameter("filter.engine.change", 10.0);
    declare_parameter("filter.engine.scale", 0.1);
    declare_parameter("filter.engine.cutoff", 0.005);
    declare_parameter("filter.engine.derivative_cutoff", 0.05);
    declare_parameter("filter.engine.speed_coefficient", 0.005);

    declare_parameter("filter.wheel.minimum", 0.0);
    declare_parameter("filter.wheel.maximum", 36.11);
    declare_parameter("filter.wheel.change", 0.25);
    declare_parameter("filter.wheel.scale", 0.1);
    declare_parameter("filter.wheel.cutoff", 0.005);
    declare_parameter("filter.wheel.derivative_cutoff", 0.05);
    declare_parameter("filter.wheel.speed_coefficient", 0.005);

    callback_handle_ = add_on_set_parameters_callback(
        std::bind(&PowertrainInterfaceNode::SetParametersCallback,
                  this,
                  std::placeholders::_1));
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
        std::chrono::duration<double>(1.0 / parse_rate_),
        std::bind(&PowertrainInterfaceNode::ParseTimerCallback, this));
    parse_timer_->cancel();

    diagnostic_timer_ = create_wall_timer(
        std::chrono::duration<double>(0.5),
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

    tachometer_odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
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

    {
        const std::lock_guard<std::mutex> filters_lock(filters_mutex_);

        // Retrieve, filter and publish the engine speed.
        engine_speed_filter_->Filter(interface_->GetEngineSpeed());
    }

    auto engine_speed_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    engine_speed_message->header.stamp = get_clock()->now();
    engine_speed_message->setpoint = engine_speed_filter_->GetFiltered();
    engine_speed_publisher_->publish(*engine_speed_message);

    // Retrieve, filter and publish the wheel speed.
    double wheel_speed =
        interface_->GetAxleSpeed() * 2.0 * M_PI * wheel_radius_;

    {
        const std::lock_guard<std::mutex> filters_lock(filters_mutex_);

        wheel_speed_filter_->Filter(wheel_speed);
    }

    auto wheel_speed_message =
        std::make_shared<olav_interfaces::msg::SetpointStamped>();
    wheel_speed_message->header.stamp = get_clock()->now();
    wheel_speed_message->setpoint = wheel_speed_filter_->GetFiltered();
    tachometer_speed_publisher_->publish(*wheel_speed_message);

    nav_msgs::msg::Odometry wheel_odometry_message;
    wheel_odometry_message.header.stamp = get_clock()->now();
    wheel_odometry_message.twist.twist.linear.x =
        wheel_speed_filter_->GetFiltered();
    tachometer_odometry_publisher_->publish(wheel_odometry_message);
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
    engine_speed_filter_ =
        std::make_shared<OutlierResistantFilter>(parse_rate_);
    engine_speed_filter_->SetMinimumValue(
        get_parameter("filter.engine.minimum").as_double());
    engine_speed_filter_->SetMaximumValue(
        get_parameter("filter.engine.maximum").as_double());
    engine_speed_filter_->SetMaximumChange(
        get_parameter("filter.engine.change").as_double());
    engine_speed_filter_->SetScalingFactor(
        get_parameter("filter.engine.scale").as_double());
    engine_speed_filter_->SetCutoffFrequency(
        get_parameter("filter.engine.cutoff").as_double());
    engine_speed_filter_->SetDerivativeCutoffFrequency(
        get_parameter("filter.engine.derivative_cutoff").as_double());
    engine_speed_filter_->SetSpeedCoefficient(
        get_parameter("filter.engine.speed_coefficient").as_double());

    wheel_speed_filter_ = std::make_shared<OutlierResistantFilter>(parse_rate_);
    wheel_speed_filter_->SetMinimumValue(
        get_parameter("filter.wheel.minimum").as_double());
    wheel_speed_filter_->SetMaximumValue(
        get_parameter("filter.wheel.maximum").as_double());
    wheel_speed_filter_->SetMaximumChange(
        get_parameter("filter.wheel.change").as_double());
    wheel_speed_filter_->SetScalingFactor(
        get_parameter("filter.wheel.scale").as_double());
    wheel_speed_filter_->SetCutoffFrequency(
        get_parameter("filter.wheel.cutoff").as_double());
    wheel_speed_filter_->SetDerivativeCutoffFrequency(
        get_parameter("filter.wheel.derivative_cutoff").as_double());
    wheel_speed_filter_->SetSpeedCoefficient(
        get_parameter("filter.wheel.speed_coefficient").as_double());

    interface_ =
        std::make_shared<OLAV::Interfaces::PowertrainInterface>(port_,
                                                                baudrate_,
                                                                timeout_);
}

rcl_interfaces::msg::SetParametersResult
PowertrainInterfaceNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    {
        const std::lock_guard<std::mutex> filters_lock(filters_mutex_);

        for(const auto& parameter : parameters) {
            if(parameter.get_name() == "filter.wheel.minimum") {
                wheel_speed_filter_->SetMinimumValue(parameter.as_double());
            } else if(parameter.get_name() == "filter.wheel.maximum") {
                wheel_speed_filter_->SetMaximumValue(parameter.as_double());
            } else if(parameter.get_name() == "filter.wheel.change") {
                wheel_speed_filter_->SetMaximumChange(parameter.as_double());
            } else if(parameter.get_name() == "filter.wheel.scale") {
                wheel_speed_filter_->SetScalingFactor(parameter.as_double());
            } else if(parameter.get_name() == "filter.wheel.cutoff") {
                wheel_speed_filter_->SetCutoffFrequency(parameter.as_double());
            } else if(parameter.get_name() ==
                      "filter.wheel.derivative_cutoff") {
                wheel_speed_filter_->SetDerivativeCutoffFrequency(
                    parameter.as_double());
            } else if(parameter.get_name() ==
                      "filter.wheel.speed_coefficient") {
                wheel_speed_filter_->SetSpeedCoefficient(parameter.as_double());
            } else if(parameter.get_name() == "filter.engine.minimum") {
                engine_speed_filter_->SetMinimumValue(parameter.as_double());
            } else if(parameter.get_name() == "filter.engine.maximum") {
                engine_speed_filter_->SetMaximumValue(parameter.as_double());
            } else if(parameter.get_name() == "filter.engine.change") {
                engine_speed_filter_->SetMaximumChange(parameter.as_double());
            } else if(parameter.get_name() == "filter.engine.scale") {
                engine_speed_filter_->SetScalingFactor(parameter.as_double());
            } else if(parameter.get_name() == "filter.engine.cutoff") {
                engine_speed_filter_->SetCutoffFrequency(parameter.as_double());
            } else if(parameter.get_name() ==
                      "filter.engine.derivative_cutoff") {
                engine_speed_filter_->SetDerivativeCutoffFrequency(
                    parameter.as_double());
            } else if(parameter.get_name() ==
                      "filter.engine.speed_coefficient") {
                engine_speed_filter_->SetSpeedCoefficient(
                    parameter.as_double());
            } else {
                RCLCPP_WARN(get_logger(),
                            "Parameter \"%s\" does not support reconfigure!",
                            parameter.get_name().c_str());
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = false;
                return result;
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

} // namespace ROS
} // namespace OLAV