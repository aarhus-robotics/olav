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

#include <olav_sensors/load_cell/load_cell_interface_node.hpp>

namespace OLAV {
namespace ROS {

LoadCellInterfaceNode::LoadCellInterfaceNode()
    : rclcpp::Node("load_cell_interface") {
    GetParameters();
    Initialize();
    CreateTimers();

    read_serial_timer_->reset();
}

void LoadCellInterfaceNode::GetParameters() {
    declare_parameter("port", "/dev/ttyACM0");
    port_ = get_parameter("port").as_string();

    declare_parameter("baudrate", 115200);
    baudrate_ = get_parameter("baudrate").as_int();

    declare_parameter("rate", 2.0);
    read_serial_rate_ = get_parameter("rate").as_double();
}

void LoadCellInterfaceNode::Initialize() {
    load_cell_interface_ =
        std::make_unique<LoadCellInterface>(port_, baudrate_);
    Connect();
}

void LoadCellInterfaceNode::CreateTimers() {
    read_serial_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / read_serial_rate_),
        std::bind(&LoadCellInterfaceNode::ReadSerialCallback, this));
    read_serial_timer_->cancel();
}

void LoadCellInterfaceNode::CreatePublishers() {}

void LoadCellInterfaceNode::Connect() {
    if(!load_cell_interface_->IsConnected()) {
        RCLCPP_INFO(get_logger(), "Attempting connection ...");
        load_cell_interface_->Connect();
    }
}

void LoadCellInterfaceNode::ReadSerialCallback() {
    load_cell_interface_->Read();
}

} // namespace ROS
} // namespace OLAV