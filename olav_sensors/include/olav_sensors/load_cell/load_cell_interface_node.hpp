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

#include <rclcpp/rclcpp.hpp>

#include <olav_sensors/load_cell/load_cell_interface.hpp>

namespace OLAV {
namespace ROS {

class LoadCellInterfaceNode : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new load cell interface node.
     */
    LoadCellInterfaceNode();

  protected:
    /**
     * @brief Declare and retrieve the load cell interface node parameters.
     */
    void GetParameters();

    /**
     * @brief Initialize the load cell interface node class members.
     */
    void Initialize();

    /**
     * @brief Create the load cell interface node timers.
     */
    void CreateTimers();

    /**
     * @brief Create the load cell interface node publishers.
     */
    void CreatePublishers();

  private:
    // Load cell interface
    // -------------------
    std::unique_ptr<LoadCellInterface> load_cell_interface_;

    std::string port_;

    int baudrate_;

    void Connect();

    // Read serial timer
    // -----------------
    /** @brief Read serial timer shared pointer. */
    rclcpp::TimerBase::SharedPtr read_serial_timer_;

    /** @brief Read serial timer tick rate. */
    double read_serial_rate_;

    /**
     * @brief Read serial timer callback function.
     */
    void ReadSerialCallback();
};

} // namespace ROS
} // namespace OLAV