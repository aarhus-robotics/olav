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

#include <iostream>
#include <queue>

#include <nlohmann/json.hpp>
#include <olav_sensors/serial/crc32.hpp>
#include <serial/serial.h>

#include <olav_sensors/serial/status_message.hpp>

namespace OLAV {
namespace ROS {

/**
 * @brief Interface to a generic device (e.g. a microcontroller) communicating
 * via a serial port. The interface assumes JSON sentences are published to the
 * serial port by the device in a specific format, with delimiters defining
 * checksum and data tokens.
 *
 */
class SerialDeviceInterface {
  public:
    /**
     * @brief Construct a new serial device interface.
     */
    SerialDeviceInterface(const std::string& port, const int& baudrate);

    /**
     * @brief Connect to the device serial port.
     *
     */
    void Connect();

    /**
     * @brief Check whether the serial device is connected.
     *
     * @return true The serial device is connected.
     * @return false The serial device is not connected.
     */
    const bool& IsConnected() const;

    /**
     * @brief Read a valid JSON document from the serial port.
     *
     * @return nlohmann::json JSON document read on the serial port.
     */
    nlohmann::json Read();

    /**
     * @brief Get the serial device status from a JSON document.
     *
     * @param message JSON document read from serial port.
     * @return SerialDeviceStatusMessage Serial device status message.
     */
    SerialDeviceStatusMessage GetStatus(nlohmann::json& message);

  private:
    /** @brief Unique pointer to the serial interface. */
    std::unique_ptr<serial::Serial> interface_;

    /** @brief Serial connection port (in the form of a device address, e.g.
     * "/dev/ttyACM0"). */
    std::string port_ = "/dev/ttyACM0";

    /** @brief Serial connection baudrate. */
    int baudrate_ = 9600;

    /**
     * @brief Initialize the serial interface with the user-provided settings.
     */
    void Initialize();

    /**
     * @brief Read bytes into a string from the serial port until a given
     * delimiter marking the end of the sentence is found.
     *
     * @param character Delimiter marking the end of the sentence.
     * @return std::string
     */
    std::string ReadUntil(std::string delimiter);

    /**
     * @brief Check whether a string token matches a provided CRC32 checksum.
     *
     * @param token Token of which to compute the checksum.
     * @param checksum Checksum to check against.
     * @return true The token matches the checksum.
     * @return false The token does not match the checksum.
     */
    bool IsChecksumValid(std::string token, std::string checksum);

    /** @brief Whether or not the connection to the serial device is open. */
    bool is_connected_ = false;

    /** @brief Message queue. */
    std::queue<nlohmann::json> queue_;
};

} // namespace ROS
} // namespace OLAV