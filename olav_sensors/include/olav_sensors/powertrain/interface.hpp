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

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <serial/serial.h>

#include <olav_sensors/powertrain/exceptions.hpp>

namespace OLAV {
namespace Interfaces {

enum Status { STOPPED, CONNECTED, DISCONNECTED };

/**
 * @brief Serial interface to the powertrain microcontroller. The
 * microcontroller provides readings on engine and rear axle speeds.
 */
class PowertrainInterface {
  public:
    /**
     * @brief Construct a new odometer interface.
     *
     * @param port Odometer microcontroller serial port device.
     * @param baudrate Odometer microcontroller serial port baudrate.
     * @param timeout Odometer microcontroller serial connection timeout in
     * milliseconds.
     */
    PowertrainInterface(std::string port, uint32_t baudrate, uint32_t timeout);

    /**
     * @brief Destroy the powertrain interface.
     */
    ~PowertrainInterface();

    /**
     * @brief Open the serial connection to the odometer.
     */
    void Open();

    /**
     * @brief Close the serial connection to the odometer.
     */
    void Close();

    /**
     * @brief Check if the serial connection to the odometer is open.
     *
     * @return true The serial connection to the odometer is open.
     * @return false The serial connection to the odometer is not open.
     */
    bool IsOpen();

    /**
     * @brief Check if new data is available from the odometer.
     *
     * @return true New data is available from the odometer.
     * @return false No new data is available from the odometer.
     */
    bool HasNewData();

    /**
     * @brief Check if the odometer interface is currently receiving data from
     * the odometer.
     *
     * @return true The odometer interface is currently receiving data from the
     * odometer.
     * @return false The odometer interface is not receiving data from the
     * odometer.
     */
    bool IsReceivingData();

    double GetEngineSpeed();

    double GetAxleSpeed();

  private:
    /*** @brief Thread handle for the information parser worker. */
    std::thread worker_;

    /**
     * @brief Worker thread to retrieve readings from the powertrain
     * microcontroller.
     */
    void Worker();

    /** @brief Serial port connection to the powertrain microcontroller. */
    serial::Serial serial_;

    /**
     * @brief Atomic boolean variable marking whether serial port communication
     * has been halted.
     */
    std::atomic<bool> stop_;

    /**
     * @brief Atomic variable defining the current status of the
     * microcontroller.
     */
    std::atomic<Status> status_;

    /**
     * @brief Atomic boolean variable marking whether new data from the
     * microcontroller is available on the serial port.
     */
    std::atomic<bool> has_new_data_;

    /**
     * @brief Timestamp of the last received line on the serial port.
     */
    std::chrono::system_clock::time_point last_message_;

    /**
     * @brief Last read axle speed, expressed in centidegrees per second.
     */
    double axle_speed_ = -1.0;

    /**
     * @brief Last received engine speed, expressed in revolutions per minute.
     *
     */
    double engine_speed_ = -1.0;
};

} // namespace Interfaces
} // namespace OLAV