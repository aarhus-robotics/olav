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
#include <bitset>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <modbus/modbus.h>

#include <olav_control/drive_by_wire/exceptions.hpp>
#include <olav_control/drive_by_wire/feedback.hpp>
#include <olav_control/drive_by_wire/setpoint.hpp>

namespace OLAV {
namespace ROS {

using OLAV::Exceptions::DriveByWireInterfaceException;

class DriveByWireInterface {
  public:
    double BRAKE_SCALE_FACTOR = 100.0;
    double THROTTLE_SCALE_FACTOR = 100.0;
    double STEERING_SCALE_FACTOR = 1500.0;

    enum GearPosition { PARK = 1, REVERSE = 2, NEUTRAL = 3, LOW = 4, HIGH = 5 };

    enum RegisterAddress {
        FEEDBACK_STEERING_ANGLE = 0x100,
        FEEDBACK_SELECTED_GEAR = 0x101,
        FEEDBACK_GEAR_ACTUATOR_POSITION = 0x102,
        FEEDBACK_BRAKE_ACTUATOR_POSITION = 0x103,
        FEEDBACK_GEAR_SELECTOR_POSITION = 0x104,
        FEEDBACK_IGNITION_ENABLED = 0x105,
        SETPOINT_STEERING = 0x200 + 0x100,
        SETPOINT_BRAKE = 0x200 + 0x101,
        SETPOINT_THROTTLE = 0x200 + 0x102,
        SETPOINT_IGNITION_ON = 0x200 + 0x103,
        SETPOINT_IGNITION_OFF = 0x200 + 0x104,
        SETPOINT_EMERGENCY_STOP = 0x200 + 0x105,
        SETPOINT_ENGINE_START = 0x200 + 0x106,
        SETPOINT_GEAR = 0x200 + 0x107,
        STEERING_PID_DYNAMIC_RECONFIGURE_ON = 0x200 + 0x108,
        STEERING_PID_PROPORTIONAL_GAIN = 0x200 + 0x109,
        STEERING_PID_INTEGRAL_GAIN = 0x200 + 0x10A,
        STEERING_PID_DERIVATIVE_GAIN = 0x200 + 0x10B,
        TICKER = 0x200 + 0x10C,
    };

    /**
     * @brief Construct a new drive-by-wire programmable logic controller
     * interface.
     *
     * @param address Address of the drive-by-wire programmable logic controller
     * Modbus TCP server.
     * @param port Port of the drive-by-wire programmable logic controller
     * Modbus TCP server.
     * @param measurements Shared pointer to the vehicle measurements.
     */
    DriveByWireInterface(std::string address, int port);

    /**
     * @brief Destroy the drive-by-wire programmable logic controller interface.
     */
    ~DriveByWireInterface();

    /**
     * @brief Open the connection to the drive-by-wire programmable logic
     * controller Modbus TCP server.
     */
    void Open();

    /**
     * @brief Close the connection to the drive-by-wire programmable logic
     * controller Modbus TCP server.
     */
    void Close();

    /**
     * @brief Disable the proportional-integral-derivative steering controller
     * in the drive-by-wire programmable logic controller settings.
     */
    void DisableSteeringPID();

    DriveByWireFeedback Read();

    /**
     * @brief Retrieve the current vehicle state from the drive-by-wire
     * programmable logic controller.
     */
    void Read(int16_t& steering_angle,
              int16_t& gear,
              int16_t& gear_actuator_position,
              int16_t& brake_actuator_position,
              bool& ignition,
              bool& is_autonomous_mode_on,
              bool& is_gear_actuator_in_position);

    void ReadSteeringPIDState(bool& use_dynamic_gains,
                              int16_t& proportional_gain,
                              int16_t& integral_gain,
                              int16_t& derivative_gain);

    void Write(DriveByWireSetpoint& setpoint);

    /**
     * @brief Write a control command to the drive-by-wire programmable logic
     * controller.
     *
     * @param steering Desired steering angle in radians.
     * @param brake Desired brake effort.
     * @param throttle Desired throttle effort.
     * @param ignition Desired ignition state.
     * @param emergency Desired emergency status.
     * @param starter Desired starter status
     * @param gear Desired gear selection.
     */
    void Write(const double& steering,
               const double& brake,
               const double& throttle,
               const bool& ignition,
               const bool& emergency,
               const bool& starter,
               const int& gear);

    void WriteSteeringPIDState(bool& use_dynamic_gains,
                               int16_t& proportional_gain,
                               int16_t& integral_gain,
                               int16_t& derivative_gain);

    template <typename T>
    T ReadRegister(const int& address) {
        int read_registers = 0;
        uint16_t buffer[1];
        read_registers +=
            modbus_read_registers(context_, address, 1, (uint16_t*)buffer);

        if(read_registers != 1) {
            Close();
            throw DriveByWireInterfaceException(
                "Could not successfully write PLC registers.");
        }

        return static_cast<T>(buffer[0]);
    }

    template <typename T>
    void WriteRegister(const int& address, const T& value) {
        auto written_registers =
            modbus_write_register(context_,
                                  address,
                                  static_cast<uint16_t>(value));

        if(written_registers != 1) {
            Close();
            throw DriveByWireInterfaceException(
                "Could not successfully write PLC registers.");
        }
    }

    /**
     * @brief Whether an active connection to the drive-by-wire programmable
     * logic controller is present or not.
     *
     * @return true There is an active connection to the drive-by-wire
     * programmable logic controller.
     * @return false There is no active connection to the drive-by-wire
     * programmable logic controller.
     */
    bool IsConnected();

  private:
    /** @brief Modbus context. */
    modbus_t* context_;

    /** @brief Modbus server TCP address. */
    std::string address_;

    /** @brief Modbus server port. */
    int port_;

    /** @brief Whether or not a connection to the PLC Modbus server has been
     * successfully established. */
    std::atomic<bool> is_connected_;

    /** @brief Helper ticker used to write alternating register entries. */
    bool ticker_ = true;
};

} // namespace ROS
} // namespace OLAV