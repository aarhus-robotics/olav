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

#include <olav_control/drive_by_wire/exceptions.hpp>
#include <olav_control/drive_by_wire/interface.hpp>

namespace OLAV {
namespace ROS {

DriveByWireInterface::DriveByWireInterface(std::string address, int port)
    : address_(address),
      port_(port) {
    // Initialize the atomic connection flag.
    is_connected_ = false;

    // Try to allocate a new Modbus context.
    context_ = modbus_new_tcp(address_.c_str(), port_);
    if(context_ == NULL) {
        throw DriveByWireInterfaceException(
            "Failed to allocate Modbus context.");
    }
}

DriveByWireInterface::~DriveByWireInterface() {
    // Close the Modbus context.
    if(IsConnected()) Close();

    // Free the Modbus context memory.
    modbus_free(context_);
}

void DriveByWireInterface::Open() {
    if(IsConnected()) return;

    if(modbus_connect(context_) == -1) {
        is_connected_ = false;
        throw DriveByWireInterfaceException(
            "Failed to connect to Modbus server.");
    } else {
        is_connected_ = true;
    }
}

void DriveByWireInterface::Close() {
    // Close and flush the Modbus context.
    modbus_close(context_);
    modbus_flush(context_);

    // Mark the PLC interface as disconnected.
    is_connected_ = false;
}

DriveByWireFeedback DriveByWireInterface::Read() {
    int16_t steering_angle, gear, gear_actuator_position,
        brake_actuator_position;
    bool ignition, is_autonomous_mode_on, is_gear_actuator_in_position;

    Read(steering_angle,
         gear,
         gear_actuator_position,
         brake_actuator_position,
         ignition,
         is_autonomous_mode_on,
         is_gear_actuator_in_position);

    auto feedback = DriveByWireFeedback(steering_angle,
                                        gear,
                                        ignition,
                                        is_autonomous_mode_on,
                                        brake_actuator_position,
                                        gear_actuator_position,
                                        is_gear_actuator_in_position);

    return feedback;
}

void DriveByWireInterface::Read(int16_t& steering_angle,
                                int16_t& gear,
                                int16_t& gear_actuator_position,
                                int16_t& brake_actuator_position,
                                bool& ignition,
                                bool& is_autonomous_mode_on,
                                bool& is_gear_actuator_in_position) {
    // Allocate a buffer to perform the read operation in block.
    const int buffer_length = 7;
    int16_t buffer[buffer_length];

    // Read from the registers - note that this operation can be performed in
    // block as all registers are adjacent.
    int read_registers = 0;
    read_registers +=
        modbus_read_registers(context_,
                              RegisterAddress::FEEDBACK_STEERING_ANGLE,
                              buffer_length,
                              (uint16_t*)buffer);

    // Check that the read operation was successful.
    if(read_registers != buffer_length) {
        Close();
        throw DriveByWireInterfaceException(
            "Could not successfully read from the feedback registers.");
    }

    // Cast the buffer contents to the referenced variables.
    steering_angle = buffer[0];
    gear = static_cast<int16_t>(buffer[1]);
    gear_actuator_position = buffer[2];
    brake_actuator_position = buffer[3];
    ignition = static_cast<bool>(buffer[5]);

    // The register at RegisterAddress::FEEDBACK_IGNITION_ENABLED stores two
    // values: the autonomous mode switch position in bit 0 and the gear
    // actuator final position reached flag in bit 1 - here we read them
    // separately through std::bitset on two 8-bit registers.
    std::bitset<16> bitset(buffer[6]);
    is_autonomous_mode_on = bitset.test(0);
    is_gear_actuator_in_position = bitset.test(1);
}

void DriveByWireInterface::ReadSteeringPIDState(bool& use_dynamic_gains,
                                                int16_t& proportional_gain,
                                                int16_t& integral_gain,
                                                int16_t& derivative_gain) {
    const int buffer_length = 4;
    int16_t buffer[buffer_length];

    // Read from the registers - note that this operation can be performed in
    // block as all registers are adjacent.
    int read_registers = 0;
    read_registers += modbus_read_registers(
        context_,
        RegisterAddress::STEERING_PID_DYNAMIC_RECONFIGURE_ON,
        buffer_length,
        (uint16_t*)buffer);

    // Check that the read operation was successful.
    if(read_registers != buffer_length) {
        Close();
        throw DriveByWireInterfaceException(
            "Could not read from the steering PID registers.");
    }

    // Cast the buffer contents to the referenced variables.
    use_dynamic_gains = static_cast<bool>(buffer[0]);
    proportional_gain = buffer[1];
    integral_gain = buffer[2];
    derivative_gain = buffer[3];
}

void DriveByWireInterface::Write(DriveByWireSetpoint& setpoint) {
    Write(setpoint.GetSteering(),
          setpoint.GetBrake(),
          setpoint.GetThrottle(),
          setpoint.GetIgnition(),
          setpoint.GetEmergencyStop(),
          setpoint.GetEngineStarter(),
          setpoint.GetGear());
}

void DriveByWireInterface::Write(const double& steering,
                                 const double& brake,
                                 const double& throttle,
                                 const bool& ignition,
                                 const bool& emergency,
                                 const bool& starter,
                                 const int& gear) {
    // TODO: Move the scaling factors to the node and make the interface operate
    // directly on integer values.

    // Allocate a buffer to perform the write operation in block.
    const int buffer_length = 8;
    int16_t buffer[buffer_length];
    buffer[0] = static_cast<int16_t>(std::floor(steering));
    buffer[1] = static_cast<int16_t>(std::floor(brake));
    buffer[2] = static_cast<int16_t>(std::floor(throttle));
    buffer[3] = static_cast<int16_t>(ignition ? 1 : 0);
    buffer[4] = static_cast<int16_t>(ignition ? 0 : 1);
    buffer[5] = static_cast<int16_t>(emergency ? 1 : 0);
    buffer[6] = static_cast<int16_t>(starter ? 1 : 0);
    buffer[7] = static_cast<int16_t>(gear);

    // Write to the registers - note that this operation can be performed in
    // block as all registers are adjacent.
    int written_registers = 0;
    written_registers +=
        modbus_write_registers(context_,
                               RegisterAddress::SETPOINT_STEERING,
                               buffer_length,
                               (uint16_t*)buffer);

    // Invert the current ticker value and write the updated ticker.
    ticker_ = !ticker_;
    written_registers += modbus_write_register(context_,
                                               RegisterAddress::TICKER,
                                               ticker_ ? 1 : 0);

    // Check that the write operation was successful.
    if(written_registers != (buffer_length + 1)) {
        Close();
        throw DriveByWireInterfaceException(
            "Could not write to the control PLC registers.");
    }
}

void DriveByWireInterface::WriteSteeringPIDState(bool& use_dynamic_gains,
                                                 int16_t& proportional_gain,
                                                 int16_t& integral_gain,
                                                 int16_t& derivative_gain) {
    // Allocate a buffer to perform the write operation in block.
    const int buffer_length = 4;
    int16_t buffer[buffer_length];
    buffer[0] = use_dynamic_gains;
    buffer[1] = proportional_gain;
    buffer[2] = integral_gain;
    buffer[3] = derivative_gain;

    // This operation can be performed in block as all registers are adjacent.
    int written_registers = 0;
    written_registers = modbus_write_registers(
        context_,
        RegisterAddress::STEERING_PID_DYNAMIC_RECONFIGURE_ON,
        buffer_length,
        (uint16_t*)buffer);

    // Check that the write operation was successful.
    if(written_registers != buffer_length) {
        Close();
        throw DriveByWireInterfaceException(
            "Could not write to the steering PID registers.");
    }
}

bool DriveByWireInterface::IsConnected() { return is_connected_; }

} // namespace ROS
} // namespace OLAV