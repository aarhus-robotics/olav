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

#include <olav_control/drive_by_wire/feedback.hpp>

namespace OLAV {
namespace ROS {

DriveByWireFeedback::DriveByWireFeedback() {}

DriveByWireFeedback::DriveByWireFeedback(
    const int16_t& steering_actuator_position,
    const int16_t& selected_gear,
    const bool& is_ignition_on,
    const bool& is_autonomous_mode_on,
    const int16_t& brake_actuator_position,
    const int16_t& gear_actuator_position,
    const bool& is_gear_actuator_in_position)
    : steering_actuator_position_(steering_actuator_position),
      selected_gear_(selected_gear),
      is_ignition_on_(is_ignition_on),
      is_autonomous_mode_on_(is_autonomous_mode_on),
      brake_actuator_position_(brake_actuator_position),
      gear_actuator_position_(gear_actuator_position),
      is_gear_actuator_in_position_(is_gear_actuator_in_position) {}

void DriveByWireFeedback::IsIgnitionOn(bool is_ignition_on) {
    is_ignition_on_ = is_ignition_on;
}

const bool& DriveByWireFeedback::IsIgnitionOn() { return is_ignition_on_; }

void DriveByWireFeedback::IsAutonomousModeOn(bool is_autonomous_mode_on) {
    is_autonomous_mode_on_ = is_autonomous_mode_on;
}

const bool& DriveByWireFeedback::IsAutonomousModeOn() {
    return is_autonomous_mode_on_;
}

void DriveByWireFeedback::SetSteeringActuatorPosition(
    int16_t steering_actuator_position) {
    steering_actuator_position_ = steering_actuator_position;
}

const int16_t& DriveByWireFeedback::GetSteeringActuatorPosition() {
    return steering_actuator_position_;
}

double DriveByWireFeedback::GetSteeringActuatorPositionInRadians() {
    return (steering_actuator_position_ - steering_actuator_voltage_offset_) /
        steering_actuator_voltage_gain_;
}

double DriveByWireFeedback::GetSteeringActuatorPositionInDegrees() {
    return GetSteeringActuatorPositionInRadians() * (180 / M_PI);
}

void DriveByWireFeedback::SetBrakeActuatorPosition(
    int16_t brake_actuator_position) {
    brake_actuator_position_ = brake_actuator_position;
}

const int16_t& DriveByWireFeedback::GetBrakeActuatorPosition() {
    return brake_actuator_position_;
}

void DriveByWireFeedback::SetSelectedGear(int16_t selected_gear) {
    selected_gear_ = selected_gear;
}

const int16_t& DriveByWireFeedback::GetSelectedGear() { return selected_gear_; }

void DriveByWireFeedback::SetGearActuatorPosition(
    int16_t gear_actuator_position) {
    gear_actuator_position_ = gear_actuator_position;
}

const int16_t& DriveByWireFeedback::GetGearActuatorPosition() {
    return gear_actuator_position_;
}

void DriveByWireFeedback::IsGearActuatorInPosition(
    bool is_gear_actuator_in_position) {
    is_gear_actuator_in_position_ = is_gear_actuator_in_position;
}

const bool& DriveByWireFeedback::IsGearActuatorInPosition() {
    return is_gear_actuator_in_position_;
}

} // namespace ROS
} // namespace OLAV