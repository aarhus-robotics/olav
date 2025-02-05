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

#include <olav_control/multiplexer/control_mode.hpp>

namespace OLAV {
namespace ROS {

ControlMode::ControlMode()
    : id_(static_cast<ControlModeIdentifier>(-1)),
      name_("null") {}

ControlMode::ControlMode(const ControlModeIdentifier& id)
    : id_(id),
      name_(FromId(id)) {}

ControlMode::ControlMode(const std::string& name)
    : id_(FromName(name)),
      name_(name) {}

std::string ControlMode::FromId(const ControlModeIdentifier& id) {
    switch(id) {
    case ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED:
        return "disabled";
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD:
        return "gamepad";
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS:
        return "autonomous";
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER:
        return "generic_throttle_brake_steer";
        break;
    case ControlModeIdentifier::MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE:
        return "generic_speed_steering_angle";
        break;
    }
}

ControlModeIdentifier ControlMode::FromName(const std::string& name) {
    if(name == "disabled") {
        return ControlModeIdentifier::MULTIPLEXER_MODE_DISABLED;
    } else if(name == "gamepad") {
        return ControlModeIdentifier::MULTIPLEXER_MODE_GAMEPAD;
    } else if(name == "autonomous") {
        return ControlModeIdentifier::MULTIPLEXER_MODE_AUTONOMOUS;
    } else if(name == "generic_throttle_brake_steer") {
        return ControlModeIdentifier::
            MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER;
    } else if(name == "generic_speed_steering_angle") {
        return ControlModeIdentifier::
            MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE;
    } else {
        throw std::invalid_argument("Invalid control mode!");
    }
}

bool ControlMode::operator==(const ControlMode& mode) {
    return id_ == mode.GetId() && name_ == mode.GetName();
}

ControlMode ControlMode::GetNext() {
    if(static_cast<int>(id_) < 3) {
        return ControlMode(
            static_cast<ControlModeIdentifier>(static_cast<int>(id_) + 1));
    } else {
        return ControlMode(static_cast<ControlModeIdentifier>(-1));
    }
}

const std::string& ControlMode::GetName() const { return name_; }

const ControlModeIdentifier& ControlMode::GetId() const { return id_; }

} // namespace ROS
} // namespace OLAV