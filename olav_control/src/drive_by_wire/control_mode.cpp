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

#include <olav_control/drive_by_wire/control_mode.hpp>

namespace OLAV {
namespace ROS {

ControlMode::ControlMode()
    : mode_identifier_(static_cast<ControlModeIdentifier>(-1)),
      mode_name_("standby"),
      authority_identifier_(static_cast<ControlAuthorityIdentifier>(-1)),
      authority_name_("system") {}

ControlMode::ControlMode(const ControlModeIdentifier& mode_identifier,
                         const ControlAuthorityIdentifier& authority_identifier)
    : mode_identifier_(mode_identifier),
      mode_name_(FromModeIdentifier(mode_identifier)),
      authority_identifier_(authority_identifier),
      authority_name_(FromAuthorityIdentifier(authority_identifier)) {}

ControlMode::ControlMode(const std::string& mode_name,
                         const std::string& authority_name)
    : mode_identifier_(FromModeName(mode_name)),
      mode_name_(mode_name),
      authority_identifier_(FromAuthorityName(authority_name)),
      authority_name_(authority_name) {}

std::string
ControlMode::FromModeIdentifier(const ControlModeIdentifier& identifier) {
    switch(identifier) {
    case ControlModeIdentifier::STANDBY: return "standby"; break;
    case ControlModeIdentifier::DRIVE_ACKERMANN:
        return "drive_ackermann";
        break;
    case ControlModeIdentifier::DRIVE_TBS: return "drive_tbs"; break;
    }
    // TODO: Add a default case that throws an exception.
}

ControlModeIdentifier ControlMode::FromModeName(const std::string& name) {
    if(name == "standby") {
        return ControlModeIdentifier::STANDBY;
    } else if(name == "drive_ackermann") {
        return ControlModeIdentifier::DRIVE_ACKERMANN;
    } else if(name == "drive_tbs") {
        return ControlModeIdentifier::DRIVE_TBS;
    } else {
        throw std::invalid_argument("Invalid control mode!");
    }
}

std::string ControlMode::FromAuthorityIdentifier(
    const ControlAuthorityIdentifier& identifier) {
    switch(identifier) {
    case ControlAuthorityIdentifier::SYSTEM: return "system"; break;
    case ControlAuthorityIdentifier::TERMINAL: return "terminal"; break;
    case ControlAuthorityIdentifier::GAMEPAD: return "gamepad"; break;
    case ControlAuthorityIdentifier::AUTONOMY: return "autonomy"; break;
    }
    // TODO: Add a default case that throws an exception.
}

ControlAuthorityIdentifier
ControlMode::FromAuthorityName(const std::string& name) {
    if(name == "system") {
        return ControlAuthorityIdentifier::SYSTEM;
    } else if(name == "terminal") {
        return ControlAuthorityIdentifier::TERMINAL;
    } else if(name == "gamepad") {
        return ControlAuthorityIdentifier::GAMEPAD;
    } else if(name == "autonomy") {
        return ControlAuthorityIdentifier::AUTONOMY;
    } else {
        throw std::invalid_argument("Invalid control mode!");
    }
}

bool ControlMode::operator==(const ControlMode& mode) {
    return (mode_identifier_ == mode.GetModeIdentifier() &&
            mode_name_ == mode.GetModeName()) &&
        (authority_identifier_ == mode.GetAuthorityIdentifier() &&
         authority_name_ == mode.GetAuthorityName());
}

const std::string& ControlMode::GetModeName() const { return mode_name_; }

const ControlModeIdentifier& ControlMode::GetModeIdentifier() const {
    return mode_identifier_;
}

const ControlAuthorityIdentifier& ControlMode::GetAuthorityIdentifier() const {
    return authority_identifier_;
}

const std::string& ControlMode::GetAuthorityName() const {
    return authority_name_;
}

} // namespace ROS
} // namespace OLAV