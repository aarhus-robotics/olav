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

#include <olav_control/drive_by_wire/control_override.hpp>

namespace OLAV {
namespace ROS {

ControlOverride::ControlOverride()
    : override_identifier_(static_cast<ControlOverrideIdentifier>(-1)),
      override_name_("standby") {}

ControlOverride::ControlOverride(const ControlOverrideIdentifier& override_identifier)
    : override_identifier_(override_identifier),
      override_name_(FromOverrideIdentifier(override_identifier)) {}

ControlOverride::ControlOverride(const std::string& override_name)
    : override_identifier_(FromOverrideName(override_name)),
      override_name_(override_name) {}

std::string
ControlOverride::FromOverrideIdentifier(const ControlOverrideIdentifier& identifier) {
    switch(identifier) {
    case ControlOverrideIdentifier::DISABLED: return "disabled"; break;
    case ControlOverrideIdentifier::OVERRIDE_THROTTLE_BRAKE:
        return "override_throttle_brake";
        break;
    case ControlOverrideIdentifier::OVERRIDE_STEERING: return "override_steering"; break;
    }
    // TODO: Add a default case that throws an exception.
}

ControlOverrideIdentifier ControlOverride::FromOverrideName(const std::string& name) {
    if(name == "disabled") {
        return ControlOverrideIdentifier::DISABLED;
    } else if(name == "override_throttle_brake") {
        return ControlOverrideIdentifier::OVERRIDE_THROTTLE_BRAKE;
    } else if(name == "override_steering") {
        return ControlOverrideIdentifier::OVERRIDE_STEERING;
    } else {
        throw std::invalid_argument("Invalid control mode!");
    }
}

bool ControlOverride::operator==(const ControlOverride& mode) {
    return (override_identifier_ == mode.GetOverrideIdentifier() &&
            override_name_ == mode.GetOverrideName()) &&
        (override_identifier_ == mode.GetOverrideIdentifier() &&
         override_name_ == mode.GetOverrideName());
}

const std::string& ControlOverride::GetOverrideName() const { return override_name_; }

const ControlOverrideIdentifier& ControlOverride::GetOverrideIdentifier() const {
    return override_identifier_;
}

} // namespace ROS
} // namespace OLAV