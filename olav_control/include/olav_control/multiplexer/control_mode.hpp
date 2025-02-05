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

#include <stdexcept>
#include <string>

namespace OLAV {
namespace ROS {

enum class ControlModeIdentifier {
    MULTIPLEXER_MODE_DISABLED = -1,
    MULTIPLEXER_MODE_GAMEPAD = 0,
    MULTIPLEXER_MODE_AUTONOMOUS = 1,
    MULTIPLEXER_MODE_GENERIC_THROTTLE_BRAKE_STEER = 2,
    MULTIPLEXER_MODE_GENERIC_SPEED_STEERING_ANGLE = 3
};

class ControlMode {
  public:
    ControlMode();

    ControlMode(const ControlModeIdentifier& id);

    ControlMode(const std::string& name);

    static std::string FromId(const ControlModeIdentifier& id);

    static ControlModeIdentifier FromName(const std::string& name);

    bool operator==(const ControlMode& mode);

    ControlMode GetNext();

    const std::string& GetName() const;

    const ControlModeIdentifier& GetId() const;

  private:
    ControlModeIdentifier id_;

    std::string name_;
};

} // namespace ROS
} // namespace OLAV