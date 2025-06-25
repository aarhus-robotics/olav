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

enum class ControlOverrideIdentifier {
    DISABLED = -1,
    OVERRIDE_THROTTLE_BRAKE = 0,
    OVERRIDE_STEERING = 1
};

class ControlOverride {
  public:
    ControlOverride();

    ControlOverride(const ControlOverrideIdentifier& override_identifier);

    ControlOverride(const std::string& override_name);

    static std::string
    FromOverrideIdentifier(const ControlOverrideIdentifier& identifier);

    static ControlOverrideIdentifier FromOverrideName(const std::string& name);

    bool operator==(const ControlOverride& mode);

    const ControlOverrideIdentifier& GetOverrideIdentifier() const;

    const std::string& GetOverrideName() const;

  private:
    ControlOverrideIdentifier override_identifier_;

    std::string override_name_;
};

} // namespace ROS
} // namespace OLAV