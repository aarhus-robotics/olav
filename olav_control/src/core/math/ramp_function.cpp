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

#include <olav_control/core/math/ramp_function.hpp>

namespace OLAV {
namespace ROS {

RampFunction::RampFunction(double time) : time_(time) {}

double RampFunction::Tick(const double& elapsed_time) {
    return Smoothstep(start_time_, start_time_ + time_, elapsed_time);
}

void RampFunction::SetStartTime(const double& time) { start_time_ = time; }

void RampFunction::Start(const double& time) {
    start_time_ = time;
    enabled_ = true;
}

void RampFunction::Reset() {
    start_time_ = -std::numeric_limits<double>::infinity();
    enabled_ = false;
}

bool RampFunction::Enabled() { return enabled_; }

float RampFunction::Smoothstep(float edge0, float edge1, float x) {
    // Scale, and clamp x to 0..1 range
    x = boost::algorithm::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);

    return x * x * (3.0f - 2.0f * x);
}

} // namespace ROS
} // namespace OLAV
