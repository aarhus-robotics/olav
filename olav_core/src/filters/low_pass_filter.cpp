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

#include <olav_core/filters/low_pass_filter.hpp>

namespace OLAV {
namespace ROS {

void LowPassFilter::SetSmoothingConstant(const double& smoothing_constant) {
    assert(("Smoothing constant must lie in the interval (0.0, 1.0]).",
            smoothing_constant > 0.0 || smoothing_constant <= 1.0));

    smoothing_constant_ = smoothing_constant;
}

LowPassFilter::LowPassFilter(const double& smoothing_constant) {
    SetSmoothingConstant(smoothing_constant);
}

double LowPassFilter::Filter(const double& value) {
    double result;
    if(is_initialized_)
        result = smoothing_constant_ * value +
            (1.0 - smoothing_constant_) * filtered_value_;
    else {
        result = value;
        is_initialized_ = true;
    }

    raw_value_ = value;
    filtered_value_ = result;

    return result;
}

double LowPassFilter::FilterWithSmoothing(const double& value,
                                          const double& smoothing_constant) {
    SetSmoothingConstant(smoothing_constant);
    return Filter(value);
}

const bool& LowPassFilter::IsInitialized() const { return is_initialized_; }

const double& LowPassFilter::GetLastRawValue() const { return raw_value_; }

const double& LowPassFilter::GetLastFilteredValue() const {
    return filtered_value_;
}

} // namespace ROS
} // namespace OLAV