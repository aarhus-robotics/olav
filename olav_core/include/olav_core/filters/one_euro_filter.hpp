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

#include <cassert>
#include <memory>

#include <olav_core/filters/low_pass_filter.hpp>

namespace OLAV {
namespace ROS {

class OneEuroFilter {
  public:
    OneEuroFilter(const double& sampling_frequency,
                  const double& cutoff_frequency = 1.0,
                  const double& derivative_cutoff_frequency = 1.0,
                  const double& speed_coefficient = 0.0);

    double Filter(const double& value);

    double GetSmoothingConstant(const double& cutoff_frequency);

    void SetSamplingFrequency(const double& sampling_frequency);

    void SetCutoffFrequency(const double& cutoff_frequency);

    void
    SetDerivativeCutoffFrequency(const double& derivative_cutoff_frequency);

    void SetSpeedCoefficient(const double& speed_coefficient);

  private:
    std::shared_ptr<LowPassFilter> signal_lowpass_filter_;

    std::shared_ptr<LowPassFilter> signal_derivative_lowpass_filter_;

    double sampling_frequency_;

    double cutoff_frequency_;

    double derivative_cutoff_frequency_;

    double speed_coefficient_;

    double ComputeDerivative(const double& value, const double& last_value);

    double RunLowPassFilter(std::shared_ptr<LowPassFilter> filter,
                            const double& value,
                            const double& smoothing_constant);
};

} // namespace ROS
} // namespace OLAV
