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

#include <boost/algorithm/clamp.hpp>

#include <olav_core/filters/one_euro_filter.hpp>

namespace OLAV {
namespace ROS {

class OutlierResistantFilter {
  public:
    OutlierResistantFilter(const double& rate);

    void SetMinimumValue(const double& minimum_value);

    void SetMaximumValue(const double& maximum_value);

    void SetMaximumChange(const double& maximum_change);

    void SetScalingFactor(const double& scaling_factor);

    void SetCutoffFrequency(const double& cutoff_frequency);

    void
    SetDerivativeCutoffFrequency(const double& derivative_cutoff_frequency);

    void SetSpeedCoefficient(const double& speed_coefficient);

    void Filter(const double& value);

    const double& GetFiltered() const;

  private:
    double sampling_period_;

    double minimum_value_ = -std::numeric_limits<double>::infinity();

    double maximum_value_ = std::numeric_limits<double>::infinity();

    double maximum_change_ = std::numeric_limits<double>::infinity();

    double scaling_factor_ = 0.1;

    double filtered_value_ = 0.0;

    OneEuroFilter adaptive_filter_;
};

} // namespace ROS
} // namespace OLAV