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

#include <olav_core/filters/outlier_resistant_filter.hpp>

namespace OLAV {
namespace ROS {

OutlierResistantFilter::OutlierResistantFilter(const double& rate)
    : sampling_period_(1.0 / rate),
      adaptive_filter_(OneEuroFilter(rate, rate / 2.0, rate / 4.0, 0.5)) {}

void OutlierResistantFilter::SetMinimumValue(const double& minimum_value) {
    minimum_value_ = minimum_value;
}

void OutlierResistantFilter::SetMaximumValue(const double& maximum_value) {
    maximum_value_ = maximum_value;
}

void OutlierResistantFilter::SetMaximumChange(const double& maximum_change) {
    maximum_change_ = maximum_change;
}

void OutlierResistantFilter::SetScalingFactor(const double& scaling_factor) {
    scaling_factor_ = scaling_factor;
}

void OutlierResistantFilter::SetCutoffFrequency(
    const double& cutoff_frequency) {
    adaptive_filter_.SetCutoffFrequency(cutoff_frequency);
}

void OutlierResistantFilter::SetDerivativeCutoffFrequency(
    const double& derivative_cutoff_frequency) {
    adaptive_filter_.SetDerivativeCutoffFrequency(derivative_cutoff_frequency);
}

void OutlierResistantFilter::SetSpeedCoefficient(
    const double& speed_coefficient) {
    adaptive_filter_.SetSpeedCoefficient(speed_coefficient);
}

void OutlierResistantFilter::Filter(const double& value) {
    // Clamp the new value to the acceptable bounds.
    double filtered_value =
        boost::algorithm::clamp(value, minimum_value_, maximum_value_);

    // Compute the rate of change between the last filtered measurement and the
    // new measurement.
    double rate_of_change =
        (filtered_value_ - filtered_value) / sampling_period_;
    filtered_value = (rate_of_change > maximum_change_) ? filtered_value +
            scaling_factor_ * (maximum_change_ * sampling_period_)
                                                        : filtered_value;

    filtered_value_ = adaptive_filter_.Filter(filtered_value);
}

const double& OutlierResistantFilter::GetFiltered() const {
    return filtered_value_;
}

} // namespace ROS
} // namespace OLAV