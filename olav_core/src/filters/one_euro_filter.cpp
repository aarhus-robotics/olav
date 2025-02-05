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

#include <olav_core/filters/one_euro_filter.hpp>

namespace OLAV {
namespace ROS {

OneEuroFilter::OneEuroFilter(const double& sampling_frequency,
                             const double& cutoff_frequency,
                             const double& derivative_cutoff_frequency,
                             const double& speed_coefficient) {
    // Validate and set the filter parameters.
    SetSamplingFrequency(sampling_frequency);
    SetCutoffFrequency(cutoff_frequency);
    SetDerivativeCutoffFrequency(derivative_cutoff_frequency);
    SetSpeedCoefficient(speed_coefficient);

    // Initialize the lowpass filters.
    signal_lowpass_filter_ =
        std::make_shared<LowPassFilter>(GetSmoothingConstant(cutoff_frequency));
    signal_derivative_lowpass_filter_ = std::make_shared<LowPassFilter>(
        GetSmoothingConstant(derivative_cutoff_frequency));
}

double OneEuroFilter::Filter(const double& value) {
    double signal_derivative =
        ComputeDerivative(value,
                          signal_lowpass_filter_->GetLastFilteredValue());

    double filtered_signal_derivative =
        RunLowPassFilter(signal_derivative_lowpass_filter_,
                         signal_derivative,
                         GetSmoothingConstant(derivative_cutoff_frequency_));

    // use it to update the cutoff frequency
    double updated_cutoff_frequency = cutoff_frequency_ +
        speed_coefficient_ * std::abs(filtered_signal_derivative);

    // Filter the given value
    return signal_lowpass_filter_->FilterWithSmoothing(
        value,
        GetSmoothingConstant(updated_cutoff_frequency));
}

double OneEuroFilter::GetSmoothingConstant(const double& cutoff_frequency) {
    double sampling_period = 1.0 / sampling_frequency_;
    double time_costant = 1.0 / (2.0 * M_PI * cutoff_frequency);

    return 1.0 / (1.0 + time_costant / sampling_period);
}

void OneEuroFilter::SetSamplingFrequency(const double& sampling_frequency) {
    assert(("Sampling frequency must be greater than or equal to zero.",
            sampling_frequency >= 0.0));
    sampling_frequency_ = sampling_frequency;
}

void OneEuroFilter::SetCutoffFrequency(const double& cutoff_frequency) {
    assert(("Cutoff frequency must be greater than or equal to zero.",
            cutoff_frequency >= 0.0));

    cutoff_frequency_ = cutoff_frequency;
}

void OneEuroFilter::SetDerivativeCutoffFrequency(
    const double& derivative_cutoff_frequency) {
    assert(
        ("Derivative cutoff frequency must be greater than or equal to zero.",
         derivative_cutoff_frequency >= 0.0));

    derivative_cutoff_frequency_ = derivative_cutoff_frequency;
}

void OneEuroFilter::SetSpeedCoefficient(const double& speed_coefficient) {
    speed_coefficient_ = speed_coefficient;
}

double OneEuroFilter::ComputeDerivative(const double& value,
                                        const double& last_value) {
    return (value - last_value) * sampling_frequency_;
}

double OneEuroFilter::RunLowPassFilter(std::shared_ptr<LowPassFilter> filter,
                                       const double& value,
                                       const double& smoothing_constant) {
    filter->SetSmoothingConstant(smoothing_constant);
    filter->Filter(value);
    return filter->GetLastFilteredValue();
}

} // namespace ROS
} // namespace OLAV