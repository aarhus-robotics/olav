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

#include <olav_control/core/controllers/pid.hpp>

namespace OLAV {
namespace ROS {

PIDController::PIDController() {}

void PIDController::SetSetpoint(const double& setpoint) {
    setpoint_ = setpoint;
}

const double& PIDController::GetSetpoint() const { return setpoint_; }

void PIDController::UseSetpointRamping(const bool& use_setpoint_ramping) {
    use_setpoint_ramping_ = use_setpoint_ramping;
}

const bool& PIDController::UseSetpointRamping() const {
    return use_setpoint_ramping_;
}

void PIDController::SetMaximumSetpointChange(
    const double& maximum_setpoint_change) {
    maximum_setpoint_change_ = maximum_setpoint_change;
}

const double& PIDController::GetMaximumSetpointChange() const {
    return maximum_setpoint_change_;
}

void PIDController::SetFeedback(const double& feedback) {
    feedback_ = feedback;
}

const double& PIDController::GetFeedback() const { return feedback_; }

void PIDController::SetFeedforwardGain(const double& feedforward_gain) {
    assert(("Feedforward gain must be greater than or equal to zero.",
            feedforward_gain >= 0.0));
    feedforward_gain_ = feedforward_gain;
}

const double& PIDController::GetFeedforwardGain() const {
    return feedforward_gain_;
}

void PIDController::SetUnscaledFeedforwardTerm(
    const double& unscaled_feedforward_term) {
    unscaled_feedforward_term_ = unscaled_feedforward_term;
}

const double& PIDController::GetFeedforwardTerm() const {
    return feedforward_term_;
}

void PIDController::SetFeedforwardOffset(const double& feedforward_offset) {
    feedforward_offset_ = feedforward_offset;
}

void PIDController::SetProportionalGain(const double& proportional_gain) {
    assert(("Proportional gain must be greater than or equal to zero.",
            proportional_gain >= 0.0));
    proportional_gain_ = proportional_gain;
}

const double& PIDController::GetProportionalGain() const {
    return proportional_gain_;
}

const double& PIDController::GetProportionalTerm() const {
    return proportional_term_;
}

void PIDController::SetIntegralGain(double integral_gain) {
    assert(("Integral gain must be greater than or equal to zero.",
            integral_gain >= 0.0));

    if(integral_gain_ > 0.0) {
        // Rescale the cumulated error.
        cumulative_error_ *= integral_gain_ / integral_gain;

        // Rescale the integral term limit.
        if(use_integral_term_limiter_) {
            maximum_cumulative_error_ = maximum_integral_term_ / integral_gain_;
        }
    }

    integral_gain_ = integral_gain;
}

const double& PIDController::GetIntegralGain() const { return integral_gain_; }

const double& PIDController::GetIntegralTerm() const { return integral_term_; }

void PIDController::UseIntegralTermLimiter(
    const bool& use_integral_term_limiter) {
    use_integral_term_limiter_ = use_integral_term_limiter;
}

const bool& PIDController::UseIntegralTermLimiter() const {
    return use_integral_term_limiter_;
}

void PIDController::SetMaximumIntegralTerm(
    const double& maximum_integral_term) {
    assert(("Maximum integral term must be greater than or equal to zero.",
            maximum_integral_term >= 0.0));

    maximum_integral_term_ = maximum_integral_term;

    // Update the maximum cumulative error accordingly.
    if(integral_gain_ > 0.0) {
        maximum_cumulative_error_ = maximum_integral_term_ / integral_gain_;
    }
}

const double& PIDController::GetMaximumIntegralTerm() const {
    return maximum_integral_term_;
}

void PIDController::SetDerivativeGain(const double& derivative_gain) {
    assert(("Derivative gain must be greater than or equal to zero.",
            derivative_gain >= 0.0));

    derivative_gain_ = derivative_gain;
}

const double& PIDController::GetDerivativeGain() const {
    return derivative_gain_;
}

const double& PIDController::GetDerivativeTerm() const {
    return derivative_term_;
}

const double& PIDController::GetOutput() const { return output_; }

void PIDController::UseOutputLimiter(const bool& use_output_limiter) {
    use_output_limiter_ = use_output_limiter;
}

const bool& PIDController::UseOutputLimiter() const {
    return use_output_limiter_;
}

void PIDController::SetMinimumOutput(const double& minimum_output) {
    assert(("Minimum output must be lower than maximum output.",
            minimum_output < maximum_output_));

    minimum_output_ = minimum_output;
}

const double& PIDController::GetMinimumOutput() const {
    return minimum_output_;
}

void PIDController::SetMaximumOutput(const double& maximum_output) {
    assert(("Maximum output must be higher than minimum output.",
            maximum_output > minimum_output_));

    maximum_output_ = maximum_output;
}

const double& PIDController::GetMaximumOutput() const {
    return maximum_output_;
}

void PIDController::UseOutputChangeLimiter(
    const bool& use_output_change_limiter) {
    use_output_change_limiter_ = use_output_change_limiter;
}

const bool& PIDController::UseOutputChangeLimiter() const {
    return use_output_change_limiter_;
}

void PIDController::SetMaximumOutputChange(
    const double& maximum_output_change) {
    assert(("Maximum output change must be larger than zero.",
            maximum_output_change > 0.0));

    maximum_output_change_ = maximum_output_change;
}

const double& PIDController::GetMaximumOutputChange() const {
    return maximum_output_change_;
}

void PIDController::UseOutputFilter(const bool& use_output_filter) {
    use_output_filter_ = use_output_filter;
}

const bool& PIDController::UseOutputFilter() const {
    return use_output_filter_;
}

void PIDController::SetOutputFilterWeight(const double& output_filter_weight) {
    assert(("Output filter change must be larger than zero.",
            output_filter_weight > 0.0));

    output_filter_weight_ = output_filter_weight;
}

const double& PIDController::GetOutputFilterWeight() const {
    return output_filter_weight_;
}

void PIDController::UseDeadbandFilter(const bool& use_deadband_filter) {
    use_deadband_filter_ = use_deadband_filter;
}

const bool& PIDController::IsDeadbanding() const { return is_deadbanding_; }

void PIDController::SetDeadbandCenter(const double& deadband_center) {
    deadband_center_ = deadband_center;
}

void PIDController::SetDeadbandLowerThreshold(
    const double& deadband_lower_threshold) {
    deadband_lower_threshold_ = deadband_lower_threshold;
}

void PIDController::SetDeadbandUpperThreshold(
    const double& deadband_upper_threshold) {
    deadband_upper_threshold_ = deadband_upper_threshold;
}

const bool& PIDController::UseDeadbandFilter() const {
    return use_deadband_filter_;
}

void PIDController::UseErrorThreshold(const bool& use_error_threshold) {
    use_error_threshold_ = use_error_threshold;
}

const bool& PIDController::UseErrorThreshold() const {
    return use_error_threshold_;
}

void PIDController::SetErrorThreshold(const double& error_threshold) {
    error_threshold_ = error_threshold;
}

const double& PIDController::GetErrorThreshold() const {
    return error_threshold_;
}

void PIDController::Tick() {
    // Ramp the setpoint used for calculations if user has opted to do so
    double ramped_setpoint = (use_setpoint_ramping_)
        ? boost::algorithm::clamp(setpoint_,
                                  last_setpoint_ - maximum_setpoint_change_,
                                  last_setpoint_ + maximum_setpoint_change_)
        : setpoint_;
    last_setpoint_ = ramped_setpoint;

    // Check if the feedback is sufficiently close to the setpoint, mock the
    // feedback if we are within tolerance to avoid oscillations around the
    // equilibrium point.
    if(boost::math::sign(feedback_) == boost::math::sign(ramped_setpoint)) {
        feedback_ = (std::abs(ramped_setpoint - feedback_) < error_threshold_)
            ? ramped_setpoint
            : feedback_;
    }

    // Store the last tracking error and compute the new one based on the
    // plant feedback.
    double error = ramped_setpoint - feedback_;
    last_feedback_ = feedback_;

    // Compute the feedforward term. Note how the output change sign is
    // computed in order to select the correct feedforward offset.
    double feedforward_offset = 0.0;
    auto output_direction = boost::math::sign(ramped_setpoint - feedback_);
    if(std::abs(last_output_) < feedforward_offset_) {
        if(output_direction > 0) {
            feedforward_offset = feedforward_offset_;
        } else if(output_direction < 0) {
            feedforward_offset = -feedforward_offset_;
        }
    }

    /*
    feedforward_term_ =
        (use_transient_feedforward_ && std::abs(error) > error_threshold_)
        ? feedforward_offset + feedforward_gain_ * unscaled_feedforward_term_
        : 0.0;
    */

    // Compute the proportional term.
    proportional_term_ = proportional_gain_ * error;

    // If this is the first controller tick, we assume the last measurement
    // was the same as the current one and the last controller output was
    // the combination of the time-invariant terms.
    if(is_first_tick_) {
        last_output_ = proportional_term_ + feedforward_term_;
        is_first_tick_ = false;
    }

    // Compute the derivative term and update the last processed feedback
    // for the next tick.
    derivative_term_ = derivative_gain_ * (error - last_error_);

    // Also limit the integral term, regardless of whether the
    // controller output is within bounds.
    cumulative_error_ = (use_integral_term_limiter_)
        ? boost::algorithm::clamp(cumulative_error_ + error,
                                  -maximum_cumulative_error_,
                                  maximum_cumulative_error_)
        : cumulative_error_ + error;

    // Compute the integral term.
    integral_term_ = (use_integral_term_limiter_)
        ? boost::algorithm::clamp(integral_gain_ * cumulative_error_,
                                  -maximum_integral_term_,
                                  maximum_integral_term_)
        : integral_gain_ * cumulative_error_;

    // Compute the controller output.
    output_ = proportional_term_ + integral_term_ + derivative_term_;

    // Enforce the bounds on the controller output.
    if(use_output_change_limiter_) {
        output_ =
            boost::algorithm::clamp(output_,
                                    last_output_ - maximum_output_change_,
                                    last_output_ + maximum_output_change_);
    }

    if(use_output_filter_) {
        output_ = last_output_ * output_filter_weight_ +
            output_ * (1.0 - output_filter_weight_);
    }

    // Add the feedforward term - this should be unaffected by the filters.
    output_ += feedforward_offset;
    output_ += feedforward_term_;

    // FIXME: The deadband filter does not apply the correct direction to the
    //        output control.
    if(use_deadband_filter_) {
        if(output_ >= deadband_lower_threshold_ && output_ < deadband_center_) {
            output_ -= std::abs(deadband_lower_threshold_);
            is_deadbanding_ = true;
        } else if(output_ <= deadband_upper_threshold_ &&
                  output_ > deadband_center_) {
            output_ += std::abs(deadband_upper_threshold_);
            is_deadbanding_ = true;
        } else
            is_deadbanding_ = false;
    }

    // Note that the output limiter should always override any previous
    // manipulation of the controller output to ensure the output is within
    // the allowed bounds.
    if(use_output_limiter_) {
        output_ =
            boost::algorithm::clamp(output_, minimum_output_, maximum_output_);
    }

    // Update output.
    last_output_ = output_;
}

void PIDController::Reset() {
    setpoint_ = 0.0;
    last_setpoint_ = 0.0;
    feedback_ = 0.0;
    last_feedback_ = 0.0;
    output_ = 0.0;
    last_output_ = 0.0;

    feedforward_term_ = 0.0;
    proportional_term_ = 0.0;
    integral_term_ = 0.0;
    derivative_term_ = 0.0;

    last_error_ = 0.0;
    cumulative_error_ = 0.0;

    is_first_tick_ = true;
    is_deadbanding_ = false;
}

} // namespace ROS
} // namespace OLAV