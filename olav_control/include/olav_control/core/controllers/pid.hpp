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
#include <limits>

#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace OLAV {
namespace ROS {

/**
 * @brief A Proportional-Integral-Derivative (PID) controller.
 */
class PIDController {
  public:
    /**
     * @brief Construct a new PID controller.
     */
    PIDController();

    void SetSetpoint(double setpoint);

    double GetSetpoint();

    void UseSetpointRamping(bool use_setpoint_ramping);

    bool UseSetpointRamping();

    void SetMaximumSetpointChange(double maximum_setpoint_change);

    double GetMaximumSetpointChange();

    /**
     * @brief Set the plant feedback.
     *
     * @param feedback Plant feedback.
     */
    void SetFeedback(double feedback);

    double GetFeedback();

    void SetFeedforwardGain(double feedforward_gain);

    double GetFeedforwardGain();

    void SetUnscaledFeedforwardTerm(double unscaled_feedforward_term);

    double GetFeedforwardTerm();

    void SetFeedforwardOffset(double feedforward_offset);

    double GetFeedforwardOffset();

    /**
     * @brief Set the proportional gain.
     *
     * @param proportional_gain Proportional gain. Must be zero or a positive
     * real number.
     */
    void SetProportionalGain(double proportional_gain);

    double GetProportionalGain();

    double GetProportionalTerm();

    void SetMaximumError(double maximum_error);

    /**
     * @brief Set the integral gain
     *
     * @param integral_gain Integral gain. Must be zero or a positive real
     * number.
     */
    void SetIntegralGain(double integral_gain);

    double GetIntegralGain();

    double GetIntegralTerm();

    /**
     * @brief Set the controller to limit the integral term.
     *
     * @param use_integral_term_limiter Integral term limiter switch. True to
     * enable the integral term limiter, false to disable the integral term
     * limiter.
     */
    void UseIntegralTermLimiter(bool use_integral_term_limiter);

    /**
     * @brief Get the integral term limiter switch state.
     *
     * @return true Integral term limiter is enabled.
     * @return false Integral term limiter is disabled.
     */
    bool UseIntegralTermLimiter();

    void SetMaximumIntegralTerm(double maximum_integral_term);

    double GetMaximumIntegralTerm();

    /**
     * @brief Set the derivative gain.
     *
     * @param integral_gain Derivative gain. Must be zero or a positive real
     * number.
     */
    void SetDerivativeGain(double integral_gain);

    double GetDerivativeGain();

    double GetDerivativeTerm();

    double GetOutput();

    void UseOutputLimiter(bool use_output_limiter);

    bool UseOutputLimiter();

    void SetMinimumOutput(double minimum_output);

    double GetMinimumOutput();

    void SetMaximumOutput(double maximum_output);

    double GetMaximumOutput();

    /**
     * @brief Set the controller to limit the output change per controller tick.
     *
     * @param use_output_change_limiter Output change limiter switch. True to
     * enable the output change limiter, false to disable the output change
     * limiter.
     */
    void UseOutputChangeLimiter(bool use_output_change_limiter);

    /**
     * @brief Get the output change limiter switch state.
     *
     * @return true Output change limiter is enabled.
     * @return false Output change limiter is disabled.
     */
    bool UseOutputChangeLimiter();

    /**
     * @brief Set the maximum allowed change in output during a controller tick.
     *
     * @param maximum_output_change Maximum allowed output change. Must be a
     * strictly positive number.
     */
    void SetMaximumOutputChange(double maximum_output_change);

    double GetMaximumOutputChange();

    void UseOutputFilter(bool use_output_filter);

    bool UseOutputFilter();

    void SetOutputFilterWeight(double output_filter_weight);

    double GetOutputFilterWeight();

    void Tick();

    void Reset();

  private:
    /**
     * @brief Check if a value is bounded.
     *
     * @param value Value to be tested.
     * @param lower Lower bound.
     * @param upper Upper bound.
     * @return true The value is bounded.
     * @return false The value is out of bounds.
     */
    bool IsBounded(double value, double lower_bound, double upper_bound);

    /** @brief Whether or not this is the first controller tick since the last
     * controller initialization or reset action. */
    bool is_first_tick_ = false;

    /** @brief Setpoint. */
    double setpoint_ = 0.0;

    /** @brief Whether or not to ramp setpoint. */
    bool use_setpoint_ramping_ = false;

    /** @brief Last setpoint. */
    double last_setpoint_ = 0.0;

    /** @brief Maximum setpoint change. */
    double maximum_setpoint_change_ = 0.01;

    /** @brief Plant feedback. */
    double feedback_ = 0.0;

    /** @brief Last feedback. */
    double last_feedback_ = 0.0;

    /** @brief Computed error at the last controller tick. */
    double last_error_;

    /** @brief Feedforward gain. */
    double feedforward_gain_ = 0.0;

    /** @brief Unscaled feedforward term. */
    double unscaled_feedforward_term_ = 0.0;

    /** @brief Feedforward offset. */
    double feedforward_offset_ = 0.0;

    /** @brief Feedforward term. */
    double feedforward_term_ = 0.0;

    /** @brief Proportional gain. */
    double proportional_gain_ = 0.0;

    /** @brief Proportional term. */
    double proportional_term_ = 0.0;

    /** @brief Integral gain. */
    double integral_gain_ = 0.0;

    /** @brief Integral term. */
    double integral_term_ = 0.0;

    /** @brief Whether or not to limit the integral term. */
    bool use_integral_term_limiter_ = false;

    /** @brief Maximum integral term. */
    double maximum_integral_term_ = 0.0;

    /** @brief Cumulative error sum. */
    double cumulative_error_ = 0.0;

    /** @brief Maximum cumulative error. */
    double maximum_cumulative_error_ = 0.5;

    /** @brief Derivative gain. */
    double derivative_gain_ = 0.0;

    /** @brief Derivative term. */
    double derivative_term_ = 0.0;

    /** @brief Controller output. */
    double output_ = 0.0;

    /** @brief Whether or not to limit the controller output to a specified
     * range. */
    bool use_output_limiter_ = false;

    /** @brief Output upper bound. */
    double maximum_output_ = std::numeric_limits<double>::infinity();

    /** @brief Output lower bound. */
    double minimum_output_ = -std::numeric_limits<double>::infinity();

    /** @brief Whether or not to limit the maximum output change between two
     * controller ticks. */
    bool use_output_change_limiter_ = false;

    /** @brief Last controller output. */
    double last_output_ = 0.0;

    /** @brief Maximum output change. */
    double maximum_output_change_ = 0.01;

    /** @brief Whether or not to filter the controller output using an
     * Exponential Moving Average (EMA) filter. */
    bool use_output_filter_ = false;

    /** @brief Exponential moving average weight. */
    double output_filter_weight_ = 1.0;
};

} // namespace ROS
} // namespace OLAV