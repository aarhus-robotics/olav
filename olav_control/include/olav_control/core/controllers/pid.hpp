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

    void SetSetpoint(const double& setpoint);

    const double& GetSetpoint() const;

    /**
     * @brief Set whether or not the controller uses a setpoint ramping
     * strategy.
     *
     * @param use_setpoint_ramping Setpoint ramping strategy setting.
     */
    void UseSetpointRamping(const bool& use_setpoint_ramping);

    /**
     * @brief Check whether or not the controller uses a setpoint ramping
     * strategy.
     *
     * @return true The controller is ramping setpoints.
     * @return false The controller is not ramping setpoints.
     */
    const bool& UseSetpointRamping() const;

    /**
     * @brief Set the maximum allowed setpoint change between controller ticks
     * before the setpoint is ramped.
     *
     * @param maximum_setpoint_change Maximum allowed setpoint change.
     */
    void SetMaximumSetpointChange(const double& maximum_setpoint_change);

    /**
     * @brief Get the maximum allowed setpoint change between controller ticks
     * before the setpoint is ramped.
     *
     * @return const double& Maximum allowed setpoint change.
     */
    const double& GetMaximumSetpointChange() const;

    /**
     * @brief Provide the controller with the plant feedback value.
     *
     * @param feedback Plant feedback value.
     */
    void SetFeedback(const double& feedback);

    /**
     * @brief Get the plant feedback value currently stored in the controller.
     *
     * @return const double& Plant feedback value.
     */
    const double& GetFeedback() const;

    /**
     * @brief Set the feedforward gain.
     *
     * @param feedforward_gain Feedforward gain.
     */
    void SetFeedforwardGain(const double& feedforward_gain);

    /**
     * @brief Get the feedforward gain.
     *
     * @return const double& Feedforward gain.
     */
    const double& GetFeedforwardGain() const;

    /**
     * @brief Set the unscaled feedforward term. This method must be called by
     * an upstream feedforward model before controller ticks.
     *
     * @param unscaled_feedforward_term Unscaled feedforward term.
     */
    void SetUnscaledFeedforwardTerm(const double& unscaled_feedforward_term);

    /**
     * @brief Get the feedforward term.
     *
     * @return const double& Feedforward term.
     */
    const double& GetFeedforwardTerm() const;

    /**
     * @brief Get the feedforward offset.
     *
     * @return const double& Feedforward offset.
     */
    const double& GetFeedforwardOffset() const;

    /**
     * @brief Set the feedforward offset.
     *
     * @param feedforward_offset Feedforward offset.
     */
    void SetFeedforwardOffset(const double& feedforward_offset);

    /**
     * @brief Set the proportional gain.
     *
     * @param proportional_gain Proportional gain.
     */
    void SetProportionalGain(const double& proportional_gain);

    /**
     * @brief Get the proportional gain.
     *
     * @return const double& Proportional gain.
     */
    const double& GetProportionalGain() const;

    /**
     * @brief Get the proportional term.
     *
     * @return const double& Proportional term.
     */
    const double& GetProportionalTerm() const;

    void SetMaximumError(double maximum_error);

    /**
     * @brief Set the integral gain
     *
     * @param integral_gain Integral gain. Must be zero or a positive real
     * number.
     */
    void SetIntegralGain(double integral_gain);

    /**
     * @brief Get the integral gain.
     *
     * @return const double& Integral gain.
     */
    const double& GetIntegralGain() const;

    /**
     * @brief Get the integral term.
     *
     * @return const double& Integral term.
     */
    const double& GetIntegralTerm() const;

    /**
     * @brief Set whether or not the controller will limit the integral term
     * magnitude.
     *
     * @param use_integral_term_limiter Integral term limiter setting.
     */
    void UseIntegralTermLimiter(const bool& use_integral_term_limiter);

    /**
     * @brief Check whether or not the controller is limiting the integral term
     * magnitude.
     *
     * @return true The controller is limiting the integral term magnitude.
     * @return false The controller is not limiting the integral term magnitude.
     */
    const bool& UseIntegralTermLimiter() const;

    /**
     * @brief Set the maximum allowed integral term.
     *
     * @param maximum_integral_term Maximum allowed integral term.
     */
    void SetMaximumIntegralTerm(const double& maximum_integral_term);

    /**
     * @brief Get the maximum allowed integral term.
     *
     * @return const double& Maximum allowed integral term.
     */
    const double& GetMaximumIntegralTerm() const;

    /**
     * @brief Set the derivative gain.
     *
     * @param integral_gain Derivative gain.
     */
    void SetDerivativeGain(const double& derivative_gain);

    /**
     * @brief Get the derivative gain.
     *
     * @return const double& Derivative gain.
     */
    const double& GetDerivativeGain() const;

    /**
     * @brief Get the derivative term.
     *
     * @return const double& Derivative term.
     */
    const double& GetDerivativeTerm() const;

    /**
     * @brief Get the controller output.
     *
     * @return const double& Controller output.
     */
    const double& GetOutput() const;

    /**
     * @brief Set whether or not the controller will clamp its output.
     *
     * @param use_output_limiter Output limiter setting.
     */
    void UseOutputLimiter(const bool& use_output_limiter);

    /**
     * @brief Check whether or not the controller will clamp its output.
     *
     * @return true The controller will clamp its output.
     * @return false The controller will not clamp its output.
     */
    const bool& UseOutputLimiter() const;

    /**
     * @brief Set the minimum allowed controller output when the output limiter
     * is enabled.
     *
     * @param minimum_output Minimum allowed controller output.
     */
    void SetMinimumOutput(const double& minimum_output);

    /**
     * @brief Get the minimum allowed controller output when the output limiter
     * is enabled.
     *
     * @return double Minimum allowed controller output.
     */
    const double& GetMinimumOutput() const;

    /**
     * @brief Set the maximum allowed controller output when the output limiter
     * is enabled.
     *
     * @param maximum_output Maximum allowed controller output.
     */
    void SetMaximumOutput(const double& maximum_output);

    /**
     * @brief Get the maximum allowed controller output when the output limiter
     * is enabled.
     *
     * @return double Maximum allowed controller output.
     */
    const double& GetMaximumOutput() const;

    /**
     * @brief Set the controller to limit the output change per controller tick.
     *
     * @param use_output_change_limiter Output change limiter switch. True to
     * enable the output change limiter, false to disable the output change
     * limiter.
     */
    void UseOutputChangeLimiter(const bool& use_output_change_limiter);

    /**
     * @brief Get the output change limiter switch state.
     *
     * @return true Output change limiter is enabled.
     * @return false Output change limiter is disabled.
     */
    const bool& UseOutputChangeLimiter() const;

    /**
     * @brief Set the maximum allowed change in output during a controller tick.
     *
     * @param maximum_output_change Maximum allowed output change.
     */
    void SetMaximumOutputChange(const double& maximum_output_change);

    /**
     * @brief Get the maximum allowed change in output during a controller tick.
     *
     * @return double Maximum allowed output change.
     */
    const double& GetMaximumOutputChange() const;

    void UseOutputFilter(const bool& use_output_filter);

    const bool& UseOutputFilter() const;

    void SetOutputFilterWeight(const double& output_filter_weight);

    const double& GetOutputFilterWeight() const;

    void UseDeadbandFilter(const bool& use_deadband_filter);

    const bool& UseDeadbandFilter() const;

    const bool& IsDeadbanding() const;

    void SetDeadbandCenter(const double& deadband_center);

    void SetDeadbandLowerThreshold(const double& deadband_lower_threshold);

    void SetDeadbandUpperThreshold(const double& deadband_upper_threshold);

    void UseErrorThreshold(const bool& use_error_threshold);

    const bool& UseErrorThreshold() const;

    void SetErrorThreshold(const double& error_threshold);

    const double& GetErrorThreshold() const;

    void Tick();

    void Reset();

  private:
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
    double maximum_cumulative_error_ = 0.0;

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

    /** @brief Whether or not to use the deadband filter. */
    bool use_deadband_filter_ = false;

    /** @brief Whether or not the controller output fell within the deadband
     * during the last controller tick. */
    bool is_deadbanding_ = false;

    double deadband_center_ = 0.0;

    /** @brief Deadband lower threshold. */
    double deadband_lower_threshold_ = -std::numeric_limits<double>::infinity();

    /** @brief Deadband upper threshold. */
    double deadband_upper_threshold_ = std::numeric_limits<double>::infinity();

    /** @brief Whether or not to use the error threshold to limit action around
     * the equilibrium point. */
    bool use_error_threshold_ = false;

    /** @brief Error threshold around equilibrium point. */
    double error_threshold_ = -std::numeric_limits<double>::infinity();

    const bool& UseTransientFeedforward();

    void UseTransientFeedforward(const bool& use_transient_feedfoward);

    bool use_transient_feedforward_ = false;

    double feedforward_hysteresis_ = 0.0;
};

} // namespace ROS
} // namespace OLAV