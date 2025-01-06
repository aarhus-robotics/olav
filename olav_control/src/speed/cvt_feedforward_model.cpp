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

#include <olav_control/speed/cvt_feedforward_model.hpp>

namespace OLAV {
namespace ROS {

CVTFeedforwardModel::CVTFeedforwardModel() {}

void CVTFeedforwardModel::SetRadius(const double& radius) { radius_ = radius; }

void CVTFeedforwardModel::SetInclination(const double& inclination) {
    inclination_ = inclination;
}

double CVTFeedforwardModel::Compute() {
    // Calculate target speed in radians.
    double target_speed = radius_ / dynamic_tire_radius_;

    // Set transmission speed ratio.
    double speed_ratio = cvt_speed_ratio_low_;

    // Correct inclination_ if IMU data is skewed
    inclination_ += inclination_offset_;

    // Calculate forces acting on vehicle at steady state
    double drag_force = air_resistance_ * target_speed * target_speed +
        rolling_resistance_ * std::cos(inclination_) * vehicle_mass_;
    double gravitational_force = std::sin(inclination_) * vehicle_mass_ * 9.81;
    double target_force = drag_force + gravitational_force;

    // Calculate needed engine torque (at max cvt ratio)
    double target_torque = (target_force > 0)
        ? target_force * dynamic_tire_radius_ / speed_ratio
        : 0.0;

    double feedforward_throttle = 0.0;
    double feedforward_brake = 0.0;
    double target_engine_speed;
    double clamped_speed_ratio;

    // TODO: What is this?
    double r_eff = 0.0;
    double P1 = 0.0;
    double P2 = 0.0;
    double P3 = 0.0;

    // Calculate steady state engine speed at zero throttle
    target_engine_speed =
        (cvt_speed_ratio_max_ -
         (cvt_speed_ratio_max_ - cvt_speed_ratio_min_) *
             (-engine_speed_belt_engagement_ / cvt_upshift_sensitivity_ -
              target_torque / cvt_downshift_sensitivity_)) /
        (r_eff / (speed_ratio * target_speed) +
         (cvt_speed_ratio_max_ - cvt_speed_ratio_min_) /
             cvt_upshift_sensitivity_);

    // Calculate and clamp cvt speed ratio
    clamped_speed_ratio = boost::algorithm::clamp(
        cvt_speed_ratio_max_ -
            (cvt_speed_ratio_max_ - cvt_speed_ratio_min_) *
                ((target_engine_speed - engine_speed_belt_engagement_) /
                     cvt_upshift_sensitivity_ -
                 target_torque / cvt_downshift_sensitivity_),
        cvt_speed_ratio_min_,
        cvt_speed_ratio_max_);

    // Recalculate engine speed with clamped cvt speed ratio
    target_engine_speed =
        clamped_speed_ratio * speed_ratio * target_speed / dynamic_tire_radius_;

    // If throttle is enough to regulate speed
    if(((-target_engine_speed * engine_friction_ * clamped_speed_ratio *
         speed_ratio / dynamic_tire_radius_) < target_force) &&
       (target_engine_speed > engine_speed_belt_engagement_)) {
        // Calculate feed forward throttle
        feedforward_throttle = 100.0 *
            ((target_force * dynamic_tire_radius_) /
                 (clamped_speed_ratio * speed_ratio) +
             engine_friction_ * target_engine_speed) /
            (P1 + (P2 + engine_friction_) * target_engine_speed +
             P3 * target_engine_speed * target_engine_speed);
        feedforward_throttle =
            boost::algorithm::clamp(feedforward_throttle, 0.0, 100.0);
    } else {
        // Calculate feed forward brake
        if(target_engine_speed > engine_speed_belt_engagement_) {
            target_force += engine_friction_ * target_engine_speed;
        }
        feedforward_brake = boost::algorithm::clamp(
            -target_force / (max_brake_torque_ / dynamic_tire_radius_) *
                    (100.0 - min_brake_percent_) +
                min_brake_percent_,
            0,
            100.0);
    }

    return feedforward_throttle - feedforward_brake;
}

} // namespace ROS
} // namespace OLAV
