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

#include <cmath>

#include <olav_control/core/controllers/pid.hpp>

namespace OLAV {
namespace ROS {

class CVTFeedforwardModel {
  public:
    /**
     * @brief Construct a new CVT feedforward model.
     */
    CVTFeedforwardModel();

    void SetRadius(const double& radius);

    /**
     * @brief Set the current terrain inclination, expressed in radians, used to
     * compute the magnitude of the feedforward term.
     *
     * @param inclination Terrain inclination expressed in radians.
     */
    void SetInclination(const double& inclination);

    /**
     * @brief Compute the feedforward term magnitude from the current state of
     * the CVT feedforward model.
     *
     * @return double Magnitude of the feedforward term.
     */
    double Compute();

  private:
    /** @brief Tire radius, expressed in meters. */
    double radius_ = 0.0;

    /** @brief Terrain inclination, expressed in radians. */
    double inclination_ = 0.0;

    /** @brief Dynamic tire radius, expressed in metres. */
    double dynamic_tire_radius_ = 0.33;

    /** @brief Estimated engine friction, expressed in Newton metre seconds per
     * radian. */
    double engine_friction_ = 0.155;

    /** @brief Estimated engine speed required to engage the CVT belt, expressed
     * in radians per second. */
    double engine_speed_belt_engagement_ = 209.0;

    /** @brief Maximum speed ratio of the CVT transmission. */
    double cvt_speed_ratio_max_ = 3.81;

    /** @brief Minimum speed ratio of the CVT transmission. */
    double cvt_speed_ratio_min_ = 0.76;

    /** @brief Speed ratio of the CVT transmission when the currently selected
     * gear is "HIGH". */
    double cvt_speed_ratio_high_ = 10.4;

    /** @brief Speed ratio of the CVT transmission when the currently selected
     * gear is "LOW". */
    double cvt_speed_ratio_low_ = 24.59;

    /** @brief Speed ratio of the CVT transmission when the currently selected
     * gear is "REVERSE". */
    double speed_ratio_reverse_ = 22.92;

    /** @brief Upshift sensitivity of the CVT transmission. */
    double cvt_upshift_sensitivity_ = 276.0;

    /** @brief Downshift sensitivity of the CVT transmission.  */
    double cvt_downshift_sensitivity_ = 197.0;

    /** @brief Air resistance, expressed in Netwon per second squared per metre
     * squared. */
    double air_resistance_ = 2.55;

    /** @brief Vehicle rolling resistance, expressed in Netwon per kilogram. */
    double rolling_resistance_ = 0.0157;

    /** @brief Vehicle mass, expressed in kilograms. */
    double vehicle_mass_ = 800.0;

    /** @brief Maximum braking torque, expressed in Netwon metre. */
    double max_brake_torque_ = 3500.0;

    /** @brief Minimum brake effort before actuation, expressed as a percentage.
     */
    double min_brake_percent_ = 20.0;

    /** @brief Inclination reading on a flat terrain, expressed in radians. */
    double inclination_offset_ = 0.0;
};

} // namespace ROS
} // namespace OLAV
