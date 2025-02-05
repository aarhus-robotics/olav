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
#include <stdint.h>

namespace OLAV {
namespace ROS {

class DriveByWireFeedback {
  public:
    DriveByWireFeedback();

    DriveByWireFeedback(const int16_t& steering_actuator_position,
                        const int16_t& selected_gear,
                        const bool& is_ignition_on,
                        const bool& is_autonomous_mode_on,
                        const int16_t& brake_actuator_position,
                        const int16_t& gear_actuator_position,
                        const bool& is_gear_actuator_in_position);

    void IsIgnitionOn(bool is_ignition_on);

    const bool& IsIgnitionOn();

    void IsAutonomousModeOn(bool is_autonomous_mode_on);

    const bool& IsAutonomousModeOn();

    void SetSteeringActuatorPosition(int16_t steering_actuator_position);

    const int16_t& GetSteeringActuatorPosition();

    double GetSteeringActuatorPositionInRadians();

    double GetSteeringActuatorPositionInDegrees();

    void SetBrakeActuatorPosition(int16_t brake_actuator_position);

    const int16_t& GetBrakeActuatorPosition();

    void SetSelectedGear(int16_t selected_gear);

    const int16_t& GetSelectedGear();

    void SetGearActuatorPosition(int16_t gear_actuator_position);

    const int16_t& GetGearActuatorPosition();

    void IsGearActuatorInPosition(bool is_gear_actuator_in_position);

    const bool& IsGearActuatorInPosition();

  private:
    /**
     * @brief Steering actuator position register value.
     */
    int16_t steering_actuator_position_ = -1;

    /**
     * @brief Steering actuator voltage offset in millivolt corresponding to
     * centered steering wheel for a null reading.
     */
    int steering_actuator_voltage_offset_ = 10776.0;

    /**
     * @brief Steering actuator voltage gain in millivolt per radian allowing to
     * convert voltage readings to steering wheel angles in radians.
     */
    double steering_actuator_voltage_gain_ = 17399.0;

    /**
     * @brief Selected gear register value.
     */
    int16_t selected_gear_ = -1;

    /**
     * @brief Ignition state register value.
     */
    bool is_ignition_on_ = false;

    /**
     * @brief Autonomous mode switch state register value.
     */
    bool is_autonomous_mode_on_ = false;

    /**
     * @brief Brake actuator position register value.
     */
    int16_t brake_actuator_position_ = -1;

    /**
     * @brief Gear actuator position register value.
     */
    int16_t gear_actuator_position_ = -1;

    /**
     * @brief Gear actuator state register value.
     */
    bool is_gear_actuator_in_position_ = false;
};

} // namespace ROS
} // namespace OLAV