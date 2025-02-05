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

class DriveByWireSetpoint {
  public:
    DriveByWireSetpoint();

    DriveByWireSetpoint(const double& throttle,
                        const double& brake,
                        const double& steering,
                        const int& gear,
                        const bool& ignition,
                        const bool& engine_starter,
                        const int& emergency_stop);

    void SetThrottle(double throttle);

    void SetThrottleRaw(int16_t throttle);

    const int16_t& GetThrottle();

    void SetBrake(double brake);

    void SetBrakeRaw(int16_t brake);

    const int16_t& GetBrake();

    void SetSteering(double steering);

    void SetSteeringRaw(int16_t steering);

    const int16_t& GetSteering();

    void SetGear(int16_t gear);

    const int16_t& GetGear();

    void SetIgnition(bool ignition);

    bool& GetIgnition();

    void StartEngine(bool engine_starter);

    bool& GetEngineStarter();

    void SetEmergencyStop(bool emergency_stop);

    bool& GetEmergencyStop();

  private:
    int16_t throttle_ = 0;

    double throttle_scale_factor_ = 100.0;

    int16_t brake_ = 100;

    double brake_scale_factor_ = 100.0;

    int16_t steering_ = 0;

    double steering_offset_ = 0.17;

    double steering_scale_factor_ = 1500.0;

    int16_t gear_ = 1;

    bool ignition_ = false;

    bool engine_starter_ = false;

    bool emergency_stop_ = false;
};

} // namespace ROS
} // namespace OLAV