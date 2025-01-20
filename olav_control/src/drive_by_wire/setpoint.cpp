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

#include <olav_control/drive_by_wire/exceptions.hpp>
#include <olav_control/drive_by_wire/setpoint.hpp>

namespace OLAV {
namespace ROS {

DriveByWireSetpoint::DriveByWireSetpoint() {}

DriveByWireSetpoint::DriveByWireSetpoint(const double& throttle,
                                         const double& brake,
                                         const double& steering,
                                         const int& gear,
                                         const bool& ignition,
                                         const bool& engine_starter,
                                         const int& emergency_stop)
    : throttle_(throttle),
      brake_(brake),
      steering_(steering),
      gear_(gear),
      ignition_(ignition),
      engine_starter_(engine_starter),
      emergency_stop_(emergency_stop) {}

void DriveByWireSetpoint::SetThrottle(double throttle) {
    if(throttle < 0.0 || throttle > 1.0) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Throttle value is outside of the valid range (0.0 - 1.0).");
    }
    throttle_ =
        static_cast<int16_t>(std::floor(throttle * throttle_scale_factor_));
}

void DriveByWireSetpoint::SetThrottleRaw(int16_t throttle) {
    if(throttle < 0 || throttle > 100) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Raw throttle value is outside of the valid range (0 - 100).");
    }
    throttle_ = throttle;
}

const int16_t& DriveByWireSetpoint::GetThrottle() { return throttle_; }

void DriveByWireSetpoint::SetBrake(double brake) {
    if(brake < 0.0 || brake > 1.0) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Brake value is outside of the valid range (0.0 - 1.0).");
    }
    brake_ = static_cast<int16_t>(std::floor(brake * brake_scale_factor_));
}

void DriveByWireSetpoint::SetBrakeRaw(int16_t brake) {
    if(brake < 0 || brake > 100) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Raw brake value is outside of the valid range (0 - 100).");
    }
    brake_ = brake;
}

const int16_t& DriveByWireSetpoint::GetBrake() { return brake_; }

void DriveByWireSetpoint::SetSteering(double steering) {
    if(steering < -1.0 || steering > 1.0) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Steering value is outside of the valid range (-1.0 - 1.0).");
    }
    steering_ = static_cast<int16_t>(
        std::round((steering + steering_offset_) * steering_scale_factor_));
}

void DriveByWireSetpoint::SetSteeringRaw(int16_t steering) {
    if(steering < -6000 || steering > 6000) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Raw steering value is outside of the valid range (-6000 - 6000).");
    }
    steering_ = steering;
}

const int16_t& DriveByWireSetpoint::GetSteering() { return steering_; }

void DriveByWireSetpoint::SetGear(int16_t gear) {
    if(gear < 1 || gear > 5) {
        throw OLAV::Exceptions::DriveByWireInterfaceException(
            "Selected gear is outside of the valid range (1, 5).");
    }
    gear_ = gear;
}

const int16_t& DriveByWireSetpoint::GetGear() { return gear_; }

void DriveByWireSetpoint::SetIgnition(bool ignition) { ignition_ = ignition; }

bool& DriveByWireSetpoint::GetIgnition() { return ignition_; }

void DriveByWireSetpoint::StartEngine(bool engine_starter) {
    engine_starter_ = engine_starter;
}

bool& DriveByWireSetpoint::GetEngineStarter() { return engine_starter_; }

void DriveByWireSetpoint::SetEmergencyStop(bool emergency_stop) {
    emergency_stop_ = emergency_stop;
}

bool& DriveByWireSetpoint::GetEmergencyStop() { return emergency_stop_; }

} // namespace ROS
} // namespace OLAV