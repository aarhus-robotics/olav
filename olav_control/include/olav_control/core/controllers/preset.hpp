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

#include <string>

namespace OLAV {
namespace ROS {

/**
 * @brief Class defining a PID controller preset. This class can be used easily
 * store and apply a predefined configuration for a PID controller.
 */
class ControllerPreset {
  public:
    /**
     * @brief Construct a new PID controller preset.
     *
     * @param identifier Friendly name used to identify and apply the PID
     * controller preset. It is up to the container class storing the presets to
     * ensure the identifier is unique.
     */
    ControllerPreset(const std::string& identifier,
                     const double& feedforward_gain,
                     const double& proportional_gain,
                     const double& integral_gain,
                     const double& derivative_gain);

  private:
    /**
     * @brief Identifier for the PID controller preset. It is up to the
     * container class storing the presets to ensure the identifier is unique.
     */
    std::string identifier_;

    /**
     * @brief PID controller feedforward gain stored in the preset.
     */
    double feedforward_gain_;

    /**
     * @brief PID controller proportional gain stored in the preset.
     */
    double proportional_gain_;

    /**
     * @brief PID controller integral gain stored in the preset.
     */
    double integral_gain_;

    /**
     * @brief PID controller derivative gain stored in the preset.
     */
    double derivative_gain_;
};

} // namespace ROS
} // namespace OLAV