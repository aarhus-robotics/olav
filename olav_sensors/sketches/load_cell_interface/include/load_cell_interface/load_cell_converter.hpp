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

#include <DFRobot_ADS1115.h>
#include <Wire.h>

namespace OLAV {
namespace Microcontrollers {

enum Range {
    /* 0~6.144 V range (0.1875 mV resolution) */
    ADC_RANGE_HIGHEST = 0,
    /* 0~4.096 V range (0.125 mV resolution) */
    ADC_RANGE_HIGH = 1,
    /* 0~2.048 V (0.0625 mV resolution) */
    ADC_RANGE_MEDIUM = 2,
    /* 0~1.024 V (0.03125 mV resolution) */
    ADC_RANGE_LOW = 3,
    /* 0~0.512 V (0.015625 mV resolution) */
    ADC_RANGE_LOWER = 4,
    /* 0~0.256 V (0.0078125mV mV resolution) */
    ADC_RANGE_TINY = 5
};

class LoadCellConverter {
  public:
    LoadCellConverter();

    LoadCellConverter(const Range& range);

    LoadCellConverter(const Range& range, const uint8_t adc_pin);

    void SetRange(const Range& range);

    bool Check();

    uint16_t Read();

    uint16_t ReadPin(const uint8_t pin);

    /**
     * @brief Initialize the ADS1115 module with default settings.
     *
     * @details The default settings configures the sensor on I2C address 0x48,
     * in continuous-conversion mode and converting at the maximum rate of 860
     * SPS.
     *
     */
    void Initialize();

  private:
    DFRobot_ADS1115 converter_;

    Range range_;
};

} // namespace Microcontrollers
} // namespace OLAV