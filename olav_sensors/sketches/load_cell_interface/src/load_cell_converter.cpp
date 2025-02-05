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

#include <load_cell_interface/load_cell_converter.hpp>

namespace OLAV {
namespace Microcontrollers {

LoadCellConverter::LoadCellConverter()
    : converter_(&Wire),
      range_(Range::ADC_RANGE_MEDIUM) {}

LoadCellConverter::LoadCellConverter(const Range& range)
    : converter_(&Wire),
      range_(range) {}

void LoadCellConverter::Initialize() {
    converter_.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);
    SetRange(range_);
    converter_.setMode(eMODE_CONTIN);
    converter_.setRate(eRATE_860);
    converter_.init();
}

bool LoadCellConverter::Check() { return converter_.checkADS1115(); }

uint16_t LoadCellConverter::Read() { return ReadPin(0); }

uint16_t LoadCellConverter::ReadPin(const uint8_t pin) {
    if(Check()) {
        return converter_.readVoltage(pin);
    } else {
        return -1;
    }
}

void LoadCellConverter::SetRange(const Range& range) {
    converter_.setGain(eADSGain_t(range));
}

} // namespace Microcontrollers
} // namespace OLAV