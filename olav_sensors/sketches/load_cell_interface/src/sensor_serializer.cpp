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

#include <load_cell_interface/sensor_serializer.hpp>

namespace OLAV {
namespace Microcontrollers {

SensorSerializer::SensorSerializer(HardwareSerial& serial) : serial_(serial) {
    document_["status"].to<JsonObject>();
    document_["sensors"].to<JsonObject>();
    document_["metadata"]["type"] = "packet";
}

void SensorSerializer::Serialize() {
    document_["sensors"]["load_cell"]["data"] = analogRead(2);

    String string;
    serializeJson(document_, string);

    String checksum =
        String(calcCRC32((uint8_t*)string.begin(), string.length()));

    JsonDocument checksummsg;
    checksummsg["metadata"]["type"] = "checksum";
    checksummsg["data"]["hash"] = checksum;

    // serial_.print(SOH);
    serializeJson(checksummsg, serial_);
    // serial_.print(STX);
    serializeJson(document_, serial_);
    // serial_.print(ETX);
    serial_.println();
}

void SensorSerializer::SetMessage(String message) {
    document_["status"]["message"] = message;
}

void SensorSerializer::SetEnabled(bool state) {
    document_["status"]["enabled"] = state;
}

void SensorSerializer::SetRate(double rate) {
    document_["staus"]["rate"] = rate;
}

void SensorSerializer::AddSensor(String sensor) {
    document_["sensors"][sensor].to<JsonObject>();
}

void SensorSerializer::SetReading(String sensor, JsonObject& reading) {
    if(document_["sensor"].containsKey(sensor)) {
        document_["sensors"][sensor] = reading;
    }
}

} // namespace Microcontrollers
} // namespace OLAV
