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

#include <olav_sensors/serial/interface.hpp>

namespace OLAV {
namespace ROS {

SerialDeviceInterface::SerialDeviceInterface(const std::string& port,
                                             const int& baudrate)
    : port_(port),
      baudrate_(baudrate) {
    Initialize();
}

void SerialDeviceInterface::Initialize() {
    interface_ = std::make_unique<serial::Serial>();

    auto timeout = serial::Timeout::simpleTimeout(1000);

    interface_->setPort(port_);
    interface_->setBaudrate(baudrate_);
    interface_->setTimeout(timeout);
}

void SerialDeviceInterface::Connect() {
    while(!interface_->isOpen()) {
        try {
            interface_->open();
            is_connected_ = true;
        } catch(const serial::SerialException& exception) {}
    }

    try {
        interface_->flush();
        interface_->setDTR(true);
    } catch(const serial::PortNotOpenedException& exception) {}
}

const bool& SerialDeviceInterface::IsConnected() const { return is_connected_; }

nlohmann::json SerialDeviceInterface::Read() {
    // Pre-allocate an ordered JSON message. The ordering is kept such that a
    // valid CRC32 checksum can be computed from the string representation of
    // the JSON as published on the serial port.
    std::string checksum;

    // TODO Yeah you need a timeeout
    while(true) {
        try {
            // Find the start of header (\x001) character marking the start of a
            // sentence.
            ReadUntil("\x001");

            // Read the line until the start of message character (\x002). In a
            // valid message sentence, this line should be the checksum
            // dictionary preceding the data message.
            {
                auto checksum_token = ReadUntil("\x002");
                auto buffer = nlohmann::ordered_json::parse(checksum_token);

                // Store the data message hash after checking the message
                // metadata to ensure this message token is a checksum.
                if(buffer.at("metadata").at("type").get<std::string>() ==
                   "checksum") {
                    checksum = buffer.at("data").at("hash").get<std::string>();
                } else {
                    continue;
                }
            }

            // Read the line until the end of message character (\x003). In a
            // valid message sentence, this line should be the data message
            // dictionary.
            {
                auto data_token = ReadUntil("\x003");
                auto buffer = nlohmann::ordered_json::parse(data_token);
                if(!IsChecksumValid(buffer.dump(), checksum)) { continue; }

                // Read the trailing part of the message until the newline
                // character
                // (\n). This should only read "\n" under normal circumstances.
                ReadUntil("\n");

                return buffer;
            }

        } catch(const nlohmann::detail::parse_error& exception) {
            continue;
        } catch(const nlohmann::json::type_error& exception) { continue; }
    }
    throw std::invalid_argument("");
}

bool SerialDeviceInterface::IsChecksumValid(std::string token,
                                            std::string checksum) {
    CRC32 crc;
    crc.Update((uint8_t*)token.data(), token.length());
    if(std::to_string(crc.GetValue()) == checksum) { return true; }
    return false;
}

// TODO: add timeout
std::string SerialDeviceInterface::ReadUntil(std::string delimiter) {
    std::string data_str;
    std::string readvalue = interface_->read();
    while(true) {
        if(memcmp(readvalue.c_str(), delimiter.c_str(), 1)) {
            data_str += readvalue;
            readvalue = interface_->read();
        } else {
            break;
        }
    }
    return data_str;
}
// return message;
//} catch(...) {}
// throw MicrocontrollerInterfaceExceptions::NoMessages();

SerialDeviceStatusMessage SerialDeviceInterface::GetStatus(
    nlohmann::json& message) { // TODO: To be implemented.
    (void)message;
    throw std::logic_error("Not implemented.");
}

} // namespace ROS
} // namespace OLAV