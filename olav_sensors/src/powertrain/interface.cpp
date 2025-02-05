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

#include <olav_sensors/powertrain/exceptions.hpp>
#include <olav_sensors/powertrain/interface.hpp>

namespace OLAV {
namespace Interfaces {

PowertrainInterface::PowertrainInterface(std::string port,
                                         uint32_t baudrate,
                                         uint32_t timeout)
    : stop_(true),
      status_(STOPPED),
      has_new_data_(false) {
    serial_.setPort(port);
    serial_.setBaudrate(baudrate);

    auto timeoutstruct = serial::Timeout::simpleTimeout(timeout);
    serial_.setTimeout(timeoutstruct);
    std::chrono::system_clock::now();
}

PowertrainInterface::~PowertrainInterface() { Close(); }

void PowertrainInterface::Open() {
    if(IsOpen()) {
        throw OLAV::Exceptions::PowertrainInterfaceException(
            "Connection to the microcontroller is already open.");
    }

    // Check if the worker thread is active but disconnected, and if so, close
    // it before opening a new connection.
    if(status_ == DISCONNECTED) { Close(); }

    try {
        // Open a new connection to the serial port and flush any pre-existing
        // buffer.
        serial_.open();
        serial_.flush();

        try {
            serial_.setDTR(true);
        } catch(serial::SerialException& exception) {
            // Data ready is not available on pseudo-terminals, so we capture
            // the SerialException and ignore it.

            // TODO: Signal that we are ignoring this exception through spdlog.
        }

        stop_ = false;
        worker_ = std::thread(&PowertrainInterface::Worker, this);

        while(status_ == STOPPED) {
            // Wait until the worker start spinning.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch(serial::IOException& exception) {
        std::stringstream stream;
        stream << "Failed to open serial connection to the microcontroller: "
               << exception.what();
        throw OLAV::Exceptions::PowertrainInterfaceException(
            stream.str().c_str());
    }
}

void PowertrainInterface::Close() {
    if(status_ != STOPPED) {
        // Trigger the atomic halt flag and join the background thread.
        stop_ = true;
        worker_.join();

        // Close the serial port.
        serial_.close();

        // Mark the interface as stopped.
        status_ = STOPPED;
    }
}

bool PowertrainInterface::IsOpen() {
    if(status_ == CONNECTED) { return true; }
    return false;
}

bool PowertrainInterface::HasNewData() {
    if(has_new_data_) {
        has_new_data_ = false;
        return true;
    }
    return false;
}

bool PowertrainInterface::IsReceivingData() {
    if(IsOpen()) {
        long time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now() - last_message_)
                        .count();

        // TODO: Expose this hardcoded timeout.
        if(time < 200) { return true; }
    }

    return false;
}

void PowertrainInterface::Worker() {
    // FIXME: Handle the emergency stop.
    // int emergency_stop = 0;
    int axle_speed_cdps = 0;
    int engine_speed_rpm = 0;

    while(!stop_ && serial_.isOpen()) {
        status_ = CONNECTED;

        try {
            std::string data = serial_.readline();

            if(!data.empty() && data.find("UGV:") == 0) {
                // Allocate a buffer to store a pointer to the substring
                // currently being parsed.
                int buffer;

                // Parse the centidegree per second, the first substring after
                // the colon.
                buffer = data.find_first_of(":") + 1;
                axle_speed_cdps = std::atoi(data.substr(buffer).c_str());

                // Parse the engine rotational speed.
                buffer = data.find_first_of(",") + 1;
                engine_speed_rpm = std::atoi(data.substr(buffer).c_str());

                // Parse the emergency stop signal.
                buffer = data.find_first_of(",", buffer) + 1;
                // emergency_stop = std::atoi(data.substr(buffer).c_str());
            }
        } catch(...) { stop_ = true; }

        // Update the engine and axle speeds.
        engine_speed_ = engine_speed_rpm;
        axle_speed_ = (0.01 * axle_speed_cdps) / 360.0;
    }

    status_ = DISCONNECTED;
}

double PowertrainInterface::GetAxleSpeed() { return axle_speed_; }

double PowertrainInterface::GetEngineSpeed() { return engine_speed_; }

} // namespace Interfaces
} // namespace OLAV