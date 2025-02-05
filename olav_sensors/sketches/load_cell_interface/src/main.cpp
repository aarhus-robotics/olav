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

#include <Arduino.h>

#include <TaskScheduler.h>

#include <load_cell_interface/load_cell_converter.hpp>
#include <load_cell_interface/load_cell_serializer.hpp>
#include <load_cell_interface/sensor_deserializer.hpp>

using namespace OLAV::Microcontrollers;

// Serial interface
// ----------------
const uint32_t g_baudrate = 115200;

// Task scheduling
// ---------------
Scheduler g_scheduler;

void ReadTaskCallback();
const int serialization_rate = 1;
Task g_read_task(1000.0 * (1.0 / serialization_rate),
                 TASK_FOREVER,
                 &ReadTaskCallback);

// Load cell
// ---------
/** @brief Load cell sensor analog-to-digital converter (ADC). */
LoadCellConverter g_converter;

/** @brief Load cell sensor JSON serializer. */
LoadCellSerializer g_serializer(Serial);
SensorDeserializer g_deserializer(Serial);

/**
 * @brief Microcontroller setup routine.
 */
void setup() {
    // Initialize the serial interface and flush the buffer.
    Serial.begin(g_baudrate);

    g_converter.Initialize();
    while(!g_converter.Check()) {
        g_serializer.SetEnabled(false);
        g_serializer.SetMessage("Initializing ADC...");
        g_serializer.Serialize();
        delay(2000.0 * 1.0 / serialization_rate);
    }

    g_scheduler.addTask(g_read_task);

    g_read_task.enable();
}

/**
 * @brief Microcontroller execution routine.
 */
void loop() {
    Serial.println(g_converter.Read());

    /*
    auto command = g_deserializer.ReadCommand();
    if(!command.isNull()) {
        if(command.containsKey("rate")) {
            g_read_task.setInterval(1000.0 * 1.0 / int(command["rate"]));
        } else if(command.containsKey("range")) {
            // g_sensor.SetRange(range);
        }
        delay(1000);
    }
    */

    // Execute the scheduled tasks.
    g_scheduler.execute();
}

void ReadTaskCallback() {}