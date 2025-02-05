Launching the system
====================

Before launching OLAV, ensure all computing systems are turned on and fully booted.

Set the vehicle to autonomous mode by flipping the autonomy switch to the high
position. Then, turn on the ignition. The drive-by-wire system will take over the throttle valve, brake wire and gear switch actuator and set the brake to fully depressed. Once the startup procedure is complete and no actuators are moving, launch the main node from a new terminal from the OLAV HMI:

.. code-block:: shell

    ros2-launch-olav

Heartbeat
---------

OLAV requires a heartbeat signal to ensure the upstream control system is active and healthy.

The heartbeat must be published at a minimum frequnecy for the OLAV controller to be in an active state. The minimum
frequency can be specified in the OLAV controller node parameters and defaults to 100 Hz. It is highly recommended to
set the minimum heartbeat frequency at least as high as the fastest control loop in any of the controllers in the
system.

The heartbeat is implemented as a ROS topic `/heartbeat` with message type `std_msgs/msg/Header`. The contents of the
published message are not parsed by the controller.

Requirements
------------

* The heartbeat signal is being published at the specified minimum frequency.
* The ready service has been called at least once since the last controller reset.
* A valid feedback is being received on the Modbus interface of the drive-by-wire programmable logic controller.
* A valid line is being read on the serial port of the wheel odometry microcontroller.
* The vehicle speed is lower than the specified maximum vehicle speed.
* No emergency signals are currently active.


Starting the engine
-------------------

Regardless of the chosen method, remember to hold the brake during engine start
request. If the brake is not depressed at the time the engine starter relay is
triggered, the engine starter will not be energized.

In regular mode
^^^^^^^^^^^^^^^

If ignition has been switched on using the key and the drive-by-wire switch was
set to enabled at that time, the engine can be started normally by turning the
key further clockwise to run the engine starter as in most passenger vehicles.

In drive-by-wire mode
^^^^^^^^^^^^^^^^^^^^^

If, on the other hand, the ignition has been switched on using the drive-by-wire
(the only possible mode when the drive-by-wire switch is on at the first
ignition attempt), the engine may only be started via:

1. The engine start button on the gamepad controller.
2. A terminal command request from the main menu.