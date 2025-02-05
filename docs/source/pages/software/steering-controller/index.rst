Steering controller
===================

The steering controller node is used to control the steering actuator in the drive-by-wire system and translate a
user-defined steering angle and steering rate to an appropriate effort actuator.

The steering controller implements a PID controller with integral windup control. The PID controller must be tuned
manually in conditions that match the operating scenario as closely as possible. ROS interface

ROS interface
-------------

Parameters
^^^^^^^^^^

.. csv-table:: Parameters
   :file: parameters.csv

Subscriptions
^^^^^^^^^^^^^

.. csv-table:: Subscriptions
   :file: subscriptions.csv

Publishers
^^^^^^^^^^

.. csv-table:: Publishers
   :file: publishers.csv

Services
^^^^^^^^

.. csv-table:: Services
   :file: services.csv