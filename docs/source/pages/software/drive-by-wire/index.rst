Drive-by-wire
=============

Overview
--------

The drive-by-wire node provides an interface to send commands and read the state of the drive-by-wire system. It
connects via Modbus TCP to the Wago PLC on the vehicle and allows direct access to the registers controlling the
actuators and triggers of the drive-by-wire system.

The drive-by-wire node requires a working odometer in order to run. This is necessary to close the loop for operations
such as engine start and gear shifts.  The drive-by-wire node checks for valid odometer outputs (engine and vehicle
speeds) before enabling normal operation.

There node runs a background health check on all systems at the same frequency of PLC communication to ensure all
systems are operational. The health check ensures information from the odometer (engine and vehicle speeds) has been
made available since the last drive-by-wire reset and that it is being published at the minimum specified rate, defined
by the user. By default, the minimum health check rate matches the PLC write rate.

The node checks for limit operating conditions for the engine speed and longitudinal vehicle speed. When the specified
limits are exceeded, the vehicle enters emergency state and stops, requiring manual user interaction to resume by-wire
or autonomous operation. By default, the engine limits are set to redline and the longitudinal speed limits are set to
15 km/h.

ROS Interface
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
^^^^^^^^^^^^^

.. csv-table:: Publishers
   :file: publishers.csv

Services
^^^^^^^^

.. csv-table:: Services
   :file: services.csv

Actions
^^^^^^^

.. csv-table:: Actions
   :file: actions.csv

Clients
^^^^^^^

.. csv-table:: Clients
   :file: clients.csv

Development notes
-----------------

The node automatically attemps connection in the background. During each healthcheck, the current connection state is
updated. All parallel threads check for a valid connection before thread execution.

Healthchecks are performed at a rate specified by the user through the `rates.healthcheck` parameter. The healthcheck
include the status of the PLC connection, the presence of a valid and recent odometry and engine speed message, the
presence of a user-acknowledgement to autonomous operation since the last interface reset and, finally the presence of a
valid and recent heartbeat from an upstream control authority.

The node accepts throttle, brake and steering efforts from the upstream control stack or from manually issued user
commands.