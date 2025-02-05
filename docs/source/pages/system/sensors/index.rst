Sensors
=======

Luxonis cameras
---------------

Install the DepthAI packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: shell
  
  sudo apt install --yes \
    ros2-humble-depthai
    ros2-humble-depthai-ros
Update the bootloader on DepthAI cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Luxonis provides a set of Python utilities to update the bootloader on their cameras. A bootloader upload may be necessary to suppress any update warnings on the ROS driver startup and have access to all latest features exposed through the configuration file.

Clone the DepthAI Python repository:

.. code-block:: shell

    mkdir -p ~/Repositories/github.com/luxonis
    cd ~/Repositories/github.com/luxonis
    git clone https://github.com/luxonis/depthai-python
    cd depthai-python


Create a new Python virtual environment under `./.venv`, activate it and install all DepthAI Python dependencies in the sandboxed environment:

.. code-block:: shell
    python3 -m venv .venv
    cd utilities
    python3 ./install_requirements.py

Finally, launch the `device_manager.py` GUI and follow the video instructions at `this link <https://www.youtube.com/watch?v=439qZda-pWk>`_ to update the bootloader on the device.

.. code-block::shell

    python3 ./device_manager.py

Backup camera
-------------

OLAV is fitted with a infrared, pan-tilt-zoom (PTZ) backup camera to aid maneuvering in reverse and to provide
surveillance capabilities.

RLC-823A
UID: 95270005PC8T1YA9

Inertial navigation system
--------------------------

The LORD Microstrain GQ7 provides OLAV with a GNSS-aided dual-antenna, RTK enabled inertial navigation system.

Software installation
^^^^^^^^^^^^^^^^^^^^^

To run the LORD Microstrain GQ7 with OLAV, you will need the `microstrain_inertial_driver` package for ROS2. The stable
version of the driver is included in the ROS Humble distribution, and the driver and supporting RQT packages may be
installed via the APT package manager:

.. code-block:: shell

    sudo apt install --yes \
        ros-humble-microstrain-inertial-driver
        ros-humble-microstrain-inertial-msgs

If you are on a computer with access to a desktop environment, also install the matchin RQt plugin.

.. code-block:: shell

    sudo apt install --yes ros-humble-microstrain-inertial-rqt


Alternatively, the latest version of the driver may be installed from sources.  Assuming the ROS workspace where OLAV is
installed resides under `~/ros2_ws`, the installation of the `microstrain_inertial_driver` package in the local
workspace is as follows:

.. code-block:: shell

    cd ~/ros2_ws/src
    git clone https://github.com/LORD-MicroStrain/microstrain_inertial --branch
    ros2
    cd ../
    colcon build \
        --packages-select \
            microstrain_inertial_driver \
            microstrain_inertial_msgs \
            microstrain_inertial_rqt
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release 

UDEV rules
^^^^^^^^^^

.. code-block::

    # LORD Microstrain GQ7
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="0000 6284.170606", MODE="0666", SYMLINK+="gnss_ins_aux"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="2-00 6284.170606", MODE="0666", SYMLINK+="gnss_ins_main"

Then refresh:

.. code-block::

    udevadm control --reload-rules
    udevadm trigger


Configure the time sync server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: shell

  sudo apt install ros-humble-ntpd-driver

Configure the NTRIP client
^^^^^^^^^^^^^^^^^^^^^^^^^^

Install the required dependencies:

.. code-block:: shell
  sudo apt install --yes ros-humble-mavros-msgs ros-humble-rtcm-msgs

.. _Powertrain microcontroller interface:

Powertrain microcontroller interface
------------------------------------

System overview
^^^^^^^^^^^^^^^

.. image:: arduino-micro.png

OLAV provides a minimal interface to obtain powertrain and odometry data for navigation, safety and debugging purposes.
The data is streamed via USB by an Arduino Micro microcontroller located in the drive-by-wire system box under the
passenger seat.

The microcontroller is connected to the main computing unit via USB 2.0. The controller can be accessed using the ROS
node `powertrain_interface_node` or, alternatively and strictly for debugging purposes, using the `odometer` module in
`pyolav`.

For more information, refer to the `Powertrain interface node`_ documentation.

Accessing powertrain interface data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The microcontroller connects via USB serial protocol with a baudrate of `9600`.  The following line is passed through
the serial port:

.. code-block::

    UGV:{engine_rpm},{front_axle_cdps},{emergency_stop}

Where:

* `{engine_rpm}` is the raw engine RPM reading as shown on the vehicle dashboard.
* `{front_axle_cdps}` is the averaged front axle speed in centidegrees per seconds.
* `{emergency_stop}` is an emergency flag raised by the odometer microcontroller in case of system malfunction.

Adding the udev rules
^^^^^^^^^^^^^^^^^^^^^

To ensure the powertrain microcontroller is consistently assigned the same device path, a userspace `/dev` (udev) rule
matching the attributes of the USB device must be specified on the OLAV server.

To do so, create a new file under `/etc/udev/rules.d/80-powertrain-microcontroller.rules` with the following content:

.. code-block:: shell

    # Powertrain microcontroller rule
    SUBSYSTEM=="tty", ATTRS{idVendor}=="", ATTRS{idProduct}=="", ATTRS{serial}=="", MODE="0666", SYMLINK+=powertrain"

Then, refresh the `/dev` userspace via `udevadm`:

.. code-block:: shell

    sudo udevadm control --reload-rules
    sudo udevadm trigger

Ouster OS-1 LiDAR
-----------------

Configure the `ouster-ros` package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install the required dependencies:

.. code-block:: shell

  sudo apt install --yes \
    libpcab-dev
    libtins-dev
    libflatbuffers-dev
    libglfw3-dev

Clone the repository `ros2` branch and initialize and update all submodules (required to retrieve the correct version of Ouster SDK):

.. code-block:: shell

    git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

Setting up PTP synchronization
------------------------------

Check the network interface capabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, install `ethtool` to check whether the install network card is capable of hardware PTP timestamping. You will need hardware PTP timestamp in order to accurately stamp sensor information and account for propagation delays. The network interfaces and switches in OLAV are all capable of hardware PTP timestamping.

.. code-block::
  sudo apt install --yes ethtool

Find the name of your primary interface by identifying the local network IP address using the `ip addr` command. Then, use `ethtool -T <interface-name>`. A network card capable of hardware PTP timestamping will return:

.. code-block:: shell

    Time stamping parameters for <interface-name>:
    Capabilities:
            hardware-transmit
            software-transmit
            hardware-receive
            software-receive
            software-system-clock
            hardware-raw-clock
    PTP Hardware Clock: 0
    Hardware Transmit Timestamp Modes:
            off
            on
    Hardware Receive Filter Modes:
            none
            all
            ptpv1-l4-sync
            ptpv1-l4-delay-req
            ptpv2-l4-sync
            ptpv2-l4-delay-req
            ptpv2-l2-sync
            ptpv2-l2-delay-req
            ptpv2-event
            ptpv2-sync
            ptpv2-delay-req

Install and configure ptpd
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: shell

    sudo apt install --yes linuxptp