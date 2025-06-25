# Building

OLAV depends on a number of external libraries. Some of these libraries are part of the standard ROS distribution and
may be automatically installed using the `rosdep` tool. Other libraries must be manually installed by the user and their
installation steps depend on the platform OLAV is being deployed to. In the following, we assume OLAV is being deployed
on Ubuntu 22.04, the Tier 1 platform for ROS Humble.

## Clone the repository

First, clone the repository to a known path. In the following, we will use
`~/Repositories/github.com/aarhus-robotics/olav` as clone path.

```shell
mkdir -p ~/Repositories/github.com/aarhus-robotics
cd ~/Repositories/github.com/aarhus-robotics
git clone https://github.com/aarhus-robotics/olav.git
```

Then, symlink the repository to the `src` folder of your ROS workspace:

```shell
mkdir -p ~/ROS/src/aarhus-robotics
cd ~/ROS/src/aarhus-robotics
ln -s ~/Repositories/github.com/aarhus-robotics/olav olav
```

## Installing ROS dependencies via `rosdep`

If you have not used `rosdep` before, set up `rosdep` by initializing it and
updating its sources:

```shell
sudo rosdep init
rosdep update
```

Then, from the root of your ROS workspace, run the following command to
automatically retrieve and install all ROS dependencies.

```shell
rosdep install --from-paths src/aarhus-robotics/olav
```

## Installing the serial library

The ``wjwood/serial`` library is used to interface with the powertrain
microcontroller. Unfortunately, the library depends on the ``catkin`` build
system which is part of the legacy ROS distribution. There is, however, a fork
available on GitHub under the repository `RoverRobotics-forks/serial-ros2
<https://github.com/RoverRobotics-forks/serial-ros2>`_.  Assuming your local ROS
workspace is located under `~/ros2_ws`, the library may be compile via `colcon`
as follows:

```shell
cd ~/ros2_ws
mkdir -p src/RoverRobotics-forks
git clone https://github.com/RoverRobotics-forks/serial-ros2 ~/ros2_ws/src/serial-ros2
colcon build --packages-select serial
```

## Installing the modbus library

The `libmodbus` library is used to interface with the drive-by-wire controller.

```shell
sudo apt install --yes libmodbus-dev
```

## Other ROS dependencies

Some additional dependencies are required to run OLAV:

```shell
sudo apt install --yes \
  libboost-dev \
  ros-humble-diagnostic-updater
```

## PlotJuggler

The OLAV operator laptop uses PlotJuggler to display real-time plots from the
vehicle sensors. Upon launch, the PlotJuggler node loads a predefined layout for
convenience. Unfortunately, the version of PlotJuggler included in the ROS
Humble distribution crashes when loading layouts with filters due to a
well-known bug. To avoid this issue, one can instead compile the latest version
of PlotJuggler from sources in the local ROS workspace. Assuming the local ROS
workspace is located under `~/ROS`:

```shell
cd ~/ROS
mkdir -p src/facontidavide
git clone https://github.com/facontidavide/PlotJuggler src/facontidavide/PlotJuggler
colcon build
```
