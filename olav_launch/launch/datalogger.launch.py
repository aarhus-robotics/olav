#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################
"""Datalogger launch file."""

from os import getlogin
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch arguments
# ----------------
# > Declare namespace launch argument
namespace_argument = DeclareLaunchArgument("namespace", default_value="olav")

# > Declare parameter overrides launch argument
parameters_overrides_argument = DeclareLaunchArgument(
    "parameters_overrides",
    default_value=Path(get_package_share_path("olav_launch") / "config/parameters/overrides_defaults.yaml").as_posix())

# > Declare log level launch argument
log_level_argument = DeclareLaunchArgument("log_level", default_value="WARN")

# Nodes
# -----
# > Datalogger node
datalogger_node = Node(
    namespace=LaunchConfiguration("namespace"),
    name="datalogger",
    package="roto_utilities",
    executable="roto_utilities_datalogger_node",
    arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    parameters=[
        Path(get_package_share_path("olav_launch") / "config/parameters/datalogger_node_defaults.yaml").as_posix(),
        {
            "path": f"/home/{getlogin()}/ROS/bags",
        },
        LaunchConfiguration("parameters_overrides"),
    ],
    remappings=[
        # > Service servers
        ("start", "datalogger/start"),
        ("stop", "datalogger/stop"),
    ],
    emulate_tty=True,
    output={
        "both": ["screen", "own_log"],
    },
    on_exit=Shutdown(),
)


def generate_launch_description():
    """Generate the launch description."""

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Nodes
        datalogger_node,
    ])
