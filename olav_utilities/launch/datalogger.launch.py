#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################
"""Launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    # ----------------
    namespace_argument = DeclareLaunchArgument("namespace",
                                               default_value="olav")

    parameters_overrides_argument = DeclareLaunchArgument(
        "parameters_overrides",
        default_value=Path(
            get_package_share_path("olav_launch") /
            "config/parameters/overrides_defaults.yaml").as_posix())

    log_level_argument = DeclareLaunchArgument("log_level",
                                               default_value="INFO")

    # Nodes
    # -----
    # > Datalogger node
    datalogger_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="datalogger_node",
        package="olav_utilities",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_utilities_datalogger_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_utilities") /
                "config/parameters/datalogger_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Services
            ("start", "datalogger/start"),
            ("stop", "datalogger/stop"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Nodes
        datalogger_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
