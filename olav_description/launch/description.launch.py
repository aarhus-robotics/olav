#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################
"""Robot description launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Input files
    # -----------
    # > Parse robot description file
    urdf = Path(
        get_package_share_path('olav_description') /
        "urdf/descriptions/olav.urdf").as_posix()
    with open(urdf, "r", encoding="UTF-8") as urdf_file:
        robot_description = urdf_file.read()

    # Launch arguments
    # ----------------
    # > Declare namespace launch argument
    namespace_argument = DeclareLaunchArgument("namespace",
                                               default_value="olav")

    # > Declare parameter overrides launch argument
    parameters_overrides_argument = DeclareLaunchArgument(
        "parameters_overrides",
        default_value=Path(
            get_package_share_path("olav_launch") /
            "config/parameters/overrides_defaults.yaml").as_posix())

    # > Declare log level launch argument
    log_level_argument = DeclareLaunchArgument("log_level",
                                               default_value="INFO")

    # Nodes
    # -----
    # > Robot state publisher node
    robot_state_publisher = Node(
        namespace=LaunchConfiguration("namespace"),
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[
            urdf, "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_description") /
                "config/parameters/robot_state_publisher_node_defaults.yaml").
            as_posix(),
            {
                "robot_description": robot_description,
            },
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("joint_states", "model/joints/states"),
            # > Publishers
            ("robot_description", "model/description"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Joint state publisher node
    joint_state_publisher = Node(
        namespace=LaunchConfiguration("namespace"),
        name="joint_state_publisher",
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_description") /
                "config/parameters/joint_state_publisher_node_defaults.yaml").
            as_posix(),
            {
                "robot_description": robot_description
            },
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("robot_description", "model/description"),
            # > Publishers
            ("joint_states", "model/joints/states"),
            ("front_wheels", "model/joints/steering"),
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
        robot_state_publisher,
        joint_state_publisher,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
