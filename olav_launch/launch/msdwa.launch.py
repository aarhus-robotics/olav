#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( N ) ( A ) ( V ) ( I )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#           NAVI: Autonomous Navigation Stack for Ground Vehicles           #
#############################################################################
"""Launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""

    # Launch arguments
    # ----------------
    # > Declare namespace launch argument
    namespace_argument = DeclareLaunchArgument("namespace", default_value="navi")

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
    # > Heart node
    heart_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="heart",
        package="olav_utilities",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_utilities_heart_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_utilities") /
                "config/parameters/heart_node_defaults.yaml"
            ).as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Publishers
            ("heartbeat", "/olav/signals/heartbeat"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > MSDWA planner node
    msdwa_planner_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="msdwa_planner",
        package="navi_planning",
        #prefix="konsole -e gdb -ex=r --args",
        executable="navi_planning_dwa_planner_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("navi_planning") /
                "config/parameters/msdwa_planner_node_defaults.yaml"
            ).as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("odometry", "/olav/sensors/inertial_navigation_system/filter/odometry"),
            ("goal", "planner/goal"),
            ("obstacles", "planner/obstacles"),

            ("steering_angle", "/olav/sensors/steering/angle"),
            # > Publishers
            ("path", "planner/path"),
            ("markers", "planner/markers"),
            ("drive", "/olav/controls/drive"),
            ("path", "planner/status"),
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
        heart_node,
        msdwa_planner_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
