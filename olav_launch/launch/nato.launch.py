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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


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
    # > Heartbeat node
    heartbeat_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="heart",
        package="navi_core",
        #prefix="konsole -e gdb -ex=r --args",
        executable="navi_core_heart_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("navi_core") /
                "config/parameters/heart_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            ("heartbeat", "multiplexer/in/heartbeat"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # Launch descriptions
    # -------------------
    nato_stack_group_action = GroupAction(actions=[
        PushRosNamespace("olav"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(
                    get_package_share_path("avt_341") /
                    "launch/avt_341.launch.py").as_posix()),
            launch_arguments={
                "namespace": "autonomy",
                "topic/in/odometry":
                "/olav/sensors/inertial_navigation_system/filter/odometry",
                "topic/in/steering_angle": "/olav/sensors/steering/angle",
                "topic/in/lidar": "/olav/sensors/lidar/points",
                "topic/in/camera": "/olav/sensors/roof_camera/image/raw",
                "topic/out/drive": "/olav/multiplexer/in/drive",
            }.items()),
    ], )

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Group actions
        heartbeat_node,
        nato_stack_group_action
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
