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
    # > LIO-SAM IMU pre-integration node.
    lio_sam_imu_preintegration_node = Node(
        namespace=LaunchConfiguration("namespace"),
        # See GitHub issue:
        # imuPreintegration node duplication problem on ros2 humble #434
        # https://github.com/TixiaoShan/LIO-SAM/issues/434
        #name="lio_sam_imu_preintegration",
        package="lio_sam",
        #prefix="konsole -e gdb -ex=r --args",
        executable="lio_sam_imuPreintegration",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_launch") /
                "config/parameters/lio_sam_nodes_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > LIO-SAM image projection node.
    lio_sam_image_projection_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="lio_sam_image_projection",
        package="lio_sam",
        #prefix="konsole -e gdb -ex=r --args",
        executable="lio_sam_imageProjection",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_launch") /
                "config/parameters/lio_sam_nodes_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > LIO-SAM feature extraction node.
    lio_sam_feature_extraction_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="lio_sam_feature_extraction",
        package="lio_sam",
        #prefix="konsole -e gdb -ex=r --args",
        executable="lio_sam_featureExtraction",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_launch") /
                "config/parameters/lio_sam_nodes_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > LIO-SAM map optimization node.
    lio_sam_map_optimization_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="lio_sam_map_optimization",
        package="lio_sam",
        #prefix="konsole -e gdb -ex=r --args",
        executable="lio_sam_mapOptimization",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_launch") /
                "config/parameters/lio_sam_nodes_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[],
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
        lio_sam_imu_preintegration_node,
        lio_sam_image_projection_node,
        lio_sam_feature_extraction_node,
        lio_sam_map_optimization_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
