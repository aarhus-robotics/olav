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
    # > Detection node
    detection_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="object_detector",
        package="navi_perception",
        #prefix="konsole -e gdb -ex=r --args",
        executable="navi_perception_object_detector_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("navi_perception") /
                "config/parameters/object_detector_node_defaults.yaml"
            ).as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("image", "/olav/sensors/camera/top/rgb/image_raw"),
            # > Publishers
            ("detections/vision", "perception/detector/detections"),
            ("detections/bounding_boxes", "perception/detector/bounding_boxes"),
            ("detections/image/raw", "perception/detector/image/raw"),
            ("info/vision", "perception/detector/info/vision"),
            ("info/label", "perception/detector/info/label"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Tracking node
    tracking_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="object_tracker",
        package="navi_perception",
        #prefix="konsole -e gdb -ex=r --args",
        executable="navi_perception__object_tracker_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("navi_perception") /
                "config/parameters/object_tracker_node_defaults.yaml"
            ).as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("detection_2d", "perception/detector/detections"),
            ("image", "/olav/sensors/camera/top/rgb/image_raw"),
            ("camera_info", "/olav/sensors/camera/top/camera_info"),
            ("points/input", "/olav/sensors/lidar/points"),
            # > Services
            ("set_target", "perception/tracker/set_target"),
            # > Publishers
            ("points/fov", "perception/tracker/points/fov"),
            ("points/roi", "perception/tracker/points/roi"),
            ("points/ground", "perception/tracker/points/ground"),
            ("points/cluster", "perception/tracker/points/cluster"),
            ("points/cropbox", "perception/tracker/points/cropbox"),
            ("detection_3d", "perception/tracker/detection_3d"),
            ("out_image", "perception/tracker/image"),
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
        detection_node,
        tracking_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
