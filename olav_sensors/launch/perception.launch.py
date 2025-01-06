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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


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
                                               default_value="WARN")

    # Launch descriptions
    # -------------------
    # > Ouster LiDAR launch description
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            Path(
                get_package_share_path("ouster_ros") /
                "launch/driver.launch.py").as_posix()
        ]),
        launch_arguments={
            "ouster_ns":
            "olav",
            "os_driver_name":
            "lidar",
            "params_file":
            Path(
                get_package_share_path("olav_sensors") /
                "config/parameters/os_driver_node_defaults.yaml").as_posix(),
            "viz":
            'False',
        }.items())

    # Nodes
    # -----
    # > DepthAI cameras node container
    poe_cameras_container = ComposableNodeContainer(
        namespace=LaunchConfiguration("namespace"),
        name="poe_cameras_container",
        package="rclcpp_components",
        #prefix="konsole -e gdb -ex=r --args",
        executable="component_container",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        composable_node_descriptions=[
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="roof_camera_state_publisher",
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                parameters=[
                    {
                        "robot_description":
                        Command([
                            "xacro", " ",
                            Path(
                                get_package_share_path("depthai_descriptions")
                                / "urdf/depthai_descr.urdf.xacro").as_posix(),
                            " ", "camera_name:=", "roof_camera", " ",
                            "camera_model:=", "OAK-D-POE", " ", "base_frame:=",
                            "roof_camera_frame", " ", "parent_frame:=",
                            "roof_camera_link", " ", "cam_pos_x:=", "0.0", " ",
                            "cam_pos_y:=", "0.0", " ", "cam_pos_z:=", "0.0",
                            " ", "cam_roll:=", "1.5708", " ", "cam_pitch:=",
                            "0.0", " ", "cam_yaw:=", "1.5708"
                        ])
                    },
                    LaunchConfiguration("parameters_overrides"),
                ]),
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="roof_camera",
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                parameters=[
                    Path(
                        get_package_share_path("olav_sensors") /
                        "config/parameters/roof_camera_node_defaults.yaml").
                    as_posix(),
                    LaunchConfiguration("parameters_overrides"),
                ],
            ),
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="bumper_camera_state_publisher",
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                parameters=[
                    {
                        "robot_description":
                        Command([
                            "xacro", " ",
                            Path(
                                get_package_share_path("depthai_descriptions")
                                / "urdf/depthai_descr.urdf.xacro").as_posix(),
                            " ", "camera_name:=", "bumper_camera", " ",
                            "camera_model:=", "OAK-D-PRO", " ", "base_frame:=",
                            "bumper_camera_frame", " ", "parent_frame:=",
                            "bumper_camera_link", " ", "cam_pos_x:=", "0.0",
                            " ", "cam_pos_y:=", "0.0", " ", "cam_pos_z:=",
                            "0.0", " ", "cam_roll:=", "1.5708", " ",
                            "cam_pitch:=", "0.0", " ", "cam_yaw:=", "1.5708"
                        ])
                    },
                    LaunchConfiguration("parameters_overrides"),
                ]),
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="bumper_camera",
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                parameters=[
                    Path(
                        get_package_share_path("olav_sensors") /
                        "config/parameters/bumper_camera_node_defaults.yaml").
                    as_posix(),
                    LaunchConfiguration("parameters_overrides"),
                ],
            ),
            #LoadComposableNodes(
            #    target_container=f"poe_cameras_container",
            #    composable_node_descriptions=[
            #        ComposableNode(
            #            package="depth_image_proc",
            #            plugin="depth_image_proc::PointCloudXyzrgbNode",
            #            name="point_cloud_xyzrgb_node",
            #            namespace=LaunchConfiguration("namespace"),
            #            remappings=[
            #                (
            #                    "depth_registered/image_rect",
            #                    f"{name}/{stereo_sens_name}/{depth_topic_suffix}",
            #                ),
            #                (
            #                    "rgb/image_rect_color",
            #                    f"{name}/{color_sens_name}/image_rect",
            #                ),
            #                ("rgb/camera_info",
            #                 f"{name}/{color_sens_name}/camera_info"),
            #                ("points", points_topic_name),
            #            ],
            #        ),
            #    ],
            #),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
    )

    resize_nodes = LoadComposableNodes(
        target_container="poe_cameras_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_color_node",
                namespace=LaunchConfiguration("namespace"),
                remappings=[
                    ("image", "roof_camera/rgb/image_raw"),
                    ("camera_info", "roof_camera/rgb/camera_info"),
                    ("image_rect", "roof_camera/rgb/image_rect"),
                    (
                        "image_rect/compressed",
                        "roof_camera/rgb/image_rect/compressed",
                    ),
                    (
                        "image_rect/compressedDepth",
                        "roof_camera/rgb/image_rect/compressedDepth",
                    ),
                    (
                        "image_rect/theora",
                        "roof_camera/rgb/image_rect/theora",
                    ),
                ],
            )
        ],
    )

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Launch configurations
        lidar_launch,
        # > Nodes
        poe_cameras_container,
        resize_nodes,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
