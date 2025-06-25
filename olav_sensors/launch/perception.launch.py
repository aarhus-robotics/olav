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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode


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

    # Launch descriptions
    # -------------------
    # > Ouster LiDAR group action
    lidar_group_action = GroupAction(actions=[
        PushRosNamespace(LaunchConfiguration("namespace")),
        SetRemap("metadata", "sensors/lidar/metadata"),
        SetRemap("points", "sensors/lidar/points"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(
                    get_package_share_path("ouster_ros") /
                    "launch/driver.launch.py").as_posix()),
            launch_arguments={
                "ouster_ns":
                "",
                "os_driver_name":
                "lidar",
                "params_file":
                Path(
                    get_package_share_path("olav_sensors") /
                    "config/parameters/os_driver_node_defaults.yaml").as_posix(
                    ),
                "viz":
                'False',
            }.items()),
    ])

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
                name="camera_top_state_publisher",
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                parameters=[
                    {
                        "robot_description":
                        Command([
                            "xacro", " ",
                            Path(
                                get_package_share_path("depthai_descriptions") /
                                "urdf/depthai_descr.urdf.xacro").as_posix(),
                            " ", "camera_name:=", "camera_top", " ",
                            "camera_model:=", "OAK-D-POE", " ", "base_frame:=",
                            "camera_top_frame", " ", "parent_frame:=",
                            "camera_top_link", " ", "cam_pos_x:=", "0.0", " ",
                            "cam_pos_y:=", "0.0", " ", "cam_pos_z:=", "0.0",
                            " ", "cam_roll:=", "1.5708", " ", "cam_pitch:=",
                            "0.0", " ", "cam_yaw:=", "1.5708"
                        ])
                    },
                    LaunchConfiguration("parameters_overrides"),
                ],
                remappings=[
                    # > Subscriptions
                    ("robot_description", "model/description"),
                    ("joint_states", "model/joints/states"),
                ],
            ),
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="camera_top",
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                parameters=[
                    Path(
                        get_package_share_path("olav_sensors") /
                        "config/parameters/camera_top_node_defaults.yaml").
                    as_posix(),
                    LaunchConfiguration("parameters_overrides"),
                ],
                remappings=[
                    # > Subscriptions
                    ("camera_top/rgb/camera_info",
                     "sensors/camera/top/camera_info"),
                    ("camera_top/rgb/image_raw",
                     "sensors/camera/top/rgb/image_raw"),
                    ("camera_top/rgb/image_raw/compressed",
                     "sensors/camera/top/rgb/image_raw/compressed"),
                    ("camera_top/rgb/image_raw/compressedDepth",
                     "sensors/camera/top/rgb/image_raw/compressedDepth"),
                    ("camera_top/rgb/image_raw/theora",
                     "sensors/camera/top/rgb/image_raw/theora"),
                ],
            ),
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="camera_front_state_publisher",
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                parameters=[
                    {
                        "robot_description":
                        Command([
                            "xacro", " ",
                            Path(
                                get_package_share_path("depthai_descriptions") /
                                "urdf/depthai_descr.urdf.xacro").as_posix(),
                            " ", "camera_name:=", "camera_front", " ",
                            "camera_model:=", "OAK-D-PRO", " ", "base_frame:=",
                            "camera_front_frame", " ", "parent_frame:=",
                            "camera_front_link", " ", "cam_pos_x:=", "0.0", " ",
                            "cam_pos_y:=", "0.0", " ", "cam_pos_z:=", "0.0",
                            " ", "cam_roll:=", "1.5708", " ", "cam_pitch:=",
                            "0.0", " ", "cam_yaw:=", "1.5708"
                        ])
                    },
                    LaunchConfiguration("parameters_overrides"),
                ],
                remappings=[
                    # > Subscriptions
                    ("robot_description", "model/description"),
                    ("joint_states", "model/joints/states"),
                ],
            ),
            ComposableNode(
                namespace=LaunchConfiguration("namespace"),
                name="camera_front",
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                parameters=[
                    Path(
                        get_package_share_path("olav_sensors") /
                        "config/parameters/camera_front_node_defaults.yaml").
                    as_posix(),
                    LaunchConfiguration("parameters_overrides"),
                ],
                remappings=[
                    # > Subscriptions
                    ("camera_front/imu/data", "sensors/camera/front/imu/data"),
                    ("camera_front/imu/mag", "sensors/camera/front/imu/mag"),
                    ("camera_front/rgb/camera_info",
                     "sensors/camera/front/rgb/camera_info"),
                    ("camera_front/rgb/image_raw",
                     "sensors/camera/front/rgb/image_raw"),
                    ("camera_front/rgb/image_raw/compressed",
                     "sensors/camera/front/rgb/image_raw/compressed"),
                    ("camera_front/rgb/image_raw/compressedDepth",
                     "sensors/camera/front/rgb/image_raw/compressedDepth"),
                    ("camera_front/rgb/image_raw/theora",
                     "sensors/camera/front/rgb/image_raw/theora"),
                    ("camera_front/stereo/camera_info",
                     "sensors/camera/front/stereo/camera_info"),
                    ("camera_front/stereo/image_raw",
                     "sensors/camera/front/stereo/image_raw"),
                    ("camera_front/stereo/image_raw/compressed",
                     "sensors/camera/front/stereo/image_raw/compressed"),
                    ("camera_front/stereo/image_raw/compressedDepth",
                     "sensors/camera/front/stereo/image_raw/compressedDepth"),
                    ("camera_front/stereo/image_raw/theora",
                     "sensors/camera/front/stereo/image_raw/theora"),
                ],
            ),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
    )

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Group actions
        lidar_group_action,
        # > Nodes
        poe_cameras_container,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
