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
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():

    # Launch arguments
    # ----------------
    # > Declare namespace launch argument
    namespace_argument = DeclareLaunchArgument("namespace", default_value="olav")

    # > Declare parameter overrides launch argument
    parameters_overrides_argument = DeclareLaunchArgument("parameters_overrides",
                                                          default_value=Path(
                                                              get_package_share_path("olav_launch") /
                                                              "config/parameters/overrides_defaults.yaml").as_posix())

    # > Declare log level launch argument
    log_level_argument = DeclareLaunchArgument("log_level", default_value="WARN")

    # Nodes
    # -----
    # > GNSS/INS node
    inertial_navigation_system_node = LifecycleNode(
        namespace=LaunchConfiguration("namespace"),
        name="inertial_navigation_system",
        package="microstrain_inertial_driver",
        #prefix="konsole -e gdb -ex=r --args",
        executable="microstrain_inertial_driver_node",
        parameters=[
            Path(get_package_share_path("olav_sensors"),
                 "config/parameters/microstrain_inertial_driver_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Publishers
            ("ekf/status", "sensors/inertial_navigation_system/device/device"),
            ("ekf/imu/data", "sensors/inertial_navigation_system/filter/imu"),
            ("ekf/llh_position", "sensors/inertial_navigation_system/filter/coordinates"),
            ("ekf/odometry_map", "sensors/inertial_navigation_system/filter/odometry"),
            ("gnss_1/llh_position", "sensors/inertial_navigation_system/antenna/left/coordinates"),
            ("gnss_1/time", "sensors/inertial_navigation_system/antenna/left/time"),
            #("gnss_1/velocity", "sensors/inertial_navigation_system/antenna/left/velocity"),
            ("gnss_2/llh_position", "sensors/inertial_navigation_system/antenna/right/coordinates"),
            #("gnss_2/velocity", "sensors/inertial_navigation_system/antenna/right/velocity"),
            ("imu/data", "sensors/inertial_navigation_system/imu/data"),
            #("imu/pressure", "sensors/inertial_navigation_system/imu/pressure"),
            ("mip/ekf/aiding_measurement_summary", "sensors/inertial_navigation_system/filter/aiding"),
            #("mip/ekf/dual_antenna_heading", "sensors/inertial_navigation_system/antenna/dual/heading"),
            ("nmea", "sensors/inertial_navigation_system/rtk/nmea"),
            ("rtcm", "sensors/inertial_navigation_system/rtk/rtcm"),
            ("mip/gnss_1/fix_info", "sensors/inertial_navigation_system/antenna/left/status/fix"),
            ("mip/gnss_1/sbas_info", "sensors/inertial_navigation_system/antenna/left/status/sbas"),
            ("mip/gnss_2/fix_info", "sensors/inertial_navigation_system/antenna/right/status/fix"),
            ("mip/gnss_2/sbas_info", "sensors/inertial_navigation_system/antenna/right/status/sbas"),
            ("mip/gnss_corrections/rtk_corrections_status", "sensors/inertial_navigation_system/status/rtk"),
            ("mip/ekf/status", "sensors/inertial_navigation_system/filter/status"),
            ("mip/ekf/gnss_dual_antenna_status", "sensors/inertial_navigation_system/antenna/dual/status"),
            ("mip/ekf/gnss_position_aiding_status", "sensors/inertial_navigation_system/status/aiding"),
            ("mip/ekf/multi_antenna_offset_correction",
             "sensors/inertial_navigation_system/antenna/dual/offset_correction"),
            # > Services
            ("mip/base/get_device_information", "sensors/inertial_navigation_system/device/get_device_information"),
            ("mip/ekf/reset", "sensors/inertial_navigation_system/filter/reset"),
            ("mip/three_dm/capture_gyro_bias", "sensors/inertial_navigation_system/capture_gyro_bias"),
            ("mip/three_dm/device_settings/load", "sensors/inertial_navigation_system/settings/load"),
            ("mip/three_dm/device_settings/save", "sensors/inertial_navigation_system/settings/save"),
            ("mip/three_dm/gpio_state/read", "sensors/inertial_navigation_system/gpio/read"),
            ("mip/three_dm/gpio_state/write", "sensors/inertial_navigation_system/gpio/write"),
            ("raw_file_config/aux/read", "sensors/inertial_navigation_system/config/aux/read"),
            ("raw_file_config/aux/write", "sensors/inertial_navigation_system/config/aux/write"),
            ("raw_file_config/main/read", "sensors/inertial_navigation_system/config/main/read"),
            ("raw_file_config/main/write", "sensors/inertial_navigation_system/config/main/write"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        #on_exit=Shutdown(),
        respawn=True,
    )

    # > NTRIP client node
    ntrip_client_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="ntrip_client",
        package="ntrip_client",
        #prefix="konsole -e gdb -ex=r --args",
        executable="ntrip_ros.py",
        parameters=[
            Path(get_package_share_path("olav_sensors") /
                 "config/parameters/ntrip_client_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Publishers
            ("nmea", "sensors/inertial_navigation_system/rtk/nmea"),
            ("rtcm", "sensors/inertial_navigation_system/rtk/rtcm"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        #on_exit=Shutdown(),
        respawn=True,
    )

    # > Static transform publisher node
    static_transform_publisher_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="map_to_local_map_static_transform",
        package="tf2_ros",
        #prefix="konsole -e gdb -ex=r --args",
        executable="static_transform_publisher",
        parameters=[
            Path(get_package_share_path("olav_sensors") /
                 "config/parameters/static_transform_publisher.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "local_map"],
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
        inertial_navigation_system_node,
        ntrip_client_node,
        static_transform_publisher_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
