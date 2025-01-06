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
    gnss_ins_node = LifecycleNode(
        namespace=LaunchConfiguration("namespace"),
        name="ins",
        package="microstrain_inertial_driver",
        #prefix="konsole -e gdb -ex=r --args",
        executable="microstrain_inertial_driver_node",
        parameters=[
            Path(get_package_share_path("olav_sensors"),
                 "config/parameters/microstrain_inertial_driver_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            ("ekf/status", "sensors/ins/status/device"),
            ("ekf/imu/data", "sensors/ins/filter/imu"),
            ("ekf/llh_position", "sensors/ins/filter/coordinates"),
            ("ekf/odometry_map", "sensors/ins/filter/odometry"),
            #("mip/ekf/dual_antenna_heading", "sensors/ins/antenna/dual/heading"),
            ("gnss_1/llh_position", "sensors/ins/antenna/left/coordinates"),
            ("gnss_1/time", "sensors/ins/antenna/left/time"),
            #("gnss_1/velocity", "sensors/ins/antenna/left/velocity"),
            ("gnss_2/llh_position", "sensors/ins/antenna/right/coordinates"),
            #("gnss_2/velocity", "sensors/ins/antenna/right/velocity"),
            ("imu/data", "sensors/ins/imu/data"),
            #("imu/pressure", "sensors/ins/imu/pressure"),
            ("mip/ekf/aiding_measurement_summary", "sensors/ins/filter/aiding"),
            # > NMEA/RTK topics
            ("nmea", "sensors/ins/rtk/nmea"),
            ("rtcm", "sensors/ins/rtk/rtcm"),
            # > Debug topics
            ("mip/gnss_1/fix_info", "sensors/ins/antenna/left/status/fix"),
            ("mip/gnss_1/sbas_info", "sensors/ins/antenna/left/status/sbas"),
            ("mip/gnss_2/fix_info", "sensors/ins/antenna/right/status/fix"),
            ("mip/gnss_2/sbas_info", "sensors/ins/antenna/right/status/sbas"),
            ("mip/gnss_corrections/rtk_corrections_status", "sensors/ins/status/rtk"),
            ("mip/ekf/status", "sensors/ins/filter/status"),
            ("mip/ekf/gnss_dual_antenna_status", "sensors/ins/antenna/dual/status"),
            ("mip/ekf/gnss_position_aiding_status", "sensors/ins/status/aiding"),
            ("mip/ekf/multi_antenna_offset_correction", "sensors/ins/antenna/dual/offset_correction"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
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
            ("nmea", "sensors/ins/rtk/nmea"),
            ("rtcm", "sensors/ins/rtk/rtcm"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > NTPD driver node
    ntpd_driver_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="ntpd_driver",
        package="ntpd_driver",
        #prefix="konsole -e gdb -ex=r --args",
        executable="shm_driver",
        parameters=[
            Path(get_package_share_path("olav_sensors") /
                 "config/parameters/ntpd_driver_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            ("time_ref", "sensors/ins/antenna/left/time"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
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
        gnss_ins_node,
        ntrip_client_node,
        ntpd_driver_node,
        static_transform_publisher_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
