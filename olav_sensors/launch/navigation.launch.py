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
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import LifecycleNode, Node

gnss_ins_node = LifecycleNode(
    namespace="olav",
    package="microstrain_inertial_driver",
    executable="microstrain_inertial_driver_node",
    name="ins",
    parameters=[
        Path(get_package_share_path("olav_sensors"),
             "config/parameters/microstrain_inertial_driver_node_defaults.yaml").
        as_posix(),
    ],
    remappings=[
        ("ekf/dual_antenna_heading", "ins/antenna/dual/heading"),
        ("ekf/imu/data", "ins/filter/imu"),
        ("ekf/llh_position", "ins/filter/coordinates"),
        ("ekf/odometry_map", "ins/filter/odometry"),
        ("ekf/status", "ins/status/device"),
        ("gnss_1/llh_position", "ins/antenna/left/coordinates"),
        ("gnss_1/time", "ins/antenna/left/time"),
        ("gnss_1/velocity", "ins/antenna/left/velocity"),
        ("gnss_2/llh_position", "ins/antenna/right/coordinates"),
        ("gnss_2/velocity", "ins/antenna/right/velocity"),
        ("imu/data", "ins/imu/data"),
        ("imu/pressure", "ins/imu/pressure"),
        ("mip/ekf/aiding_measurement_summary", "ins/filter/aiding"),
        ("mip/ekf/gnss_dual_antenna_status", "ins/antenna/dual/status"),
        ("mip/ekf/gnss_position_aiding_status", "ins/status/aiding"),
        ("mip/ekf/multi_antenna_offset_correction",
         "ins/antenna/dual/offset_correction"),
        ("mip/ekf/status", "ins/filter/status"),
        ("mip/gnss_1/fix_info", "ins/antenna/left/status/fix"),
        ("mip/gnss_1/sbas_info", "ins/antenna/left/status/sbas"),
        ("mip/gnss_2/fix_info", "ins/antenna/right/status/fix"),
        ("mip/gnss_2/sbas_info", "ins/antenna/right/status/sbas"),
        ("mip/gnss_corrections/rtk_corrections_status", "ins/status/rtk"),
        ("nmea", "ins/rtk/nmea"),
        ("rtcm", "ins/rtk/rtcm"),
    ],
    on_exit=Shutdown(),
    emulate_tty=True,
    output="both",
)

ntrip_client_node = Node(
    namespace="olav",
    package="ntrip_client",
    executable="ntrip_ros.py",
    name="ntrip_client",
    parameters=[
        Path(
            get_package_share_path("olav_sensors") /
            "config/parameters/ntrip_client_node_defaults.yaml").as_posix(),
    ],
    remappings=[
        ("nmea", "ins/rtk/nmea"),
        ("rtcm", "ins/rtk/rtcm"),
    ],
    on_exit=Shutdown(),
    emulate_tty=True,
    output="both",
)

ntpd_driver_node = Node(
    namespace="olav",
    package="ntpd_driver",
    executable="shm_driver",
    parameters=[
        Path(
            get_package_share_path("olav_sensors") /
            "config/parameters/ntpd_driver_node_defaults.yaml").as_posix(),
    ],
    remappings=[
        ("time_ref", "ins/antenna/left/time"),
    ],
    on_exit=Shutdown(),
    emulate_tty=True,
    output="both",
)

static_transform_publisher_node = Node(
    namespace="olav",
    package="tf2_ros",
    executable="static_transform_publisher",
    name="map_to_local_map_static_transform",
    arguments=[
        "-0.76900000", "-8.79000000", "0.00000000", "0.00000000", "0.00000000",
        "-0.27925268", "map", "local_map"
    ],
    on_exit=Shutdown(),
    emulate_tty=True,
    output="both",
)


def generate_launch_description():
    """Generate the launch description."""

    return LaunchDescription([
        # Nodes
        gnss_ins_node,
        #ntrip_client_node,
        ntpd_driver_node,
        static_transform_publisher_node,
    ])
