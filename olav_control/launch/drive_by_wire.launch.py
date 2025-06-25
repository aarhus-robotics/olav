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
    # > Drive-by-wire interface node
    drive_by_wire_interface_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="drive_by_wire_interface",
        package="olav_control",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_control_drive_by_wire_interface_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_control") /
                "config/parameters/drive_by_wire_interface_node_defaults.yaml").
            as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("feedback/engine/speed", "sensors/powertrain/engine/speed"),
            ("feedback/odometry", "sensors/powertrain/tachometer/odometry"),
            #("signals/heartbeat", "signals/heartbeat"),
            #("controls/tbs", "controls/tbs"),
            # > Publishers
            #("signals/drive_by_wire", "signals/drive_by_wire"),
            #("signals/estop", "signals/estop"),
            #("sensors/steering/angle", "sensors/steering/angle"),
            #("model/joints/steering", "model/joints/steering"),
            ("status", "drive_by_wire/status"),
            #("/diagnostics", "/diagnostics"),
            # > Services servers
            ("ready", "drive_by_wire/ready"),
            ("set_control_mode", "drive_by_wire/set_control_mode"),
            ("set_ignition", "drive_by_wire/set_ignition"),
            ("cycle_ignition", "drive_by_wire/cycle_ignition"),
            ("start_engine", "drive_by_wire/start_engine"),
            ("emergency_stop", "drive_by_wire/emergency_stop"),
            ("set_steering_pid_gains", "drive_by_wire/set_steering_pid_gains"),
            ("shift_gear_up", "drive_by_wire/shift_gear_up"),
            ("shift_gear_down", "drive_by_wire/shift_gear_down"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Powertrain interface node
    powertrain_interface_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="powertrain_interface",
        package="olav_sensors",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_sensors_powertrain_interface_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_sensors") /
                "config/parameters/powertrain_interface_node_defaults.yaml").
            as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Publishers
            ("engine/speed", "sensors/powertrain/engine/speed"),
            ("tachometer/speed", "sensors/powertrain/tachometer/speed"),
            ("tachometer/odometry", "sensors/powertrain/tachometer/odometry"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Diagnostic aggregator node
    diagnostic_aggregator_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="diagnostic_aggregator",
        package="diagnostic_aggregator",
        #prefix="konsole -e gdb -ex=r --args",
        executable="aggregator_node",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            Path(
                get_package_share_path("olav_control") /
                "config/parameters/diagnostic_aggregator_node_defaults.yaml").
            as_posix(),
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
        drive_by_wire_interface_node,
        powertrain_interface_node,
        diagnostic_aggregator_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
