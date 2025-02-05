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
            #("controls/throttle", "controls/throttle"),
            #("controls/brake", "controls/brake"),
            #("controls/steering", "controls/steering"),
            # > Publishers
            #("signals/drive_by_wire", "signals/drive_by_wire"),
            #("signals/estop", "signals/estop"),
            #("sensors/steering/angle", "sensors/steering/angle"),
            #("model/joints/steering", "model/joints/steering"),
            ("status", "drive_by_wire/status"),
            #("/diagnostics", "/diagnostics"),
            # > Services servers
            ("ready", "drive_by_wire/ready"),
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

    # > Control multiplexer node
    control_multiplexer_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="control_multiplexer",
        package="olav_control",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_control_control_multiplexer_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_control") /
                "config/parameters/control_multiplexer_node_defaults.yaml").
            as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("in/throttle", "multiplexer/in/throttle"),
            ("in/brake", "multiplexer/in/brake"),
            ("in/steering", "multiplexer/in/steering"),
            ("in/drive", "multiplexer/in/drive"),
            ("in/heartbeat", "multiplexer/in/heartbeat"),
            # > Publishers
            ("out/throttle", "controls/throttle"),
            ("out/brake", "controls/brake"),
            ("out/steering", "controls/steering"),
            ("out/speed", "controllers/speed/speed"),
            ("out/steering_angle", "controllers/steering/angle"),
            ("out/heartbeat", "signals/heartbeat"),
            # > Services
            ("set_control_mode", "multiplexer/set_control_mode"),
            ("cycle_control_mode", "multiplexer/cycle_control_mode"),
            # > Clients
            #("controllers/speed/reset", "controllers/speed/reset"),
            #("controllers/steering/reset", "controllers/steering/reset"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Speed controller node
    speed_controller_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="speed_controller",
        package="olav_control",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_control_speed_controller_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_control") /
                "config/parameters/speed_controller_node_defaults.yaml").
            as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("speed", "controllers/speed/speed"),
            ("odometry", "sensors/powertrain/tachometer/odometry"),
            # > Publishers
            ("throttle", "controls/throttle"),
            ("brake", "controls/brake"),
            ("status", "controllers/speed/status"),
            # Service servers
            ("reset", "controllers/speed/reset"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Steering controller node
    steering_controller_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="steering_controller",
        package="olav_control",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_control_steering_controller_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_control") /
                "config/parameters/steering_controller_node_defaults.yaml").
            as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("setpoint", "controllers/steering/angle"),
            ("feedback", "sensors/steering/angle"),
            # > Publishers
            ("output", "multiplexer/in/steering"),
            ("status", "controllers/steering/status"),
            # > Services
            ("reset", "controllers/steering/reset"),
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
        arguments=["--ros-args", "--log-level", "warn"],
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
        control_multiplexer_node,
        speed_controller_node,
        steering_controller_node,
        diagnostic_aggregator_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
