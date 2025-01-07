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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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
    log_level_argument = DeclareLaunchArgument("log_level", default_value="INFO")

    # Nodes
    # -----
    # > Drive-by-wire interface node
    drive_by_wire_interface_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="drive_by_wire_interface",
        package="olav_control",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_control_drive_by_wire_interface_node",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            Path(
                get_package_share_path("olav_control") /
                "config/parameters/drive_by_wire_interface_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("feedback/engine/speed", "sensors/powertrain/engine/speed"),
            ("feedback/odometry", "sensors/powertrain/tachometer/odometry"),
            ("controls/throttle", "controls/throttle"),
            ("controls/brake", "controls/brake"),
            ("controls/steering", "controls/steering"),
            ("signals/heartbeat", "signals/heartbeat"),
            # > Publishers
            ("signals/ready", "signals/ready"),
            ("signals/estop", "signals/estop"),
            ("sensors/steering/angle", "sensors/steering/angle"),
            ("model/joints/steering", "model/joints/updates/front_wheels"),
            ("status", "dbw/status"),
            # > Services servers
            ("ready", "dbw/ready"),
            ("cycle_ignition", "dbw/cycle_ignition"),
            ("set_ignition", "dbw/set_ignition"),
            ("start_engine", "dbw/start_engine"),
            ("emergency_stop", "dbw/emergency_stop"),
            ("shift_gear_up", "dbw/shift_gear_up"),
            ("shift_gear_down", "dbw/shift_gear_down"),
            ("set_steering_pid_gains", "dbw/set_steering_pid_gains"),
            # > Service clients
            ("steering/start", "controllers/steering/start"),
            ("steering/stop", "controllers/steering/stop"),
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
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            Path(get_package_share_path("olav_sensors") /
                 "config/parameters/powertrain_interface_node_defaults.yaml").as_posix(),
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
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            Path(get_package_share_path("olav_control") /
                 "config/parameters/control_multiplexer_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            # :: Input efforts
            ("in/throttle", "mux/in/throttle"),
            ("in/brake", "mux/in/brake"),
            ("in/steering", "mux/in/steering"),
            # :: Input commands
            ("in/drive", "mux/in/drive"),
            # :: Input signals
            ("in/heartbeat", "mux/in/heartbeat"),
            ("in/heartbeat", "mux/in/heartbeat"),
            # > Publishers
            # :: Muxed efforts
            ("out/throttle", "controls/throttle"),
            ("out/brake", "controls/brake"),
            ("out/steering", "controls/steering"),
            # :: Muxed heartbeats
            ("out/heartbeat", "signals/heartbeat"),
            # > Services servers
            ("set_control_mode", "mux/set_control_mode"),
            ("cycle_control_mode", "mux/cycle_control_mode"),
            # > Service clients
            ("controllers/speed/start", "controllers/speed/start"),
            ("controllers/speed/stop", "controllers/speed/stop"),
            ("controllers/steering/start", "controllers/steering/start"),
            ("controllers/steering/stop", "controllers/steering/stop"),
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
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            Path(get_package_share_path("olav_control") /
                 "config/parameters/speed_controller_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("speed", "controls/mux/setpoints/speed"),
            ("odometry", "sensors/ins/filter/odometry"),
            # > Publishers
            ("throttle", "commands/inputs/efforts/throttle"),
            ("brake", "commands/inputs/efforts/brake"),
            ("status", "controllers/speed/status"),
            # Service servers
            ("start", "controllers/speed/start"),
            ("stop", "controllers/speed/stop"),
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
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            Path(get_package_share_path("olav_control") /
                 "config/parameters/steering_controller_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("setpoint", "controls/mux/setpoints/steering"),
            ("feedback", "sensors/encoders/steering"),
            # > Publishers
            ("output", "commands/inputs/efforts/steering"),
            ("status", "controllers/steering/status"),
            # > Services
            ("start", "controllers/steering/start"),
            ("stop", "controllers/steering/stop"),
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
            Path(get_package_share_path("olav_control") /
                 "config/parameters/diagnostic_aggregator_node_defaults.yaml").as_posix(),
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
