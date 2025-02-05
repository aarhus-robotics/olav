#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################
"""Peripherals launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
                                               default_value="WARN")

    # Nodes
    # -----
    # > Joystick node
    joy_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="joystick_parser",
        package="joy",
        #prefix="konsole -e gdb -ex=r --args",
        executable="joy_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_peripherals") /
                "config/parameters/joy_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("joy/set_feedback", "peripherals/gamepad/force_feedback"),
            # > Publishers
            ("joy", "peripherals/gamepad/state"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Gamepad interface node
    gamepad_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="gamepad_interface",
        package="olav_peripherals",
        #prefix="konsole -e gdb -ex=r --args",
        executable="olav_peripherals_gamepad_interface_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("olav_peripherals") /
                "config/parameters/gamepad_interface_node_defaults.yaml").
            as_posix(),
            LaunchConfiguration("parameters_overrides"),
        ],
        remappings=[
            # > Subscriptions
            ("joy", "peripherals/gamepad/state"),
            # > Publishers
            ("throttle", "multiplexer/in/throttle"),
            ("brake", "multiplexer/in/brake"),
            ("drive", "multiplexer/in/drive"),
            ("heartbeat", "multiplexer/in/heartbeat"),
            # > Service clients
            ("start_engine", "drive_by_wire/start_engine"),
            ("shift_gear_up", "drive_by_wire/shift_gear_up"),
            ("shift_gear_down", "drive_by_wire/shift_gear_down"),
            ("cycle_ignition", "drive_by_wire/cycle_ignition"),
            ("cycle_control_mode", "multiplexer/cycle_control_mode"),
            ("set_control_mode", "multiplexer/set_control_mode"),
            ("emergency_stop", "drive_by_wire/emergency_stop"),
            ("ready", "drive_by_wire/ready"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    # > Control panel node
    #control_panel_node = Node(
    #    namespace=LaunchConfiguration("namespace"),
    #    name="control_panel_interface",
    #    package="olav_peripherals",
    #    #prefix="konsole -e gdb -ex=r --args",
    #    executable="control_panel_node.py",
    #    arguments=[
    #        "--ros-args", "--log-level",
    #        LaunchConfiguration("log_level")
    #    ],
    #    parameters=[
    #        Path(
    #            get_package_share_path("olav_peripherals") /
    #            "config/parameters/control_panel_node_defaults.yaml",
    #        ).as_posix(),
    #        LaunchConfiguration("parameters_overrides"),
    #    ],
    #    remappings=[
    #        # > Publishers
    #        ("efforts/throttle", "commands/inputs/efforts/throttle"),
    #        ("efforts/brake", "commands/inputs/efforts/brake"),
    #        ("efforts/steering", "commands/inputs/efforts/steering"),
    #        ("setpoints/speed", "commands/inputs/setpoints/speed"),
    #        ("setpoints/steering", "commands/inputs/setpoints/steering"),
    #        # Service clients
    #        ("set_ignition", "drive_by_wire/set_ignition"),
    #        ("start_engine", "drive_by_wire/start_engine"),
    #        ("set_control_authority", "multiplexer/set_control_authority"),
    #        ("set_control_mode", "multiplexer/set_control_mode"),
    #        ("start_recording", "datalogger/start"),
    #        ("stop_recording", "datalogger/stop"),
    #    ],
    #    emulate_tty=True,
    #    output={
    #        "both": ["screen", "own_log"],
    #    },
    #    on_exit=Shutdown(),
    #)

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Nodes
        joy_node,
        gamepad_node,
        #control_panel_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
