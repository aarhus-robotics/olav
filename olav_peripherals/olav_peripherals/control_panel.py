#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################

from __future__ import annotations

import random
import time

import pygame
import pygame.midi

from rclpy import Future, Node
from rclpy.lifecycle import State, TransitionCallbackReturn
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from std_srvs.srv import SetBool, Trigger

from olav_interfaces.msg import SetpointStamped
from olav_interfaces.srv import SetControlAuthority, SetControlMode
from olav_interfaces.action import ShiftGear


class Function:

    pass


class DynamicParameter(Function):

    TYPES = {
        "bool": ParameterType.PARAMETER_BOOL,
        "double": ParameterType.PARAMETER_DOUBLE,
    }

    def __init__(self: Parameter, name: str, topic: str, _type: str, node: Node):

        self.name: str = name
        self.topic: str = topic
        self._type: str = _type
        self.node: Node = node
        self.future = Future()

        self.client = self.node.create_client(
            SetParameters,
            self.topic + "/set_parameters",
        )

    def __call__(self, value):

        self.node.get_logger().info(
            f"Sending request to '{self.topic}' for updating parameter '{self.name}' with value '{value}.'")

        # Instantiate a new parameter
        parameter = Parameter()
        parameter.name = self.name
        parameter.value.type = self._type
        parameter.value.double_value = value

        request = SetParameters.Request()
        request.parameters.append(parameter)

        self.future = self.client.call_async(request)


class Action(Function):

    TYPES = {
        "shift_gear": ShiftGear,
    }


class Service(Function):

    TYPES = {
        "trigger": Trigger,
        "set_bool": SetBool,
        "set_control_authority": SetControlAuthority,
        "set_control_mode": SetControlMode,
    }

    def __init__(self: Service, name: str, topic: str, _type, node: Node) -> None:

        self.name: str = name
        self.future: Future = Future()
        self.topic: str = topic
        self._type = _type
        self.node = node

        # Create the service client.
        self.client = node.create_client(self._type, topic)


class SetControlAuthorityService(Service):

    def __init__(self, *args, **kwargs) -> None:

        super().__init__(*args, **kwargs)

        self.authority = 0

    def __call__(self, _):

        request = SetControlAuthority.Request()
        request.authority = self.authority

        # Update the mode iterator.
        self.authority = 0 if self.authority + 1 > 2 else self.authority + 1

        self.future = self.client.call_async(request)


class SetControlModeService(Service):

    def __init__(self, *args, **kwargs) -> None:

        super().__init__(*args, **kwargs)

        self.mode = 0

    def __call__(self, _) -> None:

        self.node.get_logger().info(f"Sending request to {self.topic} to set control mode to {self.mode}'")

        request = SetControlMode.Request()
        request.mode = self.mode

        # Update the mode iterator.
        self.mode = 0 if self.mode + 1 > 4 else self.mode + 1

        self.future = self.client.call_async(request)


class ShiftGearService(Service):

    def __call__(self, _) -> None:

        pass


class SetBoolService(Service):

    def __call__(self, value):

        request = SetBool.Request()
        request.data = value
        self.future = self.client.call_async(request)


class TriggerService(Service):

    def __call__(self, value):

        self.node.get_logger().info(f"Sending trigger request to '{self.topic}'.")

        request = Trigger.Request()
        self.future = self.client.call_async(request)


class ServiceFactory:

    @staticmethod
    def make(name: str, topic: str, _type, node: Node):

        if _type == SetBool:
            return SetBoolService(name, topic, _type, node)
        elif _type == SetControlMode:
            return SetControlModeService(name, topic, _type, node)
        elif _type == ShiftGear:
            return ShiftGearService(name, topic, _type, node)
        elif _type == Trigger:
            return TriggerService(name, topic, _type, node)


class Publisher(Function):

    TYPES = {
        "control_effort_stamped": SetpointStamped,
        "control_setpoint_stamped": SetpointStamped,
    }

    def __init__(self, name: str, topic: str, _type, repeat, node: Node):

        self.name: str = name
        self.topic: str = topic
        self._type = _type
        self.repeat = repeat
        self.node: Node = node
        self.publisher: Publisher = node.create_publisher(
            _type,
            topic,
            1,
        )


class ControlSetpointPublisher(Publisher):

    def __call__(self, value):

        control_setpoint_message = SetpointStamped()
        control_setpoint_message.header.frame_id = "console"
        control_setpoint_message.header.stamp = self.node.get_clock().now().to_msg()
        control_setpoint_message.setpoint = value
        self.publisher.publish(control_setpoint_message)


class ControlEffortPublisher(Publisher):

    def __call__(self, value):

        control_effort_message = SetpointStamped()
        control_effort_message.header.frame_id = "console"
        control_effort_message.header.stamp = self.node.get_clock().now().to_msg()
        control_effort_message.effort = value
        self.publisher.publish(control_effort_message)


class PublisherFactory:

    @staticmethod
    def make(name: str, topic: str, _type, repeat: bool, node: Node) -> Publisher:

        if _type == SetpointStamped:
            return ControlEffortPublisher(name, topic, _type, repeat, node)
        elif _type == SetpointStamped:
            return ControlSetpointPublisher(name, topic, _type, repeat, node)
        else:
            raise Exception("Invalid publisher type.")


class Key:

    def __init__(self, name: str, function: Function, gain: float, symmetric: bool = False) -> None:

        self.name: str = name
        self.function: Function = function
        self.gain: float = gain
        self.symmetric: bool = symmetric
        self.value: float = 0.0

    def update(self: Key, value: float):
        self.value = (value - 63.5) / 63.5 if self.symmetric else value / 127.0

    def execute(self: Key):

        self.function(self.value)


class ControlPanelNode(Node):
    # fmt: off
    MAPPINGS = {
        "TRACK_LEFT": 58, "TRACK_RIGHT": 59, "CYCLE": 46, "SET": 60, "MARKER_LEFT": 61, "MARKER_RIGHT": 62,
        "FAST_FORWARD_LEFT": 43, "FAST_FORWARD_RIGHT": 44, "STOP": 42, "PLAY": 41, "RECORD": 45,
        "CHANNEL_1_PAN": 16, "CHANNEL_1_SLIDER": 0, "CHANNEL_1_SOLO": 32, "CHANNEL_1_MUTE": 48, "CHANNEL_1_RECORD": 64,
        "CHANNEL_2_PAN": 17, "CHANNEL_2_SLIDER": 1, "CHANNEL_2_SOLO": 33, "CHANNEL_2_MUTE": 49, "CHANNEL_2_RECORD": 65,
        "CHANNEL_3_PAN": 18, "CHANNEL_3_SLIDER": 2, "CHANNEL_3_SOLO": 34, "CHANNEL_3_MUTE": 50, "CHANNEL_3_RECORD": 66,
        "CHANNEL_4_PAN": 19, "CHANNEL_4_SLIDER": 3, "CHANNEL_4_SOLO": 35, "CHANNEL_4_MUTE": 51, "CHANNEL_4_RECORD": 67,
        "CHANNEL_5_PAN": 20, "CHANNEL_5_SLIDER": 4, "CHANNEL_5_SOLO": 36, "CHANNEL_5_MUTE": 52, "CHANNEL_5_RECORD": 68,
        "CHANNEL_6_PAN": 21, "CHANNEL_6_SLIDER": 5, "CHANNEL_6_SOLO": 37, "CHANNEL_6_MUTE": 53, "CHANNEL_6_RECORD": 69,
        "CHANNEL_7_PAN": 22, "CHANNEL_7_SLIDER": 6, "CHANNEL_7_SOLO": 38, "CHANNEL_7_MUTE": 54, "CHANNEL_7_RECORD": 70,
        "CHANNEL_8_PAN": 23, "CHANNEL_8_SLIDER": 7, "CHANNEL_8_SOLO": 39, "CHANNEL_8_MUTE": 55, "CHANNEL_8_RECORD": 71,
    }
    # fmt: on

    def __init__(self, name, **kwargs) -> None:

        super().__init__(name, **kwargs)

        # Dictionary of mapped keys.
        self._keys = {}

        self._get_parameters()

        self._initialize()
        self._create_timers()

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:

        del self.controller
        pygame.midi.quit()

        return TransitionCallbackReturn.SUCCESS

    def _initialize(self):

        # MIDI device
        # -----------

        if self.debug:
            self.get_logger().info("Skipping MIDI connection: debug mode is on.")
        else:
            # Initialize the MIDI device.
            pygame.midi.init()

            # Wait for a valid device to come online ...
            number_of_midi_devices = pygame.midi.get_count()
            while number_of_midi_devices < 1:
                self.get_logger().warning("No MIDI devices connected, retrying...")
                time.sleep(1.0)

            # List all valid MIDI devices.
            for i in range(pygame.midi.get_count()):
                self.get_logger().info(f"Found MIDI device: {i}: {pygame.midi.get_device_info(i)}")

            # Instantiate a new input MIDI device.
            self.controller = pygame.midi.Input(self.input_device)
        # -----------

    def _get_parameters(self):

        # ... define debug mode.
        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value

        # Retrieve the user-specified MIDI device name.
        self.declare_parameter("device", 5)
        self.input_device = (self.get_parameter("device").get_parameter_value().integer_value)

        self.declare_parameter("rate", 100.0)
        self.rate: float = self.get_parameter("rate").get_parameter_value().double_value

        # Iterate through all user-specified key bindings.
        for mapping, _ in self.MAPPINGS.items():

            # Key output
            # ----------

            # These parameters may be optionally specified to remap the slider,
            # knob or button output to the desired range.

            # ... define the gain, or scaling factor.
            self.declare_parameter(f"mappings.{mapping}.gain", 1.0)
            gain = self.get_parameter(f"mappings.{mapping}.gain").value

            # ... define the offset.
            self.declare_parameter(f"mappings.{mapping}.offset", 0.0)
            offset = self.get_parameter(f"mappings.{mapping}.offset").value

            # ... define whether this key binding provides output in the range
            # (-gain * output + offset, gain * output + offset) or in the range
            # (0.0, gain * output + offset).
            self.declare_parameter(f"mappings.{mapping}.symmetric", False)
            symmetric = self.get_parameter(f"mappings.{mapping}.symmetric").value

            # Th
            self.declare_parameter(f"mappings.{mapping}.function", "none")
            function = self.get_parameter(f"mappings.{mapping}.function").value
            # ----------

            # Report to the user the key binding being processed.
            self.get_logger().info(
                f"Got mapping {mapping} with {'symmetric ' if symmetric else ''}gain {gain} offset by {offset}'")

            # Key function
            # ------------

            self.declare_parameter(f"mappings.{mapping}.name", "")
            name = self.get_parameter(f"mappings.{mapping}.name").value

            # Get the publisher topic.
            self.declare_parameter(f"mappings.{mapping}.topic", "")
            topic = self.get_parameter(f"mappings.{mapping}.topic").value

            self.declare_parameter(f"mappings.{mapping}.type", "")
            _type = self.get_parameter(f"mappings.{mapping}.type").value

            # Register function handle.
            if function == "service":
                function_handle = ServiceFactory.make(mapping, topic, Service.TYPES[_type], self)
            elif function == "parameter":
                function_handle = DynamicParameter(name, topic, DynamicParameter.TYPES[_type], self)
            elif function == "publisher":

                self.declare_parameter(f"mappings.{mapping}.repeat", False)
                repeat = self.get_parameter(f"mappings.{mapping}.repeat").value

                function_handle = PublisherFactory.make(name, topic, Publisher.TYPES[_type], repeat, self)

            # Add the key binding.
            self._keys[mapping] = Key(mapping, function_handle, gain, symmetric)
            # ------------

    def _create_timers(self):

        self.timer = self.create_timer(1.0 / self.rate, self._timer_callback)

    def _timer_callback(self):
        """run the update hook (poll the midi controller, publish the message)"""

        # Check for any futures that have been completed.
        for name, key in self._keys.items():
            if isinstance(key.function, Service):
                if key.function.future.done():
                    if key.function.future.result() is not None:
                        response = key.function.future.result()
                        self.get_logger().info(f"Result of set parameters: for {response}")
                    else:
                        self.get_logger().info(f"Service call failed {key.function.future.exception()}")
                    key.function.future = Future()
            elif isinstance(key.function, Publisher) and key.function.repeat:
                key.execute()

        if self.debug:
            datalist = [[
                [(None, self.MAPPINGS["CHANNEL_1_SLIDER"], random.randint(0, 127))],
                [(None, self.MAPPINGS["TRACK_LEFT"], random.randint(0, 127))],
                [(None, self.MAPPINGS["RECORD"], random.randint(0, 127))],
            ]]
        else:
            datalist = []

            while self.controller.poll():
                datalist.append(self.controller.read(1))

        for data in datalist:
            controls = self._get_controls(data)
            for key, value in controls:
                name = self._get_control_name(key)

                self._keys[name].update(value)
                self._keys[name].execute()

    def _get_control_name(self, control_value):
        for key, value in self.MAPPINGS.items():
            if value == control_value:
                return key

    def _get_controls(self, events):
        controls = []

        for event in events:
            control = event[0]
            if self.debug or control[0] & 0xF0 == 176:
                key = control[1]
                value = control[2]
                controls.append((key, value))

        return controls
