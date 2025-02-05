 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( O ) ( L ) ( A ) ( V )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #                  OLAV: Off-Road Light Autonomous Vehicle                  #
 #############################################################################

from __future__ import annotations

import pygame
import coloredlogs
import logging
import os
import time

# Add an environment variable to suppress the PyGame welcome screen. The
# variable must be present at the time of import.
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)

CONTROL_RATE = 100.0
LOG_RATE = 5.0
CONTROL_TIME_WINDOW = 1.0 / CONTROL_RATE
LOG_TIME_WINDOW = 1.0 / LOG_RATE
SWITCH_TIME_WINDOW = 3.0


class HumanInterfaceDevice:
    """Generic implementation for a Human Interface Device (HID). """

    def __init__(self : HumanInterfaceDevice) -> None:
        """Construct a new Human Interface Device (HID)."""

        self.steering = None
        """Steering input."""

        self.brake = None
        """Brake input."""
        
        self.throttle = None
        """Throttle input."""

        self.shift_down = None
        """Shift gear down action."""

        self.shift_up = None
        """Shift gear up action."""

        # Define the device bindings.
        self.bindings = {
            'throttle': -1,
            'brake': -1,
            'steering': -1,
            'ignition_on': -1,
            'ignition_off': -1,
            'emergency_stop': -1,
            'engine_start': -1,
            'shift_down': -1,
            'shift_up': -1
        }
        '''Device bindings.'''

        # Initialize PyGame, required for reading device input.
        pygame.init()


class Gamepad(HumanInterfaceDevice):
    """Generic implementation of a gamepad with zero or more axes, hats and
       buttons."""

    def __init__(self,
                 axes: dict = {},
                 hats: dict = {},
                 buttons: dict = {},
                 retry_delay: float = 1.0) -> None:
        """Construct a new generic gamepad.

        Args:
            axes (dict, optional): Axis label to axis index pairs. Defaults to
            {}.
            hats (dict, optional): Hat label to hat index pairs. Defaults to
            {}.
            buttons (dict, optional): Button label to button index pairs.
            Defaults to {}.
        """

        self.device = pygame.joystick.Joystick(-1)
        '''PyGame device handle.'''

        self.axes = axes
        '''Gamepad axes.'''

        self.hats = hats
        '''Gamepad hats.'''

        self.buttons = buttons
        '''Gamepad buttons.'''

        self.retry_delay = retry_delay
        '''Connection retry delay, in seconds.'''

        super().__init__()

        # Initialise the PyGame joystick module.
        pygame.joystick.init()

    def show_buttons(self : Gamepad) -> None:
        """Print the available axes, buttons and hats.

        Parameters
        ----------
        self : Gamepad
            _description_
        """

        for i in range(self.device.get_numbuttons()):
            logger.info('Button %i: %i', i, self.device.get_button(i))

        # TODO:
        for i in range(self.device.get_numaxes()):
            print(f"{i}: {self.device.get_axis(i)}")

        # TODO:
        for i in range(self.device.get_numhats()):
            print(f"{i}: {self.device.get_hat(i)}")

    def _get_button(self, button) -> int:
        return self.device.get_button(button)

    def _get_axis(self, axis) -> float:
        return self.device.get_axis(axis)

class XboxGamepad(Gamepad):

    AXES = {
        'LSTICK_H': 0,
        'LSTICK_V': 1,
        'LTRIGGER': 2,
        'RSTICK_H': 3,
        'RSTICK_V': 4,
        'RTRIGGER': 5,
    }

    HATS = {}

    BUTTONS = {
        'A': 0,
        'B': 1,
        'X': 2,
        'Y': 3,
        'LBUMPER': 4,
        'RBUMPER': 5,
        'BACK': 6,
        'START': 7,
        'GUIDE': 8,
        'LSTICK': 9,
        'RSTICK': 10,
    }

    def __init__(self, retry_delay: float = 1.0) -> None:

        super().__init__(axes=self.AXES,
                         hats=self.HATS,
                         buttons=self.BUTTONS,
                         retry_delay=retry_delay)

        self.connect()

    def connect(self) -> None:
        # Attempt to find a gamepad
        gamepads_count = pygame.joystick.get_count()
        while (gamepads_count < 1):
            logger.warning('No joysticks found, waiting for a valid device...')
            time.sleep(self.retry_delay)
            gamepads_count = pygame.joystick.get_count()

        # Looks like we have found one or more valid gamepads. Let us select the
        # first one, inform the user of our choice and initialize the device.
        pygame.joystick.init()
        if (gamepads_count == 1):
            logger.info('Found a single joystick.')

        else:
            logger.warning('Found %i connected joystics, selecting #0.',
                           gamepads_count)
        self.device = pygame.joystick.Joystick(0)
        self.device.init()

        self.show_buttons()

    def controller_bindings(self) -> None:
        """Assign the controller bindings to the vehicle commands."""

        self.steering = self._get_axis('LSTICK_H')
        self.brake = self._get_axis('LTRIGGER')
        self.throttle = self._get_axis('RTRIGGER')
        self.shift_down = self._get_button('LBUMPER')
        self.shift_up = self._get_button('RBUMPER')
