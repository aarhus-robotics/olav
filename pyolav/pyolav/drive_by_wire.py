"""Drive by wire module.
"""

import coloredlogs
import logging
import time

from interfaces import Command, ModbusInterface, State
from controller import Gamepad, XboxGamepad
from dashboard import PlotjugglerInterface

import pygame

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)


class DriveByWire:

    def __init__(self,
                 control_rate: float = 1.0 / 0.15,
                 log_rate: float = 1.0 / 0.25,
                 use_modbus: bool = True,
                 modbus: ModbusInterface = ModbusInterface(),
                 use_gamepad: bool = True,
                 gamepad: Gamepad = XboxGamepad(),
                 use_plotjuggler: bool = False,
                 plotjuggler: PlotjugglerInterface = PlotjugglerInterface(),
                 throttle: Command = Command(40, 1.0, 40),
                 brake: Command = Command(40, 1.0, 40),
                 steering: Command = Command(3000, 0.5, 0),
                 shift_interval: float = 2.0):

        # Initialize the interfaces
        self.gamepad = gamepad
        self.plotjuggler = plotjuggler

        self.modbus = modbus

        # Initialize the options.
        self.use_modbus = use_modbus
        self.use_gamepad = use_gamepad
        self.use_plotjuggler = use_plotjuggler

        self.throttle = throttle
        self.brake = brake
        self.steering = steering
        self.gear = 5

        self.ready_to_switch = False
        self.last_shift_time = -1.0
        self.shift_interval = shift_interval

        # Initialize the spin rates.
        self.control_rate = control_rate
        self.log_rate = log_rate


        if use_modbus:
            self.modbus.connect()

        if use_plotjuggler:
            self.plotjuggler.bind()
            self.plotjuggler.launch()

    def run(self):
        #logger.info('Initialising control loop at %s Hz.', CONTROL_RATE)

        pygame.event.get()

        start_time = time.time()

        # Get the system inputs.
        state = State(steering=self.steering(self.gamepad._get_axis(0)),
                      throttle=self.throttle(self.gamepad._get_axis(5)),
                      brake=self.brake(self.gamepad._get_axis(2)),
                      ignition_on=self.gamepad._get_button(2),
                      ignition_off=0,
                      engine_start=self.gamepad._get_button(1),
                      emergency_stop=0,
                      gear=5)

        # Modbus
        self.modbus.update(state)
        #feedback = self.modbus.get_feedback()

        # Log the control inputs.
        logger.info(
            'CONTROL >> S: %0.2f | B: %0.2f | T: %0.2f | G: %i | ION: %i | IOFF: %i | START: %i | ESTOP: %i',
            state.steering, state.brake, state.throttle, state.gear,
            state.ignition_on, state.ignition_off, state.engine_start,
            state.emergency_stop)
        # Log the control outputs.
        #logger.info('FEEDBACK >> S: %0.2f | ION: %i',
        #            np.int16(feedback[0]), feedback[5])

        # Send the latest information to PlotJuggler.
        self.plotjuggler.send(state, {})

        # Loop control
        # ------------
        end_time = time.time()
        #loop_duration = end_time - start_time
        #control_sleep_time = CONTROL_TIME_WINDOW - loop_duration
        #if control_sleep_time > 0.0:
        #    time.sleep(control_sleep_time)

    def _check_gear_switch(self, time: float):
        if (time - self.last_shift_time) > self.shift_interval:

            # Default to no gear shift.
            gear_shift = 0

            if not self.ready_to_switch:
                self.ready_to_switch = True
                logger.info('Gear switch ready!')
            if self.gamepad.shift_down:
                self.ready_to_switch = False
                gear_shift = -1
                self.last_shift_time = time

            elif self.gamepad.shift_up:
                self.ready_to_switch = False
                gear_shift = 1
                self.last_shift_time = time

            # Check if the target gear is within a valid range.
            target_gear = self.gear + gear_shift
            if target_gear < 5 and target_gear >= 0:
                self.gear += gear_shift
