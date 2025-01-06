"""Interfaces module.
"""

import coloredlogs
import logging
import pylibmodbus
import threading
import time

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)


class State:

    def __init__(self,
                 throttle: int = -1,
                 brake: int = -1,
                 steering: int = -1,
                 gear: int = -1,
                 ignition_on: int = -1,
                 ignition_off: int = -1,
                 engine_start: int = -1,
                 emergency_stop: int = -1):

        self.throttle = throttle
        self.brake = brake
        self.steering = steering
        self.gear = gear
        self.ignition_on = ignition_on
        self.ignition_off = ignition_off
        self.engine_start = engine_start
        self.emergency_stop = emergency_stop

    def to_json(self) -> dict:
        return {
            'steering': self.steering,
            'brake': self.brake,
            'throttle': self.throttle,
            'gear': self.gear,
            'ignition_on': self.ignition_on,
            'ignition_off': self.ignition_off,
            'engine_start': self.engine_start,
            'emergency_stop': self.emergency_stop,
        }


class Feedback:

    def __init__(self, steering: float = 0.0, emergency_stop: int = -1):

        self.steering = steering
        self.emergency_stop = emergency_stop

    def to_json(self) -> dict:

        return {
            'steering': self.steering,
            'emergency_stop': self.emergency_stop
        }


class Command:

    def __init__(self, _range: int = 0, scale: float = 0, offset: int = 0):

        self._range = _range
        self.scale = scale
        self.offset = offset

    def __call__(self, value: float):

        return int(self.scale * value * self._range + self.offset)


class ModbusInterface:

    REGISTER_STEERING_ADDRESS = 0x200 + 0x100
    REGISTER_BRAKE_ADDRESS = 0x200 + 0x101
    REGISTER_THROTTLE_ADDRESS = 0x200 + 0x102
    REGISTER_IGNITION_ON_ADDRESS = 0x200 + 0x103
    REGISTER_IGNITION_OFF_ADDRESS = 0x200 + 0x104
    REGISTER_EMERGENCY_STOP_ADDRESS = 0x200 + 0x105
    REGISTER_ENGINE_START_ADDRESS = 0x200 + 0x106
    REGISTER_GEAR_SELECT_ADDRESS = 0x200 + 0x107
    REGISTER_TICKER_ADDRESS = 0x200 + 0x10C
    REGISTER_FEEDBACK_ADDRESS = 0x100
    REGISTER_FEEDBACK_SIZE = 6

    def __init__(self,
                 address: str = '192.168.69.11',
                 port: int = 502,
                 retry_delay: float = 0.5,
                 tick_rate: float = 1.0 / 0.15):

        self.address = address
        self.port = port
        self.retry_delay = retry_delay

        self._is_modbus_connected = False
        self.modbus = pylibmodbus.ModbusTcp(self.address, self.port)

        self.ticker = True
        self.tick_rate = tick_rate
        self.last_tick_time = -1

        self.thread = threading.Thread(target=self.tick)
        self.lock = threading.Lock()

        self.state = State()

    def connect(self):

        logger.info('Attemping connection to Modbus TCP server...')

        while not self._is_modbus_connected:
            try:
                self.modbus.connect()
                self._is_modbus_connected = True
            # pylibmodbus throws a generic exception here.
            except Exception:
                logger.error('Could not connect to Modbus interface at %s:%s!',
                             self.address, self.port)
                time.sleep(self.retry_delay)

    def loop(self):

        self.thread.start()

    def stop(self):

        self.thread.join()

    def update(self, state):
        #with self.lock:
        self.state = state

    def tick(self):
        while (True):
            #with self.lock:
            current_tick_time = time.time()
            # Write the ticker value.
            #if current_tick_time - self.last_tick_time > self.tick_rate:

            self.modbus.write_register(self.REGISTER_STEERING_ADDRESS,
                                       self.state.steering)
            self.modbus.write_register(self.REGISTER_BRAKE_ADDRESS,
                                       self.state.brake)
            self.modbus.write_register(self.REGISTER_THROTTLE_ADDRESS,
                                       self.state.throttle)
            self.modbus.write_register(self.REGISTER_IGNITION_ON_ADDRESS,
                                       self.state.ignition_on)
            self.modbus.write_register(self.REGISTER_IGNITION_OFF_ADDRESS,
                                       self.state.ignition_off)
            self.modbus.write_register(self.REGISTER_EMERGENCY_STOP_ADDRESS,
                                       self.state.emergency_stop)
            self.modbus.write_register(self.REGISTER_ENGINE_START_ADDRESS,
                                       self.state.engine_start)
            self.modbus.write_register(self.REGISTER_GEAR_SELECT_ADDRESS,
                                       self.state.gear)

            self.ticker = not self.ticker
            self.modbus.write_register(self.REGISTER_TICKER_ADDRESS,
                                       self.ticker)
            self.last_tick_time = current_tick_time

            # Get feedback from the Modbus registers.
            #feedback = self.modbus.read_registers(0x100, 6)
            #ignition_on = feedback[5]
            time.sleep(0.15)
