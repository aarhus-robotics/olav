import time

from serial import Serial, SerialException

from dashboard import PlotjugglerInterface
import json


class PowertrainInterface:

    def __init__(self, port: str, baudrate: int, timeout: int):

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = Serial(port=self.port,
                             baudrate=self.baudrate,
                             timeout=self.timeout)

    def connect(self):
        self.serial.close()
        #self.serial.flush()

        try:
            self.serial.open()
        except SerialException as exception:
            print('Could not open serial port {}: {}\n'.format(
                self.serial.name, exception))

    def read_line(self):

        return self.serial.readline()

    def compute_speed(self, cdps: int) -> float:

        wheel_circumference = 2.16  # estimated, avg front and rear (2.17. 2.15)

        dps = 0.01 * cdps
        rps = dps / 360.0
        return rps * wheel_circumference


def main():

    plotter = PlotjugglerInterface()
    plotter.bind()

    interface = PowertrainInterface('/dev/ttyACM0', 9600, 10)
    interface.connect()
    while (True):
        listofstrings = interface.read_line()[4:].rstrip().decode(
            'utf-8').split(',')
        cdps = int(listofstrings[0])
        rpm = int(listofstrings[1])
        estop = bool(listofstrings[2])

        mybeautifuldict = {
            "mytopic": {
                # TODO: Remove these explicit casts.
                'vehicle_speed': float(interface.compute_speed(cdps)),
                'engine_speed': int(rpm),
                'estop': bool(estop)
            }
        }
        print(mybeautifuldict)
        plotter.send_packet(mybeautifuldict)

        time.sleep(0.0005)


if __name__ == '__main__':
    main()
