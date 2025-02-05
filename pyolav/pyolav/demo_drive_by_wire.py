import time

from drive_by_wire import DriveByWire


def main():

    controller = DriveByWire()
    controller.modbus.loop()
    while (True):
        controller.run()
        time.sleep(0.01)


if __name__ == "__main__":
    main()
