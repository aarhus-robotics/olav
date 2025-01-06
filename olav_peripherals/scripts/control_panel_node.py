#!/usr/bin/env python3

import rclpy

from olav_peripherals.control_panel import ControlPanelNode

def main():
    rclpy.init()

    node = ControlPanelNode('control_panel')

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()