 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( O ) ( L ) ( A ) ( V )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #                  OLAV: Off-Road Light Autonomous Vehicle                  #
 #############################################################################

import coloredlogs
import cv2
import logging
import os
import requests
import threading
import time

from reolinkapi import Camera
from sshkeyboard import listen_keyboard, stop_listening

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)


class BackupCamera:
    """Backup camera interface."""

    def __init__(self,
                 address: str = '127.0.0.1',
                 username: str = 'admin',
                 password: str = ''):
        """Construct a new Reolink backup camera.

        Parameters
        ----------
        address : str, optional
            Address of the Reolink management interface, by default '127.0.0.1'
        username : str, optional
            Management interface login username, by default 'admin'
        password : str, optional
            Management interface login password, by default ''
        """

        self.address = address
        self.username = username
        self.password = password

        # Initialize the camera.
        self.camera = Camera(ip=address,
                             username=username,
                             password=password,
                             https=False,
                             defer_login=True)

        # Instantiate separate threads for acquiring the video feeds and sending
        # keyboard commands.
        self.video_thread = threading.Thread(target=None)
        self.keyboard_thread = threading.Thread(target=None)

        # Set up pan speeds.
        self.pan_speeds = [1, 3, 5, 7, 10, 15, 25, 50]
        self.pan_speed = 0

    def connect(self):
        """Connect to the Reolink management interface."""

        logger.info('Connecting to %s as user %s...', self.address,
                    self.username)

        # Attempt to connect to the Reolink management interface.
        is_connected = False
        while not is_connected:
            try:
                self.camera.login()
                is_connected = True
                logger.info('Connected successfully!')
            except requests.exceptions.ConnectionError as exception:
                logger.warning('Could not connect %s! Retrying...', exception)
                time.sleep(0.5)

    def disconnect(self):
        """Disconnect from the Reolink management interface."""

        self.camera.logout()
        self.video_thread.join()

    def start_video(self):
        """Start the video streaming thread."""

        self.video_thread = threading.Thread(target=self._video_stream)
        self.video_thread.start()

    def start_keyboard(self):
        """Start the keyboard capture thread."""

        self.keyboard_thread = threading.Thread(target=self._keyboard_thread)
        self.keyboard_thread.start()

    def _video_stream(self):
        """Video streaming thread."""

        def inner_callback(img):
            cv2.imshow("name", img)
            print("got the image non-blocking")
            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyAllWindows()
                exit(1)

        stream = self.camera.open_video_stream(callback=inner_callback)
        print(stream.is_alive())
        while True:
            if not stream.is_alive():
                print("continuing")
                break

    def _keyboard_thread(self):
        """Keyboard capture thread."""

        listen_keyboard(on_press=self._get_keypress,
                        sequential=True,
                        delay_second_char=0.05,
                        sleep=0.0005)

    def _get_keypress(self, key: str):
        """Capture a keypress from console.

        Parameters
        ----------
        key : str
            _description_
        """

        print(f"'{key}' pressed")
        # Actuate the pan axis.
        if key == 'left':
            logger.info('Panning left.')
            self.camera.move_left(self.pan_speeds[self.pan_speed])
        if key == 'right':
            logger.info('Panning right.')
            self.camera.move_left(self.pan_speeds[self.pan_speed])
        # Actuate the tilt axis.
        if key == 'up':
            logger.info('Tilting up.')
            self.camera.move_up(self.pan_speeds[self.pan_speed])
        if key == 'down':
            logger.info('Tilting down.')
            self.camera.move_down(self.pan_speeds[self.pan_speed])
        # Set up pan speeds.
        if key == 'pageup':
            if (self.pan_speed < len(self.pan_speeds) - 1):
                self.pan_speed += 1
            else:
                self.pan_speed = 0
            logger.info('New pan speed %f', self.pan_speeds[self.pan_speed])
        if key == 'pagedown':
            if (self.pan_speed > 0):
                self.pan_speed -= 1
            else:
                self.pan_speed = len(self.pan_speeds) - 1
            logger.info('New pan speed %f', self.pan_speeds[self.pan_speed])
        if key == "z":
            stop_listening()
