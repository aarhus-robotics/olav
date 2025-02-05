"""PlotJuggler dashboard module.

This module is used to provide an easy to use, ZeroMQ-based interface to
PlotJuggler to debug and inspect the drive-by-wire and odometer systems outside
of ROS.
"""

import coloredlogs
import json
import logging
import subprocess
from matplotlib.font_manager import json_dump
import zmq

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)


class PlotjugglerInterface():
    """ZeroMQ-based interface to PlotJuggler."""

    def __init__(self,
                 protocol: str = 'tcp',
                 address: str = '127.0.0.1',
                 port: int = 9872):
        """Construct a new interface to PlotJuggler.

        Parameters
        ----------
        protocol : str, optional
            Protocol to use when connecting to PlotJuggler via ZeroMQ, by
            default 'tcp'
        address : str, optional
            Address to use when connecting to PlotJuggler via ZeroMQ, by default
            '127.0.0.1'
        port : int, optional
            Port to use when connecting to PlotJuggler via ZeroMQ, by default
            9872
        """

        self.protocol = protocol
        self.address = address
        self.port = port

        # Initialize a new ZeroMQ context and a PUB socket.
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)

    def bind(self):
        """Bind the interface ZeroMQ socket."""

        logger.info('Binding ZeroMQ publisher...')

        self.zmq_socket.bind(f'{self.protocol}://{self.address}:{self.port}')

    def send(self, state, feedback):
        message = {'state': state.to_json(), 'feedback': feedback}
        self.zmq_socket.send_string(json.dumps(message))

    def send_packet(self, dictionary):
        """Send a JSON dictionary via the ZeroMQ socket. 

        Parameters
        ----------
        dictionary : _type_
            JSON dictionary to be serialized and sent.
        """

        logger.info('Sending data...')

        self.zmq_socket.send_string(json.dumps(packet))

    def launch(self):
        """Launch PlotJuggler as a subprocess."""

        logger.info('Starting PlotJuggler...')

        subprocess.Popen(['plotjuggler', '-n'],
                         shell=False,
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
