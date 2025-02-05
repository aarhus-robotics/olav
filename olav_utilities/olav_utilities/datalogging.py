from __future__ import annotations

import importlib.resources
import pathlib
import pty
import random
import signal
import subprocess

from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
from typing import List, Optional

import yaml

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node, Service
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


class DataloggerNode(Node):

    def __init__(self: DataloggerNode, name: str, **kwargs) -> None:

        super().__init__(name, **kwargs)

        self._start_recording_service: Optional[Service] = None

        self._stop_recording_service: Optional[Service] = None

        self._command: Optional[List[str]] = None

        self._is_recording = False

        self.rosbag_process: Optional[subprocess.Popen] = None

        self._configure()

        self._activate()

    def _configure(self: DataloggerNode) -> None:

        self._get_parameters()

        self._initialize()

    def _get_parameters(self: DataloggerNode) -> None:

        self.declare_parameter("path", "/ROS/bags")
        self.path: Path = Path.home() / self.get_parameter(
            "path").get_parameter_value().string_value

        self.declare_parameter("topics", [""])
        self._topics: str = (self.get_parameter(
            "topics").get_parameter_value().string_array_value)
        self._topics = [] + self._topics

        self.declare_parameter("regex", "/**")
        self._regex: str = (
            self.get_parameter("regex").get_parameter_value().string_value)

        self.declare_parameter("excludes", "")
        self._excludes: str = (
            self.get_parameter("excludes").get_parameter_value().string_value)

        self.declare_parameter("storage", "mcap")
        self._storage: str = (
            self.get_parameter("storage").get_parameter_value().string_value)

        self.declare_parameter("compression.format", "none")
        self._compression_format: str = (self.get_parameter(
            "compression.format").get_parameter_value().string_value)

        self.declare_parameter("max_cache_size", 500)
        self._max_cache_size: str = (self.get_parameter(
            "max_cache_size").get_parameter_value().integer_value)

        self.declare_parameter("record_on_start", False)
        self._record_on_start: bool = (self.get_parameter(
            "record_on_start").get_parameter_value().bool_value)

        self.declare_parameter("name", "")
        self._bag_name: str = (
            self.get_parameter("name").get_parameter_value().string_value)

    def _on_set_parameters_callback(self, parameters):

        for parameter in parameters:
            if parameter.name == "topics" and parameter.type_ == Parameter.Type.STRING_ARRAY:
                self._topics = parameter.value
                if self._is_recording:
                    self.get_logger().warning(
                        "The datalogger is currently recording: your new topic list will take effect on the next record action."
                    )
            self.get_logger().info(
                f"Updated parameter \"{parameter.name}\" with value \"{parameter.value}\"."
            )
        return SetParametersResult(successful=True)

    def _initialize(self: DataloggerNode) -> None:

        self._load_filename_pairs()

        self._define_command()

    def _load_filename_pairs(self: DataloggerNode) -> None:

        filename_pairs_path: Path = Path(
            importlib.resources.files("olav_utilities").joinpath(
                "data/datalogger/config/filenames.yaml"))

        with open(filename_pairs_path, "r",
                  encoding="utf-8") as filename_pairs_file:

            filename_pairs_dictionary = yaml.safe_load(filename_pairs_file)

            self._filename_names = filename_pairs_dictionary["names"]
            self._filename_adjectives = filename_pairs_dictionary["adjectives"]

    def _define_command(self: DataloggerNode) -> None:

        self.command = ["ros2", "bag", "record"]

        self.command += ["-e"] + [self._regex]

        self.command += ["-x"] + [self._excludes]

        self.command += ["--storage"] + [self._storage]
        if self._storage == "mcap":
            self.command += ["--storage-config-file"] + [
                Path(
                    importlib.resources.files("olav_utilities").joinpath(
                        "data/datalogger/config/mcap_storage_defaults.yaml")).
                as_posix()
            ]

        self.command += ["--max-cache-size"] + [str(self._max_cache_size)]
        if self._compression_format != "none" and self._storage != "mcap":
            self.command += ["--compression-mode", "file"] + [
                "--compression-format", self._compression_format
            ]

    def _activate(self: DataloggerNode) -> None:

        self._create_services()

        self._create_additional_callbacks()

        self._run_post_activate_hooks()

    def _create_services(self: DataloggerNode) -> None:

        self._start_recording_service = self.create_service(
            Trigger, "start", self._start_recording)

        self._stop_recording_service = self.create_service(
            Trigger, "stop", self._stop_recording)

    def _create_additional_callbacks(self: DataloggerNode) -> None:

        self.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def _run_post_activate_hooks(self: DataloggerNode) -> None:

        if self._record_on_start:

            assert self._start_recording(Trigger.Request(),
                                         Trigger.Response()).success

    def _start_recording(self: DataloggerNode, request: Trigger.Request,
                         response: Trigger.Response) -> Trigger.Response:

        _ = request

        if self._is_recording:
            message: str = "Could not start a new recording: an existing recording is currently in progress."

            self.get_logger().warning(message)

            response.success = False
            response.message = message

            return response

        if not pathlib.Path(self.path).exists():
            self.get_logger().warning(
                f"Path {self.path} did not exist, creating a new folder tree..."
            )
        pathlib.Path(self.path).mkdir(parents=True, exist_ok=True)

        name_index = random.randint(0, len(self._filename_names) - 1)
        adjective_index = random.randint(0, len(self._filename_adjectives) - 1)

        # Instantiate a virtual TTY to avoid ROS bag warnings on STDIN device
        # availability.
        _, pty_slave = pty.openpty()

        if self._bag_name != "":
            bag_name = self._bag_name
        else:
            bag_name: str = datetime.now(
                ZoneInfo("Europe/Copenhagen")
            ).strftime("%Y-%m-%d-%H-%M-%S") + "-" + self._filename_adjectives[
                adjective_index] + "-" + self._filename_names[name_index]

        command = self.command + [
            "--output",
            bag_name,
        ]
        if not "" in self._topics: command += self._topics
        else: command += ["--all"]

        self.rosbag_process = subprocess.Popen(
            command,
            cwd=self.path,
            stdin=pty_slave,
            stdout=subprocess.DEVNULL,
        )
        self._is_recording = True

        self.get_logger().info(
            f"Started recorder with process ID {self.rosbag_process.pid}")

        response.success = True
        response.message = f"Started recorder with process ID {self.rosbag_process.pid}"
        return response

    def _stop_recording(self: DataloggerNode, request: Trigger.Request,
                        response: Trigger.Response) -> Trigger.Response:
        _ = request

        if self._is_recording:
            self.get_logger().info(
                f"Sending SIGINT to the recording process...")
            self.rosbag_process.send_signal(signal.SIGINT)
            self.rosbag_process.wait()
            self._is_recording = False

        response.success = True
        response.message = "Succesfully stopped recorder."
        return response
