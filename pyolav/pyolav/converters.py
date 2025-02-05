from __future__ import annotations

from typing import Optional
from pathlib import Path

import logging

import tables
import numpy as np

import rosbags
from rosbags.rosbag2 import Reader as BagReader


class Converter:

    def __init__(self: Converter) -> None:

        self._logger = self._get_logger()
        self._logger.setLevel(logging.INFO)
        self._bag_path: Optional[Path] = None
        self._bag_typestore = self._setup_typestore()

    def _get_logger(self: Converter) -> logging.Logger:

        return logging.getLogger(__name__)

    def _setup_typestore(self: Converter, path: Optional[str] = None) -> None:

        return rosbags.typesys.get_typestore(rosbags.typesys.Stores.ROS2_HUMBLE)

    def load(self: Converter, path: str) -> None:

        path: Path = Path(path)
        self._logger.info("Loading bag at \"%s\" ...", path.as_posix())

        self._bag_path = path
        self._bag_reader: BagReader = BagReader(path)
        self._bag_reader.open()

        self._logger.info("Loaded bag \"%s\"", self._get_bag_name())
        self._get_bag_topics()

    def _get_bag_name(self: Converter) -> str:

        return " ".join(self._bag_path.stem.rsplit("-", 2)[-2:]).title()

    def _get_bag_topics(self: Converter) -> str:

        # Notify the user of the available topics.
        self._bag_topics = [(connection.topic, connection.msgtype)
                            for connection in self._bag_reader.connections]
        for topic, message_type in self._bag_topics:
            self._logger.info("Found topic: \"%s\" with type \"%s\"", topic,
                              message_type)

    def _get_topic_connections(self: Converter, topic_name: str):

        return [
            connection for connection in self._bag_reader.connections
            if connection.topic == topic_name
        ]

    def _get_messages_count(self: Converter, topic_name: str) -> int:

        return len(
            list(
                self._bag_reader.messages(
                    self._get_topic_connections(topic_name))))

    def _load_topic(self: Converter, topic_name: str) -> np.array:

        timestamps = np.zeros((self._get_messages_count(topic_name), ))
        points = np.zeros((self._get_messages_count(topic_name), ))

        for index, (connection, timestamp, data) in enumerate(
                self._bag_reader.messages(
                    self._get_topic_connections(topic_name))):

            message = self._bag_typestore.deserialize_cdr(
                data, connection.msgtype)

            timestamps[index] = timestamp
            points[index] = message.data

        self._map = {}
        self._map[topic_name] = {}
        self._map[topic_name]["timestamps"] = timestamps / 1.0e9 - timestamps[0] / 1.0e9
        self._map[topic_name]["data"] = points

        return timestamps, points

    def save_hdf5(self: Converter, path: str) -> None:

        path: Path = Path(path)

        self._logger.info("Saving bag to HDF5 file \"%s\"", path.as_posix())

        file = tables.open_file(path, mode="w", title=self._get_bag_name())
        group = file.create_group("/", "my_group", "My group")

        for key, value in self._map.items():
            file.create_group("/my_group", "quantity")
            for entry, data in value.items():
                file.create_array("/my_group/quantity", entry, data)

        file.close()
