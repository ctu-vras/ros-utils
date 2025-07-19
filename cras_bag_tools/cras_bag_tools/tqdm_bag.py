# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Bag file reader that shows progressbars when reading messages."""

import os
import rosbag2_py
from tqdm import tqdm
from typing import Optional, Tuple, Union, Any, Generator


class BagIterator(rosbag2_py.SequentialReader):
    """Utility class for easier working with ROS2 BAG files."""

    def __init__(self, storage_options: Union[rosbag2_py.StorageOptions, str],
                 converter_options: rosbag2_py.ConverterOptions = rosbag2_py.ConverterOptions("", "")):
        """Construct the bag iterator and give it arguments for opening the bag, but do not open it yet.

        :param storage_options: Storage options.
        :param converter_options: Converter options.
        """
        super().__init__()

        if isinstance(storage_options, str):
            storage_options = rosbag2_py.StorageOptions(storage_options)

        self._storage_options = storage_options
        self._converter_options = converter_options

    def read_messages(self, topics: Optional[list[str]] = None,
                      start_time: Optional[int] = None, end_time: Optional[int] = None) -> \
            Generator[tuple[str, bytes, int], Any, None]:
        """Iterate through messages in the bag.

        :param topics: If nonempty, this is the list of topics to filter by.
        :param start_time: If set, this is the time of the first read message (in nanoseconds since epoch).
        :param end_time: If set, this is the time of the last read message (in nanoseconds since epoch).
        :return: Iterator over messages. Returns tuples (topic, serialized message byte content, receive stamp (nanos)).
        """
        if topics is not None and len(topics) > 0:
            self.set_filter(rosbag2_py.StorageFilter(topics))

        if start_time is not None:
            self.seek(start_time)

        while self.has_next():
            topic, data, t = self.read_next()
            if end_time is not None and t > end_time:
                break
            yield topic, data, t

        if topics is not None:
            self.reset_filter()

    def __enter__(self):
        self.open(self._storage_options, self._converter_options)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class TqdmBag(BagIterator):
    """Extension of BagIterator which uses tqdm to report progress."""

    def read_messages(self, topics: Optional[list[str]] = None,
                      start_time: Optional[int] = None, end_time: Optional[int] = None) ->\
            Union[Generator[tuple[str, bytes, int], Any, None], tqdm]:
        basename = os.path.basename(self._storage_options.uri)

        msg_count = self.get_metadata().message_count
        if topics is not None:
            msg_count = 0
            for t in self.get_metadata().topics_with_message_count:
                if t.topic_metadata.name in topics:
                    msg_count += t.message_count

        # if only a part of the bag is to be read, estimate the number of contained images by interpolation
        if start_time is not None or end_time is not None:
            start = self.get_metadata().starting_time if start_time is None else start_time
            end = self.get_metadata().starting_time + self.get_metadata().duration if end_time is None else end_time
            duration = end - start
            msg_count *= float(duration) / self.get_metadata().duration

        return tqdm(super().read_messages(topics, start_time, end_time), total=msg_count, desc=basename, unit='msg')
