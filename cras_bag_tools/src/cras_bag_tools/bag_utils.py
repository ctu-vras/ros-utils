# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with bag files."""

import heapq
import os
from typing import Any, Callable, Dict, Iterable, Iterator, Optional, Sequence, Tuple, Union

import genpy
import rosbag

from cras.string_utils import STRING_TYPE


ConnectionFilter = Callable[[STRING_TYPE, STRING_TYPE, STRING_TYPE, STRING_TYPE, Dict[STRING_TYPE, STRING_TYPE]], bool]
ConnectionInfo = rosbag.bag._ConnectionInfo  # noqa
ConnectionEntry = rosbag.bag._IndexEntry  # noqa


class MultiBag(object):
    """Generalization of Bag which can read multiple bags simultaneously."""

    def __init__(self,
                 bag_files,  # type: Union[Sequence[rosbag.Bag], STRING_TYPE]
                 mode='r',  # type: STRING_TYPE
                 compression=rosbag.Compression.NONE,  # type: rosbag.Compression
                 options=None,  # type: Dict[STRING_TYPE, Any]
                 skip_index=False  # type: bool
                 ):

        if isinstance(bag_files, STRING_TYPE):
            bag_files = bag_files.split(os.path.pathsep)
        self.bags = [self.open_bag(b, compression, mode, options, skip_index) for b in bag_files]

    def open_bag(self, b, compression, mode, options, skip_index):
        return rosbag.Bag(b, mode, compression, options=options, skip_index=skip_index)

    def __iter__(self):
        return self.read_messages()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        self.close()

    @property
    def size(self):
        return sum(b.size for b in self.bags)

    def close(self):
        if hasattr(self, 'bags'):
            for b in self.bags:
                b.close()

    def get_message_count(self, topic_filters=None, start_time=None, end_time=None):
        # type: (Optional[Sequence[STRING_TYPE]], Optional[genpy.Time], Optional[genpy.Time]) -> int
        start = start_time
        if start is None:
            start = self.get_start_time()
        start = max(start, self.get_start_time())

        end = end_time
        if end is None:
            end = self.get_end_time()
        end = min(end, self.get_end_time())

        connections = dict(self._get_connections(topic_filters, with_bag=True))
        entries = self._get_entries(connections, start, end)
        return sum(1 for _ in entries)

    def get_start_time(self):  # type: () -> float
        return min(b.get_start_time() for b in self.bags)

    def get_end_time(self):  # type: () -> float
        return max(b.get_end_time() for b in self.bags)

    def read_index(self):
        for b in self.bags:
            if not b._connection_indexes_read and b._version == 200:  # noqa
                b._reader._read_connection_index_records()  # noqa

    def _get_connections(self, topics=None, connection_filter=None, with_bag=False):
        # type: (Optional[Sequence[STRING_TYPE]], Optional[ConnectionFilter], bool) -> Iterator[ConnectionInfo]
        for bag in self.bags:
            if with_bag:
                yield bag, bag._get_connections(topics, connection_filter)  # noqa
            else:
                for connection in bag._get_connections(topics, connection_filter):  # noqa
                    yield connection

    def _get_entries(self,
                     connections=None,  # type: Optional[Dict[rosbag.Bag, Iterable[ConnectionInfo]]]
                     start_time=None,  # type: Optional[genpy.Time]
                     end_time=None,  # type: Optional[genpy.Time]
                     ):
        # type: (...) -> Iterator[Tuple[rosbag.Bag, ConnectionEntry, ConnectionInfo]]
        all_indexes = []
        for bag in self.bags:
            conns = list(connections[bag] if connections is not None else bag._get_connections())  # noqa
            indexes = bag._get_indexes(conns)  # noqa
            for conn, index in zip(conns, indexes):
                all_indexes.append([(bag, entry, conn) for entry in index])

        for bag, entry, conn in heapq.merge(*all_indexes, key=lambda x: x[1].time.to_nsec()):
            if start_time and entry.time.to_sec() < start_time:
                continue
            if end_time and entry.time.to_sec() > end_time:
                return
            yield bag, entry, conn

    def _read_messages(self,
                       topics=None,  # type: Optional[Sequence[STRING_TYPE]]
                       start_time=None,  # type: Optional[genpy.Time]
                       end_time=None,  # type: Optional[genpy.Time]
                       connection_filter=None,  # type: Optional[ConnectionFilter]
                       raw=False,  # type: bool
                       return_connection_header=False,  # type: bool
                       ):
        # type: (...) -> Iterator[Union[rosbag.bag.BagMessage, rosbag.bag.BagMessageWithConnectionHeader]]
        connections = dict(self._get_connections(topics, connection_filter, True))
        for bag, entry, _ in self._get_entries(connections, start_time, end_time):
            msg = bag._reader.seek_and_read_message_data_record(  # noqa
                (entry.chunk_pos, entry.offset), raw, return_connection_header)
            yield msg

    def read_messages(self,
                      topics=None,  # type: Optional[Sequence[STRING_TYPE]]
                      start_time=None,  # type: Optional[genpy.Time]
                      end_time=None,  # type: Optional[genpy.Time]
                      connection_filter=None,  # type: Optional[ConnectionFilter]
                      raw=False,  # type: bool
                      return_connection_header=False,  # type: bool
                      ):
        # type: (...) -> Iterator[Union[rosbag.bag.BagMessage, rosbag.bag.BagMessageWithConnectionHeader]]
        return self._read_messages(topics, start_time, end_time, connection_filter, raw, return_connection_header)


__all__ = [
    MultiBag.__name__
]
