# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with bag files."""

import heapq
import os

import rosbag

from cras.string_utils import STRING_TYPE


class MultiBag(object):
    """Generalization of Bag which can read multiple bags simultaneously."""

    def __init__(self, bag_files, mode='r', compression=rosbag.Compression.NONE, options=None, skip_index=False):
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

    def get_start_time(self):
        return min(b.get_start_time() for b in self.bags)

    def get_end_time(self):
        return max(b.get_end_time() for b in self.bags)

    def read_index(self):
        for b in self.bags:
            if not b._connection_indexes_read and b._version == 200:  # noqa
                b._reader._read_connection_index_records()  # noqa

    def _get_connections(self, topics=None, connection_filter=None, with_bag=False):
        for bag in self.bags:
            if with_bag:
                yield bag, bag._get_connections(topics, connection_filter)  # noqa
            else:
                for connection in bag._get_connections(topics, connection_filter):  # noqa
                    yield connection

    def _get_entries(self, connections=None, start_time=None, end_time=None):
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

    def _read_messages(self, topics=None, start_time=None, end_time=None, connection_filter=None, raw=False,
                       return_connection_header=False):
        connections = dict(self._get_connections(topics, connection_filter, True))
        for bag, entry, _ in self._get_entries(connections, start_time, end_time):
            msg = bag._reader.seek_and_read_message_data_record(  # noqa
                (entry.chunk_pos, entry.offset), raw, return_connection_header)
            yield msg

    def read_messages(self, topics=None, start_time=None, end_time=None, connection_filter=None, raw=False,
                      return_connection_header=False):
        return self._read_messages(topics, start_time, end_time, connection_filter, raw, return_connection_header)


__all__ = [
    MultiBag.__name__
]
