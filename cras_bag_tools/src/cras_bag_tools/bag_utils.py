# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with bag files."""

import copy
import heapq
import os
from typing import Any, Callable, Dict, Iterable, Iterator, Optional, Sequence, Tuple, Union

import genpy
import rosbag

from cras.string_utils import STRING_TYPE

from .time_range import TimeRanges


ConnectionFilter = Callable[
    [
        STRING_TYPE,  # topic
        STRING_TYPE,  # datatype
        STRING_TYPE,  # MD5 sum
        STRING_TYPE,  # message definition
        Dict[STRING_TYPE, STRING_TYPE]  # connection header
    ],
    bool]  # whether the connection should be accepted
ConnectionInfo = rosbag.bag._ConnectionInfo  # noqa
ConnectionEntry = rosbag.bag._IndexEntry  # noqa


class BagWrapper(object):
    """Wrapper of already open rosbag.Bag that adds the ability to iterate messages with a TimeRanges filter."""
    def __init__(self, bag):  # type: (rosbag.Bag) -> None
        self.bag = bag

        self._orig_get_entries = bag._get_entries  # noqa
        self._orig_get_entries_reverse = bag._get_entries_reverse  # noqa

        self.bag._get_entries = self._get_entries
        self.bag._get_entries_reverse = self._get_entries_reverse

    def __del__(self):
        self.bag._get_entries = self._orig_get_entries
        self.bag._get_entries_reverse = self._orig_get_entries_reverse

    def __getattr__(self, item):
        return getattr(self.bag, item)

    def read_messages(self, topics, start_time, end_time, topic_filter, raw, return_connection_header=False):
        if topics is not None and len(topics) == 0:
            return
        for msg in self.bag.read_messages(topics, start_time, end_time, topic_filter, raw, return_connection_header):
            yield msg

    def _get_entries(self, connections=None, start_time=None, end_time=None):
        all_ranges = None
        if start_time is not None and isinstance(start_time, TimeRanges):
            all_ranges = copy.copy(start_time)
        if end_time is not None and isinstance(end_time, TimeRanges):
            if all_ranges is None:
                all_ranges = copy.copy(end_time)
            else:
                all_ranges.append(end_time.ranges)

        for entry in heapq.merge(*self.bag._get_indexes(connections), key=lambda x: x.time.to_nsec()):  # noqa
            if all_ranges is not None:
                if all_ranges > entry.time:
                    continue
                elif all_ranges < entry.time:
                    return
                elif entry.time not in all_ranges:
                    continue
            else:
                if start_time is not None and isinstance(start_time, genpy.Time) and entry.time < start_time:
                    continue
                if end_time is not None and isinstance(end_time, genpy.Time) and entry.time > end_time:
                    return
            yield entry

    def _get_entries_reverse(self, connections=None, start_time=None, end_time=None):
        all_ranges = None
        if start_time is not None and isinstance(start_time, TimeRanges):
            all_ranges = copy.copy(start_time)
        if end_time is not None and isinstance(end_time, TimeRanges):
            if all_ranges is None:
                all_ranges = copy.copy(end_time)
            else:
                all_ranges.append(end_time.ranges)

        for entry in heapq.merge(*(reversed(index) for index in self._get_indexes(connections)),
                                 key=lambda x: x.time.to_nsec(), reverse=True):
            if all_ranges is not None:
                if all_ranges > entry.time:
                    return
                elif all_ranges < entry.time:
                    continue
                elif entry.time not in all_ranges:
                    continue
            else:
                if start_time is not None and isinstance(start_time, genpy.Time) and entry.time < start_time:
                    return
                if end_time is not None and isinstance(end_time, genpy.Time) and entry.time > end_time:
                    continue
            yield entry


class MultiBag(object):
    """Generalization of Bag which can read multiple bags simultaneously."""

    def __init__(self,
                 bag_files,  # type: Union[Sequence[rosbag.Bag], STRING_TYPE]
                 mode='r',  # type: STRING_TYPE
                 compression=rosbag.Compression.NONE,  # type: rosbag.Compression
                 options=None,  # type: Dict[STRING_TYPE, Any]
                 skip_index=False,  # type: bool
                 limit_to_first_bag=False,  # type: bool
                 ):
        """
        :param bag_files: The paths to bags to open (either a sequence of colon-separated string of paths).
        :param mode: Open mode (r/w/a).
        :param compression: Compression (used for write mode).
        :param options: Bag options (compression, chunk threshold, ...).
        :param skip_index: Whether index should be read right away. Otherwise, call read_index() when you need it.
        :param limit_to_first_bag: If True, the multibag will report its start and end to be equal to the
                                   first open bag. If False, the start and end correspond to the earliest and latest
                                   stamp in all bags.
        """

        if isinstance(bag_files, STRING_TYPE):
            bag_files = bag_files.split(os.path.pathsep)
        self.bags = [self.open_bag(b, compression, mode, options, skip_index) for b in bag_files]
        self._limit_to_first_bag = limit_to_first_bag

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
        if self._limit_to_first_bag:
            return self.bags[0].size
        return sum(b.size for b in self.bags)

    def close(self):
        if hasattr(self, 'bags'):
            for b in self.bags:
                b.close()

    def get_message_count(self,
                          topic_filters=None,  # type: Optional[Sequence[STRING_TYPE]]
                          start_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                          end_time=None  # type: Optional[Union[genpy.Time, TimeRanges]]
                          ):
        # type: (...) -> int
        connections = dict(self._get_connections(topic_filters, with_bag=True))
        entries = self._get_entries(connections, start_time, end_time)
        return sum(1 for _ in entries)

    def get_start_time(self):  # type: () -> float
        if self._limit_to_first_bag:
            return self.bags[0].get_start_time()
        return min(b.get_start_time() for b in self.bags)

    def get_end_time(self):  # type: () -> float
        if self._limit_to_first_bag:
            return self.bags[0].get_end_time()
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
                     start_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                     end_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                     ):
        # type: (...) -> Iterator[Tuple[rosbag.Bag, ConnectionEntry, ConnectionInfo]]
        all_indexes = []
        for bag in self.bags:
            conns = list(connections[bag] if connections is not None else bag._get_connections())  # noqa
            indexes = bag._get_indexes(conns)  # noqa
            for conn, index in zip(conns, indexes):
                all_indexes.append([(bag, entry, conn) for entry in index])

        if start_time is None and self._limit_to_first_bag:
            start_time = genpy.Time(self.get_start_time())
        if end_time is None and self._limit_to_first_bag:
            end_time = genpy.Time(self.get_end_time())

        time_ranges = None
        if start_time is not None and isinstance(start_time, TimeRanges):
            time_ranges = {}
            for bag in self.bags:
                time_range = copy.copy(start_time)
                time_range.set_base_time(genpy.Time(bag.get_start_time()))
                time_ranges[bag] = time_range
            start_time = None  # Simplify the check in the next loop

        extra_time_ranges = None
        if end_time is not None and isinstance(end_time, TimeRanges):
            extra_time_ranges = {}
            for bag in self.bags:
                time_range = copy.copy(end_time)
                time_range.set_base_time(genpy.Time(bag.get_start_time()))
                extra_time_ranges[bag] = time_range
            end_time = None  # Simplify the check in the next loop

        for bag, entry, conn in heapq.merge(*all_indexes, key=lambda x: x[1].time.to_nsec()):
            if extra_time_ranges is not None and entry.time in extra_time_ranges[bag]:
                yield bag, entry, conn
                continue

            if start_time is not None and entry.time < start_time:
                continue
            if end_time is not None and entry.time > end_time:
                return
            if time_ranges is not None:
                time_range = time_ranges[bag]
                if time_range > entry.time:
                    continue
                elif time_range < entry.time:
                    return
                elif entry.time not in time_range:
                    continue

            yield bag, entry, conn

    def _read_messages(self,
                       topics=None,  # type: Optional[Sequence[STRING_TYPE]]
                       start_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                       end_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                       connection_filter=None,  # type: Optional[ConnectionFilter]
                       raw=False,  # type: bool
                       return_connection_header=False,  # type: bool
                       ):
        # type: (...) -> Iterator[Union[rosbag.bag.BagMessage, rosbag.bag.BagMessageWithConnectionHeader]]
        if topics is not None and len(topics) == 0:
            return
        connections = dict(self._get_connections(topics, connection_filter, True))
        for bag, entry, _ in self._get_entries(connections, start_time, end_time):
            msg = bag._reader.seek_and_read_message_data_record(  # noqa
                (entry.chunk_pos, entry.offset), raw, return_connection_header)
            yield msg

    def read_messages(self,
                      topics=None,  # type: Optional[Sequence[STRING_TYPE]]
                      start_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                      end_time=None,  # type: Optional[Union[genpy.Time, TimeRanges]]
                      connection_filter=None,  # type: Optional[ConnectionFilter]
                      raw=False,  # type: bool
                      return_connection_header=False,  # type: bool
                      ):
        # type: (...) -> Iterator[Union[rosbag.bag.BagMessage, rosbag.bag.BagMessageWithConnectionHeader]]
        return self._read_messages(topics, start_time, end_time, connection_filter, raw, return_connection_header)


__all__ = [
    BagWrapper.__name__,
    MultiBag.__name__,
]
