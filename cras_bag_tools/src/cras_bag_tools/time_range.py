# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Representation of a time range."""

from typing import Iterable, Union

import genpy


def is_time_relative(stamp):
    """Decide whether the given time instant is relative or absolute.

    :param genpy.Time stamp: The time to test.
    :return: Whether the time is relative or absolute.
    """
    return stamp.secs < 1600000000


class TimeRange(object):
    """Representation of a time range (including start time, excluding end time).

    Start and end times can be both absolute (UNIX time) or relative (e.g. since bag start). All values lower than
    1.6e9 are treated as relative. Even rospy.Time objects with these lower values are considered to be relative.
    """

    def __init__(self, start, end_or_duration):
        """
        :param start: The start time of the time range.
        :type start: genpy.Time or float
        :param end_or_duration: The end time of the time range, or a duration from start. If a rospy.Duration object is
                                passed, it means relative duration from the given start time. If another type is passed
                                and is small enough to be treated as relative time, it means time relative to the given
                                base time base set in set_base_time() (or relative to the given start time
                                if set_base_time has not been called).
        :type end_or_duration: genpy.Time or genpy.Duration or float
        """
        self.start = start if isinstance(start, genpy.Time) else genpy.Time(start)
        if isinstance(end_or_duration, genpy.Duration):
            self.end = self.start + end_or_duration
        else:
            self.end = end_or_duration if isinstance(end_or_duration, genpy.Time) else genpy.Time(end_or_duration)

        self.abs_start = self.start
        self.abs_end = self.end
        if is_time_relative(self.end) and not is_time_relative(self.start):
            self.end += genpy.Duration(self.start.secs, self.start.nsecs)

        self._base_time = None
        self._base_time_duration = None

    def __contains__(self, stamp):
        """Whether the given absolute stamp is inside this time range.

        :param stamp: The queried stamp. It can be absolute or relative (if set_base_time has been called).
        :type stamp: genpy.Time or float
        :return: Whether the given absolute stamp is inside this time range.
        :rtype: bool
        """
        if not isinstance(stamp, genpy.Time):
            stamp = genpy.Time(stamp)
        if self._base_time_duration is not None and is_time_relative(stamp):
            stamp += self._base_time_duration

        return self.abs_start <= stamp < self.abs_end

    def __eq__(self, other):
        if not isinstance(other, TimeRange):
            return NotImplemented

        return self.abs_start == other.abs_start and self.abs_end == other.abs_end

    def __lt__(self, other):
        if isinstance(other, TimeRange):
            return (self.abs_start < other.abs_start or
                    (self.abs_start == other.abs_start and self.abs_end < other.abs_end))
        elif isinstance(other, genpy.Time):
            return other > self.abs_end
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, TimeRange):
            return self.abs_end > other.abs_end or (self.abs_end == other.abs_end and self.abs_start > other.abs_start)
        elif isinstance(other, genpy.Time):
            return other < self.abs_start
        return NotImplemented

    def overlaps(self, other):  # type: (TimeRange) -> bool
        min_range = min(self, other)
        max_range = max(self, other)
        return max_range.abs_start in min_range

    def get_merged(self, other):  # type: (TimeRange) -> TimeRange
        base_time = self._base_time if self._base_time is not None else other._base_time  # noqa

        def get_rel_time(t):
            return t if is_time_relative(t) or base_time is None or t <= base_time else t - base_time

        start = min(get_rel_time(self.start), get_rel_time(other.start))
        end = max(get_rel_time(self.end), get_rel_time(other.end))
        result = TimeRange(start, end)

        if base_time is not None:
            result.set_base_time(base_time)
        return result

    def set_base_time(self, stamp):
        """Set the base time to which all relative values are related.

        :param stamp: The base time.
        :type stamp: genpy.Time or float
        """
        self._base_time = stamp if isinstance(stamp, genpy.Time) else genpy.Time(stamp)
        self._base_time_duration = genpy.Duration(self._base_time.secs, self._base_time.nsecs)
        if is_time_relative(self.start):
            self.abs_start = self.start + self._base_time_duration
        if is_time_relative(self.end):
            self.abs_end = self.end + self._base_time_duration

    def __str__(self):
        if self._base_time is None:
            return "%s - %s" % (str(self.start), str(self.end))
        return "%s - %s (since %s)" % (str(self.start), str(self.end), str(self._base_time))

    def __repr__(self):
        return self.__str__()


class TimeRanges(object):
    """Representation of a union of time ranges."""

    def __init__(self, ranges):
        # type: (Iterable[TimeRange]) -> None
        self.ranges = []
        self.append(ranges)

    def set_base_time(self, stamp):
        """Set the base time to which all relative values are related.

        :param stamp: The base time.
        :type stamp: genpy.Time or float
        """
        for r in self.ranges:
            r.set_base_time(stamp)

    def append(self, time_range):  # type: (Union[TimeRange, Iterable[TimeRange]]) -> None
        ranges = list(self.ranges)
        try:
            ranges += time_range
        except TypeError:
            ranges.append(time_range)

        self.ranges = []
        # Sort and merge overlapping ranges so that we only store a growing sequence of non-overlapping ranges
        for r in sorted(ranges):
            if len(self.ranges) == 0 or not r.overlaps(self.ranges[-1]):
                self.ranges.append(r)
            else:
                self.ranges[-1] = self.ranges[-1].get_merged(r)

    def __contains__(self, item):
        return any(item in r for r in self.ranges)

    def __lt__(self, other):
        if isinstance(other, genpy.Time):
            return all(r < other for r in self.ranges)
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, genpy.Time):
            return all(r > other for r in self.ranges)
        return NotImplemented

    def __str__(self):
        return "(%s)" % (", ".join(map(str, self.ranges)),)

    def __repr__(self):
        return self.__str__()


__all__ = [
    is_time_relative.__name__,
    TimeRange.__name__,
    TimeRanges.__name__,
]
