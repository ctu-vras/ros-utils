# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Efficient data structure to hold a static set of topics with super-fast is-in-set queries.

The querying is done using expressions like `topic in set` or `topic not in set`.
"""


import sys
from typing import AnyStr, Iterable

import rospy


def is_time_relative(stamp):
    """Decide whether the given time instant is relative or absolute.

    :param rospy.Time stamp: The time to test.
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
        :type start: rospy.Time or float
        :param end_or_duration: The end time of the time range, or a duration from start. If a rospy.Duration object is
                                passed, it means relative duration from the given start time. If another type is passed
                                and is small enough to be treated as relative time, it means time relative to the given
                                base time base set in set_base_time() (or relative to the given start time
                                if set_base_time has not been called).
        :type end_or_duration: rospy.Time or rospy.Duration or float
        """
        self._start = start if isinstance(start, rospy.Time) else rospy.Time(start)
        if isinstance(end_or_duration, rospy.Duration):
            self._end = self._start + end_or_duration
        else:
            self._end = end_or_duration if isinstance(end_or_duration, rospy.Time) else rospy.Time(end_or_duration)

        self._abs_start = self._start
        self._abs_end = self._end
        if is_time_relative(self._end) and not is_time_relative(self._start):
            self._end += rospy.Duration(self._start.secs, self._start.nsecs)

        self._base_time = None
        self._base_time_duration = None

    def __contains__(self, stamp):
        """Whether the given absolute stamp is inside this time range.

        :param stamp: The queried stamp. It can be absolute or relative (if set_base_time has been called).
        :type stamp: rospy.Time or float
        :return: Whether the given absolute stamp is inside this time range.
        :rtype: bool
        """
        if not isinstance(stamp, rospy.Time):
            stamp = rospy.Time(stamp)
        if self._base_time_duration is not None and is_time_relative(stamp):
            stamp += self._base_time_duration

        return self._abs_start <= stamp < self._abs_end

    def set_base_time(self, stamp):
        """Set the base time to which all relative values are related.

        :param stamp: The base time.
        :type stamp: rospy.Time or float
        """
        self._base_time = stamp if isinstance(stamp, rospy.Time) else rospy.Time(stamp)
        self._base_time_duration = rospy.Duration(self._base_time.secs, self._base_time.nsecs)
        if is_time_relative(self._start):
            self._abs_start = self._start + self._base_time_duration
        if is_time_relative(self._end):
            self._abs_end = self._end + self._base_time_duration

    def __str__(self):
        if self._base_time is None:
            return "%s - %s" % (str(self._start), str(self._end))
        return "%s - %s (since %s)" % (str(self._start), str(self._end), str(self._base_time))

    def __repr__(self):
        return self.__str__()
