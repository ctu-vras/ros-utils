# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when working with rospy Time, Duration and Rate objects.

Parts of this library are adapted from BSD-licensed
https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/timer.py
Copyright (c) 2010, Willow Garage, Inc.
"""

import genpy
import time

try:
    # Python 2
    from monotonic import monotonic
except ImportError:
    # Python 3
    # noinspection PyUnresolvedReferences
    from time import monotonic

import rospy
from rospy import Time, Duration, Rate

DURATION_MAX = Duration(2 ** 31 - 1, 999999999)
"""Maximum value of rospy.Duration that can be read without problems by other ROS client libraries."""
DURATION_MIN = Duration(-2 ** 31, 0)
"""Minimum value of rospy.Duration that can be read without problems by other ROS client libraries."""

TIME_MAX = Time(2 ** 32 - 1, 999999999)
"""Maximum value of rospy.Time that can be read without problems by other ROS client libraries."""
TIME_MIN = Time(0, 1)
"""Minimum value of rospy.Time that can be read without problems by other ROS client libraries. Zero is not included."""


def slowest_rate():
    """Slowest representable rate.

    :return: Slowest representable rate.
    :rtype: Rate
    """
    rate = Rate(1)
    rate.sleep_dur = DURATION_MAX
    return rate


def slowest_negative_rate():
    """Slowest representable negative rate.

    :return: Slowest representable negative rate.
    :rtype: Rate
    """
    rate = Rate(1)
    rate.sleep_dur = DURATION_MIN
    return rate


def frequency(rate, max_cycle_time_means_zero=False):
    """Return the frequency represented by the given rate.

    :param Rate rate: The rate to convert.
    :param max_cycle_time_means_zero: If True, return 0 frequency in case the rate's cycle time is the maximum duration.
    :return: The frequency.
    :rtype: float
    """
    if max_cycle_time_means_zero and (rate.sleep_dur == DURATION_MAX or rate.sleep_dur == DURATION_MIN):
        return 0.0
    return 1.0 / rate.sleep_dur.to_sec()


def safe_rate(freq):
    """Return a rate representing the given frequency. If the frequency is zero or too small, return min/max
    representable rate.

    :param float freq: The frequency to convert.
    :return: The corresponding Rate object.
    :rtype: Rate
    """
    if freq == 0:
        return slowest_rate()
    rate = Rate(freq)
    if rate.sleep_dur > DURATION_MAX:
        rate.sleep_dur = DURATION_MAX
    elif rate.sleep_dur < DURATION_MIN:
        rate.sleep_dur = DURATION_MIN
    return rate


def rate_equals(rate1, rate2):
    """Compare two rates for equality.

    :param Rate rate1: Rate.
    :param Rate rate2: Rate.
    :return: Whether the durations of the rates are equal.
    :rtype: bool
    """
    return rate1.sleep_dur == rate2.sleep_dur


def wallsleep(duration):
    """Sleep for the given duration in wall clock.

    :note: The duration of the sleep is not affected by system time jumps (as if sleeping in monotonic clock).
    :param Duration|float duration: The duration to sleep for.
    """
    if isinstance(duration, genpy.Duration):
        duration = duration.to_sec()
    if duration < 0:
        return
    else:
        rospy.rostime.wallsleep(duration)


class WallTime(Time):
    """Time object that always uses wall time."""

    def __init__(self, secs=0, nsecs=0):
        super(WallTime, self).__init__(secs, nsecs)

    @staticmethod
    def now():
        float_secs = time.time()
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return WallTime(secs, nsecs)


class SteadyTime(Time):
    """Time object that always uses monotonic time."""

    def __init__(self, secs=0, nsecs=0):
        super(SteadyTime, self).__init__(secs, nsecs)

    @staticmethod
    def now():
        float_secs = monotonic()
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return SteadyTime(secs, nsecs)


class CustomRate(Rate):
    """Rate object that sleeps in a custom time."""

    def __init__(self, hz, now_fn, sleep_fn):
        """Constructor.

        :param float hz: hz rate to determine sleeping
        :param Callable[[],rospy.Time] now_fn: Function that returns current time.
        :param Callable[[rospy.Duration],None] sleep_fn: Function that sleeps for the given duration.
        """
        super(CustomRate, self).__init__(hz, False)
        self._hz = hz
        self._now_fn = now_fn
        self._sleep_fn = sleep_fn
        self.last_time = self.now()

    def now(self):
        """Return the current time.

        :return: Current time.
        :rtype: rospy.Time
        """
        return self._now_fn()

    def remaining(self):
        curr_time = self.now()
        return self._remaining(curr_time)

    def sleep(self):
        curr_time = self.now()
        self._sleep_fn(self._remaining(curr_time))
        after_time = self.now()
        if after_time < curr_time:
            self.last_time = after_time
            return
        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time


class WallRate(CustomRate):
    """Rate object that sleeps in wall time.

    :note: This is almost never a good idea as the timer is affected by system time jumps. Consider using
           :class:`SteadyRate` instead.
    """

    def __init__(self, hz):
        """Constructor.

        :param float hz: hz rate to determine sleeping
        """
        super(WallRate, self).__init__(hz, WallTime.now, wallsleep)


class SteadyRate(CustomRate):
    """Rate object that sleeps in monotonic time, i.e. is not affected by system time jumps."""

    def __init__(self, hz):
        """Constructor.

        :param float hz: hz rate to determine sleeping
        """
        super(SteadyRate, self).__init__(hz, SteadyTime.now, wallsleep)


class Timer(rospy.Timer):
    """Timer that can use custom implementations of rate.

    Make sure to keep a reference to this timer for the whole time you want it working.
    """

    def __init__(self, rate, callback, oneshot=False):
        """Constructor.

        :param CustomRate rate: The rate to use for sleeping.
        :param Callable[[rospy.timer.TimerEvent],None] callback: Callback to call when the rate is up.
        :param bool oneshot: Whether the timer should only fire once and stop.
        """
        self._rate = rate
        super(Timer, self).__init__(rate.sleep_dur, callback, oneshot, True)

    def __del__(self):
        self.shutdown()
        self.join()

    def run(self):
        current_expected = self._rate.now() + self._period
        last_expected, last_real, last_duration = None, None, None
        while not rospy.core.is_shutdown() and not self._shutdown:
            try:
                self._rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                if rospy.core.is_shutdown():
                    break
                raise
            if self._shutdown:
                break
            current_real = self._rate.now()
            start = SteadyTime.now()
            self._callback(rospy.timer.TimerEvent(
                last_expected, last_real, current_expected, current_real, last_duration))
            if self._oneshot:
                break
            last_duration = (SteadyTime.now() - start).to_sec()
            last_expected, last_real = current_expected, current_real
            current_expected += self._period
