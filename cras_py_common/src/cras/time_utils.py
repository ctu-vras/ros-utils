# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when working with rospy Time, Duration and Rate objects."""

from rospy import Time, Duration, Rate


DURATION_MAX = Duration(2**31-1, 999999999)
"""Maximum value of rospy.Duration that can be read without problems by other ROS client libraries."""
DURATION_MIN = Duration(-2**31, 0)
"""Minimum value of rospy.Duration that can be read without problems by other ROS client libraries."""

TIME_MAX = Time(2**32-1, 999999999)
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
