# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when writing nodes."""

import rospy

from .param_utils import get_param
from .time_utils import SteadyRate, Timer, DURATION_MAX, TIME_MIN


class Node(object):
    """Base class for all nodes.

    This class reads the following ROS parameters:
    - `/jump_back_tolerance` (float, default 3.0 in wall time and 0.0 in sim time):
        Threshold for ROS time jump back detection.
    - `~jump_back_tolerance` (float, default from `/jump_back_tolerance`): Threshold for ROS time jump back detection.
    - `~reset_on_time_jump_back` (bool, default True): Whether to call :func:`reset()` when ROS time jumps back.
    - `/jump_forward_tolerance` (float, default 10.0 in sim time and max duration in wall time):
        Threshold for ROS time jump forward detection.
    - `~jump_forward_tolerance` (float, default from `/jump_forward_tolerance`):
        Threshold for ROS time jump forward detection.
    - `~reset_on_time_jump_forward` (bool, default True in sim time and False in wall time):
        Whether to call :func:`reset()` when ROS time jumps forward.

    :ivar rospy.Duration _jump_back_tolerance: Threshold for ROS time jump back detection.
    :ivar rospy.Duration _jump_forward_tolerance: Threshold for ROS time jump forward detection.
    :ivar rospy.Time _last_time_stamp: Last recorded time stamp. Used for time jump back detection.
    :ivar bool _reset_on_time_jump_back: Whether to call :func:`reset()` when ROS time jumps back.
    :ivar bool _reset_on_time_jump_forward: Whether to call :func:`reset()` when ROS time jumps forward.
    """

    def __init__(self):
        super(Node, self).__init__()
        is_wall_time = rospy.rostime.is_wallclock() if rospy.rostime.is_rostime_initialized() else True

        back_tolerance = rospy.Duration(3) if is_wall_time else rospy.Duration(0)
        back_tolerance = get_param("/jump_back_tolerance", back_tolerance, "s", print_messages=False)
        self._jump_back_tolerance = get_param("~jump_back_tolerance", back_tolerance, "s", print_messages=False)

        forward_tolerance = DURATION_MAX if is_wall_time else rospy.Duration(10)
        forward_tolerance = get_param("/jump_forward_tolerance", forward_tolerance, "s", print_messages=False)
        self._jump_forward_tolerance = get_param("~jump_forward_tolerance", forward_tolerance, "s",
                                                 print_messages=False)

        self._reset_on_time_jump_back = get_param("~reset_on_time_jump_back", True, print_messages=False)
        self._reset_on_time_jump_forward = get_param("~reset_on_time_jump_forward", not is_wall_time,
                                                     print_messages=False)

        self._last_time_stamp = None
        self._auto_check_timer = None

    def reset(self):
        """Reset state of the class to the state where it was just after initialization.

        :note: Do not forget to call super().reset() in subclasses!
        """
        self._last_time_stamp = None
        rospy.logdebug("Reset")

    def check_time_jump(self, now=None):
        """Check if ROS time has not jumped back. If it did, call :func:`reset()`.

        :note: This function should be called periodically. Call `func:`start_auto_check_time_jump` to spin a thread
               that will do these periodic calls automatically. Or you can call this function from your callbacks.
        :param rospy.Time|None now: The timestamp that should be interpreted as current time (if None,
                                    `rospy.Time.now()` is used).
        """
        if not self._reset_on_time_jump_back and not self._reset_on_time_jump_forward or \
                not rospy.rostime.is_rostime_initialized():
            return

        now = now if now is not None else rospy.Time.now()
        if now < TIME_MIN:
            return

        if self._last_time_stamp is not None:
            # Do not invert the following comparisons so that they would contain subtraction - in such a case, the
            # resulting Time instance could go into negative numbers and that is not allowed.
            if self._reset_on_time_jump_back and self._last_time_stamp > now + self._jump_back_tolerance:
                rospy.loginfo("ROS time jumped back, resetting.")
                self.reset()
            elif self._reset_on_time_jump_forward and self._last_time_stamp + self._jump_forward_tolerance < now:
                rospy.loginfo("ROS time jumped forward, resetting.")
                self.reset()

        self._last_time_stamp = now

    def _auto_check_timer_cb(self, _):
        self.check_time_jump()

    def start_auto_check_time_jump(self, hz=1.0):
        """Periodically check for time jumps (the checking period is in wall/steady time).

        :param float hz: The rate at which the checking should be done.
        """
        self._auto_check_timer = Timer(SteadyRate(hz), self._auto_check_timer_cb)

    def stop_auto_check_time_jump(self):
        """Stop periodic checking for time jumps."""
        if self._auto_check_timer is not None:
            self._auto_check_timer.shutdown()
            self._auto_check_timer = None
