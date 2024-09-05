# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with rospy logging."""

import logging

import rospy

from rosgraph_msgs.msg import Log


log_levels = {
    Log.DEBUG,
    Log.INFO,
    Log.WARN,
    Log.ERROR,
    Log.FATAL,
}
"""All available log levels."""

log_level_names = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}
"""Stringified names of the ROS logging levels."""

log_level_ros_to_py = {
    Log.DEBUG: logging.DEBUG,
    Log.INFO: logging.INFO,
    Log.WARN: logging.WARN,
    Log.ERROR: logging.ERROR,
    Log.FATAL: logging.FATAL,
}
"""Convert ROS log level int to python logging module level int."""

log_level_py_to_ros = dict(map(reversed, log_level_ros_to_py.items()))
"""Convert python logging module level int to ROS log level int."""

log_level_ros_to_py_name = dict([(level, logging.getLevelName(log_level_ros_to_py[level])) for level in log_levels])
"""Convert ROS log level int to python logging module level string."""

log_level_py_name_to_ros = dict(map(reversed, log_level_ros_to_py_name.items()))
"""Convert python logging module level string to ROS log level int."""

log_functions = {
    Log.DEBUG: rospy.logdebug,
    Log.INFO: rospy.loginfo,
    Log.WARN: rospy.logwarn,
    Log.ERROR: rospy.logerr,
    Log.FATAL: rospy.logfatal,
}
"""The rospy logging functions accessible by their ROS logging level index."""

log_throttle_functions = {
    Log.DEBUG: rospy.logdebug_throttle,
    Log.INFO: rospy.loginfo_throttle,
    Log.WARN: rospy.logwarn_throttle,
    Log.ERROR: rospy.logerr_throttle,
    Log.FATAL: rospy.logfatal_throttle,
}
"""The rospy throttled logging functions accessible by their ROS logging level index."""

log_throttle_identical_functions = {
    Log.DEBUG: rospy.logdebug_throttle_identical,
    Log.INFO: rospy.loginfo_throttle_identical,
    Log.WARN: rospy.logwarn_throttle_identical,
    Log.ERROR: rospy.logerr_throttle_identical,
    Log.FATAL: rospy.logfatal_throttle_identical,
}
"""The rospy identical-throttled logging functions accessible by their ROS logging level index."""

log_once_functions = {
    Log.DEBUG: rospy.logdebug_once,
    Log.INFO: rospy.loginfo_once,
    Log.WARN: rospy.logwarn_once,
    Log.ERROR: rospy.logerr_once,
    Log.FATAL: rospy.logfatal_once,
}
"""The rospy once-only logging functions accessible by their ROS logging level index."""


def log(level, *args, **kwargs):
    """Log a ROS message with a logger that has the given severity level.

    :param int level: Severity of the message (one of :class:`rosgraph_msgs.msg.Log` constants).
    :param str message: The logged message template (same rules as in the logging module apply to args/kwargs).
    :param args: Args are passed directly to :func:`rospy.loginfo()` or the other functions.
    :param kwargs: Keyword args are passed directly to :func:`rospy.loginfo()` or the other functions.
    :raises KeyError: if level is not one of the :class:`rosgraph_msgs.msg.Log` constants.
    """
    # Unfortunately, we can't call the rospy.loginfo() etc. functions directly, as the calling
    # location would be this line. The internal code in base_logger unwinds two frames from stack trace to get the
    # logging location, so if we call _base_logger() directly, we get exactly what we want.
    # noinspection PyProtectedMember
    rospy.core._base_logger(message, args, kwargs, level=log_level_ros_to_py_name[level].lower())


def log_throttle(level, period, message, *args, **kwargs):
    """Log a ROS message with a identical-throttled logger that has the given severity level.

    :param int level: Severity of the message (one of :class:`rosgraph_msgs.msg.Log` constants).
    :param float period: Logging period (in seconds).
    :param str message: The logged message template (same rules as in the logging module apply to args/kwargs).
    :param args: Args are passed directly to :func:`rospy.loginfo_throttle()` or the other functions.
    :param kwargs: Keyword args are passed directly to :func:`rospy.loginfo_throttle()` or the other functions.
    :raises KeyError: if level is not one of the :class:`rosgraph_msgs.msg.Log` constants.
    """
    # Unfortunately, we can't call the rospy.loginfo_throttle() etc. functions directly, as the calling
    # location would be this line. The internal code in base_logger unwinds two frames from stack trace to get the
    # logging location, so if we call _base_logger() directly, we get exactly what we want.
    # noinspection PyProtectedMember
    rospy.core._base_logger(message, args, kwargs, throttle=period, level=log_level_ros_to_py_name[level].lower())


def log_throttle_identical(level, period, message, *args, **kwargs):
    """Log a ROS message with a identical-throttled logger that has the given severity level.

    :param int level: Severity of the message (one of :class:`rosgraph_msgs.msg.Log` constants).
    :param float period: Logging period (in seconds).
    :param str message: The logged message template (same rules as in the logging module apply to args/kwargs).
    :param args: Args are passed directly to :func:`rospy.loginfo_throttle_identical()` or the other functions.
    :param kwargs: Keyword args are passed directly to :func:`rospy.loginfo_throttle_identical()` or other functions.
    :raises KeyError: if level is not one of the :class:`rosgraph_msgs.msg.Log` constants.
    """
    # Unfortunately, we can't call the rospy.loginfo_throttle_identical() etc. functions directly, as the calling
    # location would be this line. The internal code in base_logger unwinds two frames from stack trace to get the
    # logging location, so if we call _base_logger() directly, we get exactly what we want.
    # noinspection PyProtectedMember
    rospy.core._base_logger(message, args, kwargs, throttle=period, throttle_identical=True,
                            level=log_level_ros_to_py_name[level].lower())


def log_once(level, message, *args, **kwargs):
    """Log a ROS message with a once-only logger that has the given severity level.

    :param int level: Severity of the message (one of :class:`rosgraph_msgs.msg.Log` constants).
    :param str message: The logged message template (same rules as in the logging module apply to args/kwargs).
    :param args: Args are passed directly to :func:`rospy.loginfo_once()` or the other functions.
    :param kwargs: Keyword args are passed directly to :func:`rospy.loginfo_once()` or the other functions.
    :raises KeyError: if level is not one of the :class:`rosgraph_msgs.msg.Log` constants.
    """
    # Unfortunately, we can't call the rospy.loginfo_once() etc. functions directly, as the calling location would be
    # this line. The internal code in base_logger unwinds two frames from stack trace to get the logging location, so
    # if we call _base_logger() directly, we get exactly what we want.
    # noinspection PyProtectedMember
    rospy.core._base_logger(message, args, kwargs, once=True, level=log_level_ros_to_py_name[level].lower())
