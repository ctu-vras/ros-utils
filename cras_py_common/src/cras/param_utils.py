# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with ROS parameters.

Package cras.impl contains some predefined advanced parameter parsers. Other advanced parsers can be registered using
:func:`register_param_conversion()`.

The predefined conversions can parse:

- :class:`rospy.Duration` from a float or from dict {'sec', 'nsec'}
- :class:`rospy.Time` from a float or from dict {'sec', 'nsec'}
- :class:`geometry_msgs.msg.Vector3` from a 3-tuple or from dict {'x', 'y', 'z'}
- :class:`geometry_msgs.msg.Point` from a 3-tuple or from dict {'x', 'y', 'z'}
- :class:`geometry_msgs.msg.Point32` from a 3-tuple or from dict {'x', 'y', 'z'}
- :class:`geometry_msgs.msg.Pose2D` from a 3-tuple or from dict {'x', 'y', 'theta'}
- :class:`geometry_msgs.msg.Quaternion` from a 4-tuple (`w` last!) or from dict {'x', 'y', 'z', 'w'}, or from
    {'r', 'p', 'y'} or from {'roll', 'pitch', 'yaw'}. Any missing dict key will be replaced by 0. Beware that if you
    specify only 'y' key, it is ambiguous. The quaternion is not explicitly normalized (when specified with 4 elements).
- :class:`geometry_msgs.msg.Pose` from a 6-tuple (xyzrpy), 7-tuple (xyzxyzw), 16-tuple (row-major homogeneous transform
    matrix) or from dict {'position', 'orientation'}, where 'position' is parsed as `Point` and
    'orientation' as `Quaternion` (by any of their supported specifications).
"""

from functools import partial

import rospy
from rosgraph_msgs.msg import Log

from .log_utils import log
from .string_utils import to_str


class GetParamResultInfo:
    """Detailed information about the executed :func:`get_param_verbose()` call.

    :ivar bool default_used: Whether the default value has been used.
    :ivar bool convert_failed: Whether a value conversion failed.
    :ivar bool required_missing: Whether a required parameter was found missing or could not be read.
    :ivar str message: The log message (returned even if option `print_messages` is False).
    :ivar uint message_level: Severity of the log message (one of :class:`rosgraph_msgs.msg.Log` constants).
    """
    def __init__(self, default_used=False, convert_failed=False, required_missing=False,
                 message="", message_level=Log.INFO):
        self.default_used = default_used
        self.convert_failed = convert_failed
        self.required_missing = required_missing
        self.message = message
        self.message_level = message_level


class GetParamResult:
    """Wrapper for the result of a :func:`get_param_verbose()` call. It contains the parameter value and additional
    information.

    :ivar Any value: The returned value.
    :ivar GetParamResultInfo info: Additional details about the :func:`get_param_verbose()` call progress.
    """
    def __init__(self, value=None, info=None):
        self.value = value
        self.info = info if info is not None else GetParamResultInfo()


class GetParamException(RuntimeError):
    """Exception thrown when conversion of a parameter fails during :func:`get_param()` if option
    `throw_if_convert_fails` is True or when a missing parameter is required.

    :ivar GetParamResultInfo info: Details about :func:`get_param()` execution until the failure.
    """

    def __init__(self, info):
        super(GetParamException, self).__init__(info.message)
        self.info = info


def __convert_with_bounds_check(target_type, min_value, max_value, value):
    if min_value <= value <= max_value:
        return target_type(value)
    raise ValueError("Cannot convert %s value %s to %s" % (to_str(type(value)), str(value), to_str(target_type)))


__param_conversions = {  # Dict[Type, Dict[Type, Callable[[Type], Type]]]
    bool: {
        int: partial(__convert_with_bounds_check, bool, 0, 1),
    },
    int: {
        bool: int,
    },
    float: {
        int: float,
    },
}

__default_units = dict()  # Dict[Type, str]


def register_param_conversion(result_type, param_type, convert_fn):
    """Register a function converting ROS parameters of type `param_type` to `result_type`.

    :param Type result_type: Type of the result value.
    :param Type param_type: Type of the parameter value read from ROS parameter server.
    :param Callable[[param_type],result_type] convert_fn: Function converting the ROS parameter to the result type. It
                                                          should raise :class:`ValueError` if the value is not suitable.
    """
    if result_type not in __param_conversions:
        __param_conversions[result_type] = dict()
    __param_conversions[result_type][param_type] = convert_fn


def register_default_unit(result_type, unit):
    """Register a default unit used when printing log messages.

    :param Type result_type: Type of the parameter.
    :param str unit: Unit to use for the given parameter type if none other is specified.
    """
    __default_units[result_type] = unit


def get_param_verbose(param_name, default_value=None, unit="", print_default_as_warn=False, print_messages=True,
                      throw_if_convert_fails=False, result_type=None):
    """Load a ROS parameter from param server, possibly logging what we found to console.

    :param str param_name: Name of the parameter.
    :param Any default_value: Default value to be used it the parameter is not set. In None, the parameter is required.
    :param str unit: Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
                     more informative. If empty, a default is looked up which can be registered using
                     :func:`register_default_unit()`.
    :param bool print_default_as_warn: Whether defaulted parameters are reported as warning or info level messages.
    :param bool print_messages: Whether to print error messages to log.
    :param bool throw_if_convert_fails: Throw :class:`GetParamException` if any conversion fails. If False, the default
                                        value is used instead of a value that failed to convert. In such case, the log
                                        message is of error level.
    :param Any result_type: Desired type of the result. If None, the same type as `default_value` is used.
                            If `default_value` is also `None`, no type conversion is done.
    :return: The parameter value wrapped in :class:`GetParamResult` struct with additional details.
    :rtype: GetParamResult
    :raises GetParamException: If the parameter is required and not set on param server, or if type conversion fails.
    """
    target_type = result_type
    if target_type is None and default_value is not None:
        target_type = default_value.__class__

    unit_resolved = unit
    if unit_resolved == "" and target_type is not None:
        unit_resolved = __default_units.get(target_type, "")
    unit_str = " " + unit_resolved if unit_resolved != "" else ""

    result = GetParamResult()
    if rospy.has_param(param_name):
        value = rospy.get_param(param_name)
        value_type = value.__class__
        if target_type is not None and value_type != target_type:
            expected_types = None
            try:
                if target_type in __param_conversions:
                    convert_fns = __param_conversions[target_type]
                    if value_type in convert_fns:
                        value = convert_fns[value_type](value)
                    else:
                        expected_types = tuple(convert_fns.keys())
                        raise ValueError()
                elif target_type in (list, tuple, set):
                    if value_type is list:
                        value = value_type(value)
                    else:
                        expected_types = (list, tuple, set)
                        raise ValueError()
                elif target_type is dict:
                    if value_type is not dict:
                        expected_types = (dict,)
                        raise ValueError()
                else:
                    value = target_type(value)
            except Exception as e:
                result.info.convert_failed = True
                result.info.message_level = Log.ERROR
                # If we did not find a convert function, e.message is empty and expected_types are not.
                if expected_types is not None:
                    types = ", ".join(sorted([to_str(type) for type in expected_types]))
                    result.info.message = \
                        "%s: Parameter %s found, but it has wrong XmlRpc type. Expected one of %s, got type %s with " \
                        "value %s." % (rospy.get_name(), param_name, types, value_type.__name__, to_str(value))
                else:
                    result.info.message = \
                        "%s: Parameter %s found with correct XmlRpc type %s and value %s, but its conversion to " \
                        "type %s has failed due to the following errors: %s." % (
                            rospy.get_name(), param_name, to_str(value_type), to_str(value), to_str(target_type),
                            str(e))
                if throw_if_convert_fails or default_value is None:
                    if default_value is None:
                        result.info.required_missing = True
                    if print_messages:
                        log(result.info.message_level, result.info.message)
                    raise GetParamException(result.info)
                else:
                    value = default_value
                    result.info.default_used = True

        result.value = value

        if len(result.info.message) == 0:
            result.info.message = ("%s: Found parameter: %s, value: %s%s." % (
                rospy.get_name(),
                param_name,
                to_str(result.value),
                unit_str
            ))
            result.info.message_level = Log.INFO
    elif default_value is not None:
        result.value = default_value
        result.info.default_used = True
        result.info.message = ("%s: Cannot find value for parameter: %s." % (rospy.get_name(), param_name))
        result.info.message_level = Log.WARN if print_default_as_warn else Log.INFO
    else:
        result.info.required_missing = True
        result.info.message = "%s: Required parameter %s is not set." % (rospy.get_name(), param_name)
        result.info.message_level = Log.ERROR
        if print_messages:
            log(result.info.message_level, result.info.message)
        raise GetParamException(result.info)

    if result.info.default_used:
        result.info.message += " Assigning default: %s%s." % (to_str(default_value), unit_str)

    if print_messages:
        log(result.info.message_level, result.info.message)

    return result


def get_param(param_name, default_value=None, unit="", print_default_as_warn=False, print_messages=True,
              throw_if_convert_fails=False, result_type=None):
    """Load a ROS parameter from param server, logging what we found to console.

    :param str param_name: Name of the parameter.
    :param Any default_value: Default value to be used it the parameter is not set. In None, the parameter is required.
    :param str unit: Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
                     more informative. If empty, a default is looked up which can be registered using
                     :func:`register_default_unit()`.
    :param bool print_default_as_warn: Whether defaulted parameters are reported as warning or info level messages.
    :param bool print_messages: Whether to print error messages to log.
    :param bool throw_if_convert_fails: Throw :class:`GetParamException` if any conversion fails. If False, the default
                                        value is used instead of a value that failed to convert. In such case, the log
                                        message is of error level.
    :param Any result_type: Desired type of the result. If `None`, the same type as `default_value` is used.
                            If `default_value` is also `None`, no type conversion is done.
    :return: The parameter value.
    :rtype: Any
    :raises GetParamException: If the parameter is required and not set on param server, or if type conversion fails.
    """
    return get_param_verbose(param_name, default_value, unit, print_default_as_warn, print_messages,
                             throw_if_convert_fails, result_type).value
