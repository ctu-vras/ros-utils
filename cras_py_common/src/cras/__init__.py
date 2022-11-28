# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for more convenient usage of rospy."""

from .geometry_utils import quat_get_rpy, quat_get_roll, quat_get_pitch, quat_get_yaw, \
    quat_tuple_from_rpy, quat_msg_from_rpy

from .log_utils import log, log_throttle, log_throttle_identical, log_once, \
    log_functions, log_throttle_functions, log_throttle_identical_functions, log_once_functions, log_levels, \
    log_level_names, log_level_ros_to_py_name, log_level_py_to_ros, log_level_ros_to_py, log_level_py_name_to_ros

from .message_utils import get_msg_field, get_msg_type

from .node_utils import Node

from .param_utils import get_param, get_param_verbose, GetParamException, GetParamResult, GetParamResultInfo, \
    register_default_unit, register_enum_conversion, register_param_conversion

import cras.impl.param_utils_geometry_msgs
import cras.impl.param_utils_numpy
import cras.impl.param_utils_rospy

from .time_utils import DURATION_MIN, DURATION_MAX, TIME_MIN, TIME_MAX, slowest_rate, slowest_negative_rate,\
    frequency, safe_rate, rate_equals, monotonic, wallsleep, WallTime, SteadyTime, WallRate, SteadyRate, Timer

try:
    from .time_utils import steadysleep, SteadyRate
except ImportError:
    pass

from .string_utils import to_str, register_to_str

from .topic_utils import GenericMessageSubscriber
