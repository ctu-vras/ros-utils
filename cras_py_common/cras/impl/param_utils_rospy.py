# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""rospy time parsers for ROS parameters."""

from functools import partial

import rospy

import cras.param_utils as param_utils
from cras.string_utils import to_str
from cras.time_utils import safe_rate


def __convert_timeval_from_dict(timeval_type, dict_value):
    if not isinstance(dict_value, dict):
        raise ValueError("Cannot convert %s to time value." % (to_str(type(dict_value)),))
    if "sec" not in dict_value and "nsec" not in dict_value:
        raise ValueError("Time value dict should consist of fields 'sec' and 'nsec'.")
    sec = dict_value.get("sec", 0)
    nsec = dict_value.get("nsec", 0)
    if not isinstance(sec, int):
        raise ValueError("'sec' field of a time value has to be an integer.")
    if not isinstance(nsec, int):
        raise ValueError("'nsec' field of a time value has to be an integer.")
    return timeval_type(sec, nsec)


param_utils.register_param_conversion(rospy.Duration, int, rospy.Duration)
param_utils.register_param_conversion(rospy.Duration, float, rospy.Duration)
param_utils.register_param_conversion(rospy.Duration, dict, partial(__convert_timeval_from_dict, rospy.Duration))

param_utils.register_param_conversion(rospy.Time, int, rospy.Time)
param_utils.register_param_conversion(rospy.Time, float, rospy.Time)
param_utils.register_param_conversion(rospy.Time, dict, partial(__convert_timeval_from_dict, rospy.Time))

param_utils.register_param_conversion(rospy.Rate, int, safe_rate)
param_utils.register_param_conversion(rospy.Rate, float, safe_rate)

param_utils.register_default_unit(rospy.Duration, "s")
param_utils.register_default_unit(rospy.Rate, "Hz")
