# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy


def get_param(param_name, default_value=None, unit="", print_default_as_warn=False):
    """
    Load a ROS parameter from param server, logging what we found to console.
    :param str param_name: Name of the parameter.
    :param Any default_value: Default value to be used it the parameter is not set. In None, the parameter is required.
    :param str unit: Optional unit of the param values. Will be used in console messages.
    :param bool print_default_as_warn: If true and default value was used, print the message as warning, otherwise print
                                       it at info level.
    :return: The parameter value.
    :rtype: Any
    :raises ValueError: If the parameter is required and not set on param server.
    """
    if rospy.has_param(param_name):
        value = rospy.get_param(param_name)

        rospy.loginfo("%s: Found parameter: %s, value: %s%s." % (
            rospy.get_name(),
            param_name,
            str(value),
            " " + unit if unit != "" else ""
        ))

        return value
    elif default_value is not None:
        log_fn = rospy.logwarn if print_default_as_warn else rospy.loginfo
        log_fn("%s: Cannot find value for parameter: %s, assigning default: %s%s." % (
            rospy.get_name(),
            param_name,
            str(default_value),
            " " + unit if unit != "" else ""
        ))

        return default_value
    else:
        raise ValueError("%s: Required parameter %s is not set." % (
            rospy.get_name(), param_name
        ))
