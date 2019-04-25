import rospy


def get_param(param_name, default_value=None, unit=""):
    """
    Load a ROS parameter from param server, logging what we found to console.
    :param str param_name: Name of the parameter.
    :param Any default_value: Default value to be used it the parameter is not set. In None, the parameter is required.
    :param str unit: Optional unit of the param values. Will be used in console messages.
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
        rospy.logwarn("%s: Cannot find value for parameter: %s, assigning default: %s%s." % (
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
