# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Geometry utilities.

Conversions quaternion<->roll/pitch/yaw are consistent with those in cras_cpp_common tf2_utils.
"""


from geometry_msgs.msg import Quaternion
# tf.transformations imports are moved inside the functions so that tf2_ros doesn't need to be imported when doing a
# simple "import cras" and not using this module. tf2_ros import requires the ROS_PACKAGE_PATH to be filled.


def quat_get_rpy(quaternion, *args):
    """Convert the given quaternion to roll, pitch and yaw.

    :param Quaternion|List[float]|Tuple[float] quaternion: The quaternion to convert. If a 4-tuple is passed, make sure
                                                           `w` is the last component, not the first!
    :param Tuple[Any] args: You can also call this function with 4 arguments instead of passing a tuple.
    :return: Tuple (roll, pitch, yaw) in radians.
    :rtype: Tuple[float]
    :raises ValueError: If invalid number of arguments is passed or they have invalid number of elements.
    """
    from tf.transformations import euler_from_quaternion
    q = quaternion
    if len(args) > 0:
        if len(args) != 3:
            raise ValueError("quat_get_rpy() requires one 4-tuple argument or exactly 4 float arguments.")
        q = (q,) + tuple(args)
    elif isinstance(q, Quaternion):
        q = (q.x, q.y, q.z, q.w)
    elif len(q) != 4:
        raise ValueError("quat_get_rpy() requires one 4-tuple argument or exactly 4 float arguments.")
    return euler_from_quaternion(q)


def quat_get_roll(quaternion, *args):
    """Get the given quaternion's roll in radians.

    :param Quaternion|List[float]|Tuple[float] quaternion: The quaternion to convert. If a 4-tuple is passed, make sure
                                                           `w` is the last component, not the first!
    :param Tuple[Any] args: You can also call this function with 4 arguments instead of passing a tuple.
    :return: Roll in radians.
    :rtype: float
    :raises ValueError: If invalid number of arguments is passed or they have invalid number of elements.
    """
    return quat_get_rpy(quaternion, *args)[0]


def quat_get_pitch(quaternion, *args):
    """Get the given quaternion's pitch in radians.

    :param Quaternion|List[float]|Tuple[float] quaternion: The quaternion to convert. If a 4-tuple is passed, make sure
                                                           `w` is the last component, not the first!
    :param Tuple[Any] args: You can also call this function with 4 arguments instead of passing a tuple.
    :return: Pitch in radians.
    :rtype: float
    :raises ValueError: If invalid number of arguments is passed or they have invalid number of elements.
    """
    return quat_get_rpy(quaternion, *args)[1]


def quat_get_yaw(quaternion, *args):
    """Get the given quaternion's yaw in radians.

    :param Quaternion|List[float]|Tuple[float] quaternion: The quaternion to convert. If a 4-tuple is passed, make sure
                                                           `w` is the last component, not the first!
    :param Tuple[Any] args: You can also call this function with 4 arguments instead of passing a tuple.
    :return: Yaw in radians.
    :rtype: float
    :raises ValueError: If invalid number of arguments is passed or they have invalid number of elements.
    """
    return quat_get_rpy(quaternion, *args)[2]


def quat_tuple_from_rpy(roll, pitch=None, yaw=None):
    """Convert the given roll, pitch and yaw to quaternion represented as 4-tuple of floats (`w` last).

    :param float|Tuple[float]|List[float] roll: Roll in radians (or a 3-tuple (roll, pitch, yaw)).
    :param float|None pitch: Pitch in radians (do not use if you pass 3-tuple in the first argument).
    :param float|None yaw: Yaw in radians (do not use if you pass 3-tuple in the first argument).
    :return: The corresponding quaternion.
    :rtype: Tuple[float]
    :raises ValueError: If invalid number of arguments is passed or they have invalid number of elements.
    """
    from tf.transformations import quaternion_from_euler
    if isinstance(roll, list) or isinstance(roll, tuple):
        if pitch is not None or yaw is not None or len(roll) != 3:
            raise ValueError("quat_tuple_from_rpy() can be called either with a 3-tuple or with 3 float arguments.")
        pitch = roll[1]
        yaw = roll[2]
        roll = roll[0]
    elif pitch is None or yaw is None:
        raise ValueError("quat_tuple_from_rpy() can be called either with a 3-tuple or with 3 float arguments.")
    return quaternion_from_euler(float(roll), float(pitch), float(yaw))


def quat_msg_from_rpy(roll, pitch=None, yaw=None):
    """Convert the given roll, pitch and yaw to :class:`geometry_msgs.msg.Quaternion`.

    :param float|Tuple[float]|List[float] roll: Roll in radians (or a 3-tuple (roll, pitch, yaw)).
    :param float|None pitch: Pitch in radians (do not use if you pass 3-tuple in the first argument).
    :param float|None yaw: Yaw in radians (do not use if you pass 3-tuple in the first argument).
    :return: The corresponding quaternion.
    :rtype: Quaternion
    """
    q = quat_tuple_from_rpy(roll, pitch, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
