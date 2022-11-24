# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when working with ROS messages."""


import importlib


__pkg_modules = dict()


def get_msg_type(msg_type_str):
    """Load and return a Python type corresponding to the given ROS message type.

    :param str msg_type_str: Type of the message in textual form (e.g. `sensor_msgs/Imu`).
    :return: The Python type object.
    :rtype: Type
    """
    pkg, msg = msg_type_str.split('/')
    pkg = pkg + '.msg'
    if pkg not in __pkg_modules:
        __pkg_modules[pkg] = importlib.import_module(pkg)
    pkg_module = __pkg_modules[pkg]
    return getattr(pkg_module, msg)


def get_msg_field(msg, field, sep="/"):
    """Parse a data field from a ROS message using a textual field address description.

    An example could be calling `get_msg_field(odom, 'pose/pose/position/x')` on an Odometry message.

    :param genpy.Message msg: Message.
    :param str field: The field address, levels separated with `sep` or using array access [].
    :param str sep: Separator used in the field address. Usually `/` or `.`.
    :return: Value found in the given field in the message.
    :rtype: Any
    """
    data = msg
    if field.startswith(sep):
        field = field[1:]
    for f in field.split(sep):
        parts = f.split("[")
        data = getattr(data, parts[0])
        for i in range(1, len(parts)):
            idx = parts[i]
            idx = idx[:-1]  # remove ] from the end
            idx = int(idx)
            data = data[idx]
    return data
