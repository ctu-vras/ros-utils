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


def get_srv_types(srv_type_str):
    """Load and return a Python types corresponding to the given ROS service type.

    :param str srv_type_str: Type of the service in textual form (e.g. `std_srvs/SetBool`).
    :return: A tuple of the main service Python type, the request type and the response type.
    :rtype: tuple
    """
    pkg, msg = srv_type_str.split('/')
    pkg = pkg + '.srv'
    if pkg not in __pkg_modules:
        __pkg_modules[pkg] = importlib.import_module(pkg)
    pkg_module = __pkg_modules[pkg]
    return getattr(pkg_module, msg), getattr(pkg_module, msg + "Request"), getattr(pkg_module, msg + "Response")


def get_cfg_module(cfg_type_str):
    """Load and return a Python module corresponding to the given ROS dynamic reconfigure type.

    :param str cfg_type_str: Type of the config in textual form (e.g. `compressed_image_transport/CompressedPublisher`).
    :return: The Python module.
    """
    if len(cfg_type_str) == 0:
        return None
    pkg, cfg = cfg_type_str.split('/')
    cfg += 'Config'
    cfg_module = pkg + '.cfg.' + cfg
    if cfg_module not in __pkg_modules:
        __pkg_modules[cfg_module] = importlib.import_module(cfg_module)
    return __pkg_modules[cfg_module]


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
