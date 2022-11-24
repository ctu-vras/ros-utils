# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities for working with ROS types."""

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
