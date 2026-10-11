# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when working with ROS messages."""


import importlib

import genpy
from dynamic_reconfigure.msg import Config, BoolParameter, DoubleParameter, IntParameter, StrParameter

from .string_utils import BufferStringIO, STRING_TYPE


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


def dict_to_dynamic_config_msg(d):
    """Convert configuration dict to :class:`dynamic_reconfigure.msg.Config`.

    :param d: Configuration dict (or already the message, in which case it is just returned).
    :type d: dict or dynamic_reconfigure.msg.Config or None
    :return: The config message.
    :rtype: dynamic_reconfigure.msg.Config
    """
    if d is None:
        return Config()
    if isinstance(d, Config):
        return d
    c = Config()
    for key, value in d.items():
        if isinstance(value, bool):
            c.bools.append(BoolParameter(key, value))
        elif isinstance(value, float):
            c.doubles.append(DoubleParameter(key, value))
        elif isinstance(value, int):
            c.ints.append(IntParameter(key, value))
        elif isinstance(value, STRING_TYPE):
            c.strs.append(StrParameter(key, value))
    return c


def msg_to_raw(msg):  # type: (genpy.Message) -> (str, bytes, str, type)
    """Converts a message to raw representation.

    :param msg: The message to convert.
    :return: ROS datatype (as string), the raw bytes, md5sum of the datatype, ROS datatype (Python type)
    """
    datatype = msg.__class__._type
    md5sum = msg.__class__._md5sum
    pytype = msg.__class__
    msg_to_raw_buffer = BufferStringIO()
    msg.serialize(msg_to_raw_buffer)
    data = msg_to_raw_buffer.getvalue()
    return datatype, data, md5sum, pytype


def raw_to_msg(datatype, data, md5sum, pytype):
    """Convert a raw message representation to a ROS message

    :param str datatype: The ROS datatype as string.
    :param bytes data: The raw data.
    :param str md5sum: The MD5 sum of the datatype.
    :param type pytype: The ROS datatype (Python type).
    :return: The message instance.
    :rtype: genpy.Message
    :raises RuntimeError: If pytype does not match datatype and md5sum.
    """
    if pytype._type != datatype or pytype._md5sum != md5sum:
        raise RuntimeError(
            "The provided Python type %r does not match the ROS datatype %s/%s" % (pytype, datatype, md5sum))
    msg = pytype()
    msg.deserialize(data)
    return msg
