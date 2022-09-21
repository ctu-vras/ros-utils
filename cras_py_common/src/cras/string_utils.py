# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when working with strings in Python and rospy."""
import genpy
import rospy

from .time_utils import frequency


STRING_TYPE = str
"""A type that represents all string types, compatible with Py2 and Py3"""
try:
    STRING_TYPE = basestring
except NameError:
    pass


def __time_val_to_str(time):
    """
    Convert a time or duration to a sensible string.
    :param rospy.Time|rospy.Duration time: The time/duration object.
    :return: The string representation.
    :rtype: str
    """
    return "%i.%09i" % (time.secs, time.nsecs)


__to_str_functions = {
    rospy.Time: __time_val_to_str,
    rospy.Duration: __time_val_to_str,
    rospy.Rate: lambda r: str(frequency(r)),
}


def register_to_str(data_type, fn):
    """Register new conversion for `to_str()`.

    :param Type data_type: The type to register.
    :param Callable[[Any],str] fn: The conversion function. It should return the string representation.
    """
    __to_str_functions[data_type] = fn


__numpy_array_type = None
try:
    import numpy as np
    __numpy_array_type = np.ndarray
except ImportError:
    pass


def to_str(obj):
    """Convert the given object to a human-friendly string representation.

    :param Any obj: The object to convert.
    :return: The object's string representation.
    :rtype: str
    """
    if isinstance(obj, STRING_TYPE):
        return obj

    obj_type = obj.__class__
    if obj_type in __to_str_functions:
        return __to_str_functions[obj_type](obj)

    if hasattr(obj, "__name__"):
        return obj.__name__

    if isinstance(obj, genpy.Message):
        # keep this algorithm in sync with cras_cpp_common/string_utils/ros.hpp
        s = str(obj)
        return s.rstrip("\n").replace("\n", ", ")

    if isinstance(obj, dict):
        items = list()
        for key in obj:
            key_str = to_str(key)
            if isinstance(key, STRING_TYPE):
                key_str = "'%s'" % (key_str,)
            val_str = to_str(obj[key])
            if isinstance(obj[key], STRING_TYPE):
                val_str = "'%s'" % (val_str,)
            items.append((key_str, val_str))
        items = sorted(items)
        braces = str(dict())
        return braces[0] + ", ".join([k + ": " + v for k, v in items]) + braces[1]
    elif __numpy_array_type is not None and isinstance(obj, __numpy_array_type):
        # numpy matrix has weird behavior that fails in the next branch, so we shortcut it here
        return str(obj)
    else:
        try:
            items = list()
            for i in obj:
                val_str = to_str(i)
                if isinstance(i, STRING_TYPE):
                    val_str = "'%s'" % (val_str,)
                items.append(val_str)
            if isinstance(obj, set):
                items = sorted(items)
            braces = str(obj_type())
            if obj_type is set:
                braces = "{}"
            return braces[0] + ", ".join(items) + braces[1]
        except TypeError:  # obj is not iterable
            return str(obj)
