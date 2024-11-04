# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when working with strings in Python and rospy."""

import ctypes
import locale
import os
import re

from errno import errorcode, E2BIG, EILSEQ
from enum import Enum

import genpy
import rospy

from .ctypes_utils import get_libc, get_ro_c_buffer
from .python_utils import temp_locale
from .time_utils import frequency, WallTime, WallRate, SteadyTime, SteadyRate

STRING_TYPE = str
"""A type that represents all string types, compatible with Py2 and Py3"""
try:
    # noinspection PyCompatibility
    STRING_TYPE = basestring
except NameError:
    pass

# A StringIO/BytesIO depending on Python version that can be used for serializing ROS messages
try:
    from cStringIO import StringIO as BufferStringIO  # Python 2.x
except ImportError:
    from io import BytesIO as BufferStringIO  # Python 3.x


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
    genpy.rostime.Time: __time_val_to_str,
    rospy.Duration: __time_val_to_str,
    genpy.rostime.Duration: __time_val_to_str,
    rospy.Rate: lambda r: str(frequency(r)),
    WallTime: __time_val_to_str,
    SteadyTime: __time_val_to_str,
    WallRate: lambda r: str(frequency(r)),
    SteadyRate: lambda r: str(frequency(r)),
}


def register_to_str(data_type, fn):
    """Register new conversion for `to_str()`.

    :param Type data_type: The type to register.
    :param Callable[[Any],str] fn: The conversion function. It should return the string representation.
    """
    __to_str_functions[data_type] = fn


__numpy_array_type = None
try:
    # noinspection PyUnresolvedReferences
    import numpy

    __numpy_array_type = numpy.ndarray
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

    if isinstance(obj, Enum):
        # represent enum values just by the values without a namespace
        return obj.name

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


__iconv_open = None
__iconv = None
__ICONV_CONV_DESCS = dict()


def __errno_to_str(errno):
    if errno == 0:
        return "No error"
    return "%s: %s" % (errorcode[errno], os.strerror(errno))


def __iconv_get_conv_desc(to_encoding, from_encoding):
    if (to_encoding, from_encoding) not in __ICONV_CONV_DESCS:
        global __iconv_open
        if __iconv_open is None:
            __iconv_open = get_libc().iconv_open
            __iconv_open.restype = ctypes.c_void_p

        ctypes.set_errno(0)
        cd = __iconv_open(
            ctypes.c_char_p(to_encoding.encode("utf-8")),
            ctypes.c_char_p(from_encoding.encode("utf-8")))

        if cd == ctypes.c_void_p(-1).value:
            raise ValueError("Could not create conversion descriptor from encoding '%s' to '%s': Error %s" % (
                from_encoding, to_encoding, __errno_to_str(ctypes.get_errno())))

        __ICONV_CONV_DESCS[(to_encoding, from_encoding)] = cd

    return __ICONV_CONV_DESCS[(to_encoding, from_encoding)]


class __IconvError(Exception):
    def __init__(self, errno):
        self.errno = errno


def iconv_convert_bytes(to_encoding, from_encoding, in_bytes, translit=False, ignore=False,
                        initial_outbuf_size_scale=1.0, outbuf_enlarge_coef=2.0, localename=None):
    """Convert `in_bytes` from `from_encoding` to `to_encoding` using iconv.

    :param str to_encoding: The target encoding. It may contain the //TRANSLIT and //IGNORE suffixes.
    :param str from_encoding: The source encoding.
    :param bytes in_bytes: The bytes to convert (get them e.g. by calling `"my_string".encode('utf-8')`).
    :param bool translit: If True, the conversion will try to transliterate letters not present in target encoding.
    :param bool ignore: If True, letters that can't be converted and transliterated will be left out.
    :param float initial_outbuf_size_scale: The initial scale of the size of the output buffer. Setting this to the
                                            correct value may speed up the conversion in case the output is much larger
                                            than the input.
    :param outbuf_enlarge_coef: The step size to use for enlarging the output buffer if it shows that its initial size
                                is insufficient. Must be strictly larger than 1.0.
    :param localename: If set, specifies the locale used for the iconv call. It may influence the transliteration
                       results. If not set, a default english locale is used that usually works quite well.
    :type localename: str, optional
    :return: The converted bytes. Call e.g. `result.decode('utf-8')` to get a string from it.
    :rtype: bytes
    :raises ValueError: If some characters cannot be converted and `ignore` is False, or if the encodings are unknown.
    """
    if outbuf_enlarge_coef <= 1.0:
        raise ValueError("outbuf_enlarge_coef has to be strictly larger than 1.0")

    if translit and "//TRANSLIT" not in to_encoding:
        to_encoding += "//TRANSLIT"
    if ignore and "//IGNORE" not in to_encoding:
        to_encoding += "//IGNORE"
    ignore = "//IGNORE" in to_encoding or ("//" in to_encoding and ",IGNORE" in to_encoding)

    cd = __iconv_get_conv_desc(to_encoding, from_encoding)

    c_char_p = ctypes.POINTER(ctypes.c_char)

    global __iconv
    if __iconv is None:
        def errcheck(ret, func, args):
            if ret == ctypes.c_size_t(-1).value:
                raise __IconvError(ctypes.get_errno())
            return ret

        __iconv = get_libc().iconv
        __iconv.errcheck = errcheck
        __iconv.restype = ctypes.c_size_t
        __iconv.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(c_char_p), ctypes.POINTER(ctypes.c_size_t),
            ctypes.POINTER(c_char_p), ctypes.POINTER(ctypes.c_size_t)
        ]

    inbuf = get_ro_c_buffer(in_bytes)
    inbuf_ptr = ctypes.cast(inbuf, c_char_p)
    inbuf_unread_size = ctypes.c_size_t(len(in_bytes))

    outbuf_len = int(len(in_bytes) * initial_outbuf_size_scale)
    outbuf = ctypes.create_string_buffer(outbuf_len)
    outbuf_ptr = ctypes.cast(outbuf, c_char_p)
    outbuf_unused_size = ctypes.c_size_t(outbuf_len)

    # Read the input until there is something to read
    while inbuf_unread_size.value > 0:
        try:
            # iconv transliteration doesn't work with the default C locale, we need a UTF-8 one
            with temp_locale(locale.LC_CTYPE, localename if localename is not None else "en_US.UTF-8"):
                ctypes.set_errno(0)
                __iconv(cd,
                        ctypes.byref(inbuf_ptr), ctypes.byref(inbuf_unread_size),
                        ctypes.byref(outbuf_ptr), ctypes.byref(outbuf_unused_size))

                # Clean up the conversion descriptor and flush possible "shift sequences"
                ctypes.set_errno(0)
                __iconv(cd, None, None, ctypes.byref(outbuf_ptr), ctypes.byref(outbuf_unused_size))
        except __IconvError as e:
            # The output buffer is too small; increase its size and try the conversion again
            if e.errno == E2BIG:
                inbuf = get_ro_c_buffer(in_bytes)
                inbuf_ptr = ctypes.cast(inbuf, c_char_p)
                inbuf_unread_size = ctypes.c_size_t(len(in_bytes))

                outbuf_len = int(outbuf_len * outbuf_enlarge_coef)  # Enlarge the output buffer size
                outbuf = ctypes.create_string_buffer(outbuf_len)
                outbuf_ptr = ctypes.cast(outbuf, c_char_p)
                outbuf_unused_size = ctypes.c_size_t(outbuf_len)
            # Invalid byte sequence encountered or cannot transliterate to output
            else:
                # Reset the conversion descriptor as we'll be ignoring some bytes, so all context is lost
                ctypes.set_errno(0)
                __iconv(cd, None, None, None, None)

                if not ignore:
                    raise ValueError("Could not convert %r from encoding %s to %s. Error %s" % (
                        in_bytes, from_encoding, to_encoding, __errno_to_str(e.errno)))

                # Ignore invalid input byte sequences or sequences we can't transliterate
                if e.errno == EILSEQ and inbuf_unread_size.value > 1:
                    inbuf_unread_size.value -= 1
                    inbuf_ptr = ctypes.cast(ctypes.addressof(inbuf) + len(in_bytes) - inbuf_unread_size.value, c_char_p)
                # EINVAL means invalid byte sequence at the end of input, just throw it away
                # inbuf_unread_size == 0 means ignore is True, some chars were ignored, but otherwise, we have success
                else:
                    break

    return outbuf.value[:(outbuf_len - outbuf_unused_size.value)]


def transliterate_to_ascii(text):
    """Transliterate the given string from UTF-8 to ASCII (replace non-ASCII chars by closest ASCII chars).

    :param str text: The string to transliterate.
    :return: The transliterated string.
    :rtype: str
    """
    assert isinstance(text, STRING_TYPE)
    conv_bytes = iconv_convert_bytes(
        "ASCII", "UTF-8", text.encode('utf-8'), translit=True, ignore=True)
    return conv_bytes.decode('ascii', 'ignore')


NAME_LEGAL_CHARS_P = re.compile(r'^[~/]?[A-Za-z][a-zA-Z0-9/]*$')


def is_legal_name(name):
    """Check if name is a legal ROS name for graph resources.

    This function works correctly, which can't be said about the rosgraph.names version.

    :param str name: Name
    :rtype: bool
    """
    if name is None:
        return False
    # empty string is a legal name as it resolves to namespace
    if name == '' or name == '/' or name == '~':
        return True
    if '//' in name:
        return False
    m = NAME_LEGAL_CHARS_P.match(name)
    return m is not None and m.group(0) == name


BASE_NAME_LEGAL_CHARS_P = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def is_legal_base_name(name):
    """Validates that name is a legal base name for a graph resource. A base name has no namespace context, e.g.
    "node_name".

    This function works correctly, which can't be said about the rosgraph.names version.

    :param str name: Name
    :rtype: bool
    """
    if name is None:
        return False
    m = BASE_NAME_LEGAL_CHARS_P.match(name)
    return m is not None and m.group(0) == name


def to_valid_ros_name(text, base_name=True, fallback_name=None):
    """Make sure the given string can be used as ROS name.

    :param str text: The text to convert.
    :param bool base_name: If True, the text represents only one "level" of names. If False, it can be the absolute or
                           relative name with ~ and /.
    :param str fallback_name: If specified, this name will be used if the automated conversion fails. This name is not
                              checked to be valid.
    :return: The valid ROS graph resource name.
    :rtype: str
    :raises ValueError: If the given text cannot be converted to a valid ROS name, and no `fallback_name` is specified.
    """
    is_legal = is_legal_base_name if base_name else is_legal_name
    if is_legal(text):
        return text

    if len(text) == 0:
        if fallback_name is None:
            raise ValueError("Empty name is not allowed")
        return fallback_name

    name = transliterate_to_ascii(text)
    prefix = ''
    if base_name:
        name = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    else:
        if name[0] == '~':
            prefix = '~'
            name = name[1:]
        elif name[0] == '/':
            prefix = '/'
            name = name[1:]
        name = re.sub(r'[^a-zA-Z0-9_/]', '_', name)

    while "__" in name:
        name = name.replace("__", "_")

    name = re.sub(r'^[^a-zA-Z]*', '', name)
    if len(name) == 0:
        if fallback_name is None:
            raise ValueError("Name '%s' cannot be converted to valid ROS name" % (name,))
        return fallback_name

    name = prefix + name
    if not is_legal(name):
        if fallback_name is None:
            raise ValueError("Name '%s' cannot be converted to valid ROS name" % (name,))
        return fallback_name

    return name
