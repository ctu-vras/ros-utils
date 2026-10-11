# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Python bindings for sensor_msgs/image_encodings.h"""


import ctypes

from .ctypes_utils import load_library


sensor_msgs_py = load_library("sensor_msgs_py")


def __str_const(name):
    return ctypes.c_char_p.in_dll(sensor_msgs_py, name).value.decode("ascii")


RGB8 = __str_const("RGB8")
RGBA8 = __str_const("RGBA8")
RGB16 = __str_const("RGB16")
RGBA16 = __str_const("RGBA16")
BGR8 = __str_const("BGR8")
BGRA8 = __str_const("BGRA8")
BGR16 = __str_const("BGR16")
BGRA16 = __str_const("BGRA16")
MONO8 = __str_const("MONO8")
MONO16 = __str_const("MONO16")
TYPE_8UC1 = __str_const("TYPE_8UC1")
TYPE_8UC2 = __str_const("TYPE_8UC2")
TYPE_8UC3 = __str_const("TYPE_8UC3")
TYPE_8UC4 = __str_const("TYPE_8UC4")
TYPE_8SC1 = __str_const("TYPE_8SC1")
TYPE_8SC2 = __str_const("TYPE_8SC2")
TYPE_8SC3 = __str_const("TYPE_8SC3")
TYPE_8SC4 = __str_const("TYPE_8SC4")
TYPE_16UC1 = __str_const("TYPE_16UC1")
TYPE_16UC2 = __str_const("TYPE_16UC2")
TYPE_16UC3 = __str_const("TYPE_16UC3")
TYPE_16UC4 = __str_const("TYPE_16UC4")
TYPE_16SC1 = __str_const("TYPE_16SC1")
TYPE_16SC2 = __str_const("TYPE_16SC2")
TYPE_16SC3 = __str_const("TYPE_16SC3")
TYPE_16SC4 = __str_const("TYPE_16SC4")
TYPE_32SC1 = __str_const("TYPE_32SC1")
TYPE_32SC2 = __str_const("TYPE_32SC2")
TYPE_32SC3 = __str_const("TYPE_32SC3")
TYPE_32SC4 = __str_const("TYPE_32SC4")
TYPE_32FC1 = __str_const("TYPE_32FC1")
TYPE_32FC2 = __str_const("TYPE_32FC2")
TYPE_32FC3 = __str_const("TYPE_32FC3")
TYPE_32FC4 = __str_const("TYPE_32FC4")
TYPE_64FC1 = __str_const("TYPE_64FC1")
TYPE_64FC2 = __str_const("TYPE_64FC2")
TYPE_64FC3 = __str_const("TYPE_64FC3")
TYPE_64FC4 = __str_const("TYPE_64FC4")
BAYER_RGGB8 = __str_const("BAYER_RGGB8")
BAYER_BGGR8 = __str_const("BAYER_BGGR8")
BAYER_GBRG8 = __str_const("BAYER_GBRG8")
BAYER_GRBG8 = __str_const("BAYER_GRBG8")
BAYER_RGGB16 = __str_const("BAYER_RGGB16")
BAYER_BGGR16 = __str_const("BAYER_BGGR16")
BAYER_GBRG16 = __str_const("BAYER_GBRG16")
BAYER_GRBG16 = __str_const("BAYER_GRBG16")
YUV422 = __str_const("YUV422")


def isColor(encoding):
    fn = sensor_msgs_py.isColor
    fn.restype = ctypes.c_bool
    fn.argtypes = [ctypes.c_char_p]
    return fn(encoding.encode('ascii'))


def isMono(encoding):
    fn = sensor_msgs_py.isMono
    fn.restype = ctypes.c_bool
    fn.argtypes = [ctypes.c_char_p]
    return fn(encoding.encode('ascii'))


def isBayer(encoding):
    fn = sensor_msgs_py.isBayer
    fn.restype = ctypes.c_bool
    fn.argtypes = [ctypes.c_char_p]
    return fn(encoding.encode('ascii'))


def isDepth(encoding):
    return encoding == TYPE_16UC1 or encoding == TYPE_32FC1


def hasAlpha(encoding):
    fn = sensor_msgs_py.hasAlpha
    fn.restype = ctypes.c_bool
    fn.argtypes = [ctypes.c_char_p]
    return fn(encoding.encode('ascii'))


def numChannels(encoding):
    fn = sensor_msgs_py.numChannels
    fn.restype = ctypes.c_int
    fn.argtypes = [ctypes.c_char_p]
    return fn(encoding.encode('ascii'))


def bitDepth(encoding):
    fn = sensor_msgs_py.bitDepth
    fn.restype = ctypes.c_int
    fn.argtypes = [ctypes.c_char_p]
    return fn(encoding.encode('ascii'))
