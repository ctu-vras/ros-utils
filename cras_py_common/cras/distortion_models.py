# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Python bindings for sensor_msgs/distortion_models.h"""


import ctypes

from .ctypes_utils import load_library


sensor_msgs_py = load_library("sensor_msgs_py")


def __str_const(name):
    return ctypes.c_char_p.in_dll(sensor_msgs_py, name).value.decode("ascii")


PLUMB_BOB = __str_const("PLUMB_BOB")
RATIONAL_POLYNOMIAL = __str_const("RATIONAL_POLYNOMIAL")
EQUIDISTANT = __str_const("EQUIDISTANT")
