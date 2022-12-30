# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Numpy parsers for ROS parameters."""

from functools import partial

from cras.string_utils import to_str
import cras.param_utils as param_utils


def __convert_with_bounds_check(target_type, min_value, max_value, value):
    if min_value <= value <= max_value:
        return target_type(value)
    raise ValueError("Cannot convert %s value %s to %s" % (to_str(type(value)), str(value), to_str(target_type)))


try:
    import numpy as np

    def __convert_np_int_with_bounds_check(target_type, value):
        iinfo = np.iinfo(target_type)
        return __convert_with_bounds_check(target_type, iinfo.min, iinfo.max, value)

    def __convert_np_float_with_bounds_check(target_type, value):
        iinfo = np.finfo(target_type)
        return __convert_with_bounds_check(target_type, iinfo.min, iinfo.max, value)

    def __convert_list_to_ndarray(target_type, value):
        if not isinstance(value, list):
            raise ValueError("Can only convert lists to ndarray, but %s was given." % (to_str(type(value)),))
        return target_type(value)

    param_utils.register_param_conversion(np.bool, bool, np.bool)
    param_utils.register_param_conversion(np.int, int, partial(__convert_np_int_with_bounds_check, np.int))
    param_utils.register_param_conversion(np.long, int, partial(__convert_np_int_with_bounds_check, np.long))
    param_utils.register_param_conversion(np.float, int, partial(__convert_np_float_with_bounds_check, np.float))
    param_utils.register_param_conversion(np.float, float, partial(__convert_np_float_with_bounds_check, np.float))
    param_utils.register_param_conversion(np.double, int, partial(__convert_np_float_with_bounds_check, np.double))
    param_utils.register_param_conversion(np.double, float, partial(__convert_np_float_with_bounds_check, np.double))
    param_utils.register_param_conversion(np.uint8, int, partial(__convert_np_int_with_bounds_check, np.uint8))
    param_utils.register_param_conversion(np.uint16, int, partial(__convert_np_int_with_bounds_check, np.uint16))
    param_utils.register_param_conversion(np.uint32, int, partial(__convert_np_int_with_bounds_check, np.uint32))
    param_utils.register_param_conversion(np.uint64, int, partial(__convert_np_int_with_bounds_check, np.uint64))
    param_utils.register_param_conversion(np.int8, int, partial(__convert_np_int_with_bounds_check, np.int8))
    param_utils.register_param_conversion(np.int16, int, partial(__convert_np_int_with_bounds_check, np.int16))
    param_utils.register_param_conversion(np.int32, int, partial(__convert_np_int_with_bounds_check, np.int32))
    param_utils.register_param_conversion(np.int64, int, partial(__convert_np_int_with_bounds_check, np.int64))
    param_utils.register_param_conversion(np.float32, int, partial(__convert_np_float_with_bounds_check, np.float32))
    param_utils.register_param_conversion(np.float32, float, partial(__convert_np_float_with_bounds_check, np.float32))
    param_utils.register_param_conversion(np.float64, int, partial(__convert_np_float_with_bounds_check, np.float64))
    param_utils.register_param_conversion(np.float64, float, partial(__convert_np_float_with_bounds_check, np.float64))
    param_utils.register_param_conversion(np.ndarray, list, partial(__convert_list_to_ndarray, np.array))
    param_utils.register_param_conversion(np.matrix, list, partial(__convert_list_to_ndarray, np.matrix))
except ImportError:
    pass
