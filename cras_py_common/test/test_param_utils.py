#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for param_utils."""

import math
import numpy as np
import unittest

from enum import Enum
from math import radians as rad

import geometry_msgs.msg as gm
import rospy
from rospy import Duration, Time, Rate
import rostest
from rosgraph_msgs.msg import Log

from cras import get_param, rate_equals, get_param_verbose, GetParamException, GetParamResult, GetParamResultInfo, \
    slowest_rate, to_str, register_default_unit, register_enum_conversion, register_param_conversion
from cras.test_utils import RosconsoleCapture


class CustomClass(object):
    def __str__(self):
        return "custom"


class TestEnum(Enum):
    a = 1,
    asd = 2


class NodeUtils(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(NodeUtils, self).__init__(*args, **kwargs)
        rospy.init_node("param_utils_test")

        register_enum_conversion(TestEnum)

    def test_get_param_basic(self):
        self.assertRaises(GetParamException, get_param, "nonexistent")
        self.assertEqual(get_param("nonexistent", "default"), "default")
        self.assertIsInstance(get_param("test_dict_config"), dict)
        self.assertEqual(get_param("test_dict_config/name"), "dict")
        self.assertIsInstance(get_param("test_dict_config/params/bool_True"), bool)
        self.assertEqual(get_param("test_dict_config/params/bool_True"), True)
        self.assertEqual(get_param("test_dict_config/params/bool_False"), False)
        self.assertEqual(get_param("test_dict_config/params/bool_true"), True)
        self.assertEqual(get_param("test_dict_config/params/bool_false"), False)
        self.assertIsInstance(get_param("test_dict_config/params/int_0"), int)
        self.assertEqual(get_param("test_dict_config/params/int_0"), 0)
        self.assertEqual(get_param("test_dict_config/params/int_1"), 1)
        self.assertEqual(get_param("test_dict_config/params/int_2"), 2)
        self.assertEqual(get_param("test_dict_config/params/int_minus_1"), -1)
        self.assertEqual(get_param("test_dict_config/params/int_max"), 2147483647)
        self.assertEqual(get_param("test_dict_config/params/int_min"), -2147483648)
        self.assertIsInstance(get_param("test_dict_config/params/double_0"), float)
        self.assertEqual(get_param("test_dict_config/params/double_0"), 0.0)
        self.assertEqual(get_param("test_dict_config/params/double_1"), 1.0)
        self.assertEqual(get_param("test_dict_config/params/double_3_14"), 3.14)
        self.assertEqual(get_param("test_dict_config/params/double_minus_1"), -1.0)
        self.assertEqual(get_param("test_dict_config/params/double_minus_3_14"), -3.14)
        self.assertEqual(get_param("test_dict_config/params/double_lowest"), -1.79769e+308)
        self.assertEqual(get_param("test_dict_config/params/double_min"), 2.22507e-308)
        self.assertEqual(get_param("test_dict_config/params/double_max"), 1.79769e+308)
        self.assertTrue(math.isnan(get_param("test_dict_config/params/double_nan")))
        self.assertEqual(get_param("test_dict_config/params/double_inf"), float('inf'))
        self.assertEqual(get_param("test_dict_config/params/double_minus_inf"), -float('inf'))
        self.assertEqual(get_param("test_dict_config/params/str_empty"), "")
        self.assertEqual(get_param("test_dict_config/params/str_a"), "a")
        self.assertEqual(get_param("test_dict_config/params/str_asd"), "asd")
        self.assertEqual(get_param("test_dict_config/params/list_empty"), list())
        self.assertEqual(get_param("test_dict_config/params/list_bool"), [True, False, True])
        self.assertEqual(get_param("test_dict_config/params/list_int"), [0, 1, -1])
        self.assertEqual(get_param("test_dict_config/params/list_uint"), [0, 1, 2])
        self.assertEqual(get_param("test_dict_config/params/list_double"), [0.0, 1.0, -1.0])
        self.assertEqual(get_param("test_dict_config/params/list_str"), ["a", "b", "cde"])
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/dict_empty")
        self.assertEqual(get_param("test_dict_config/params/dict_bool"), {"a": True, "b": False, "c": True})
        self.assertEqual(get_param("test_dict_config/params/dict_int"), {"a": 0, "b": 1, "c": -1})
        self.assertEqual(get_param("test_dict_config/params/dict_uint"), {"a": 0, "b": 1, "c": 2})
        self.assertEqual(get_param("test_dict_config/params/dict_double"), {"aaa": 0.0, "bbb": 1.0, "ccc": -1.0})
        self.assertEqual(get_param("test_dict_config/params/dict_str"), {"aaa": "a", "bbb": "b", "ccc": "c"})
        self.assertEqual(get_param("test_dict_config/params/dict_mixed"), {"bbb": 1.0, "ccc": False})
        self.assertEqual(get_param("test_dict_config/params/dict_str_recursive"),
                         {"a": {"b": 1, "c": 2}, "d": {"e": 3, "f": 4, "g": 5}})
        self.assertEqual(get_param("test_dict_config/params/dict_crazy"), {"a": {"aa": [0, 1]}, "b": {"bb": [-2, -3]}})
        self.assertEqual(get_param("test_dict_config/params/quat3"), [rad(180.0), 0.0, 0.0])
        self.assertEqual(get_param("test_dict_config/params/quat4"), [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(get_param("test_dict_config/params/tf6"), [1.0, 2.0, 3.0, rad(180.0), 0.0, 0.0])
        self.assertEqual(get_param("test_dict_config/params/tf7"), [1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(get_param("test_dict_config/params/tf16"),
                         [1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 2.0, 0.0, 0.0, -1.0, 3.0, 0.0, 0.0, 0.0, 1.0])

    def test_get_param_default_not_used(self):
        self.assertEqual(get_param("test_dict_config/name", ""), "dict")
        self.assertEqual(get_param("test_dict_config/params/bool_True", False), True)
        self.assertEqual(get_param("test_dict_config/params/bool_False", True), False)
        self.assertEqual(get_param("test_dict_config/params/bool_true", False), True)
        self.assertEqual(get_param("test_dict_config/params/bool_false", True), False)
        self.assertIsInstance(get_param("test_dict_config/params/int_0", 1), int)
        self.assertEqual(get_param("test_dict_config/params/int_0", 1), 0)
        self.assertEqual(get_param("test_dict_config/params/int_1", 0), 1)
        self.assertEqual(get_param("test_dict_config/params/int_2", 1), 2)
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", 1), -1)
        self.assertEqual(get_param("test_dict_config/params/int_max", 1), 2147483647)
        self.assertEqual(get_param("test_dict_config/params/int_min", 1), -2147483648)
        self.assertIsInstance(get_param("test_dict_config/params/double_0", 0.0), float)
        self.assertEqual(get_param("test_dict_config/params/double_0", 1.0), 0.0)
        self.assertEqual(get_param("test_dict_config/params/double_1", 0.0), 1.0)
        self.assertEqual(get_param("test_dict_config/params/double_3_14", 1.0), 3.14)
        self.assertEqual(get_param("test_dict_config/params/double_minus_1", 1.0), -1.0)
        self.assertEqual(get_param("test_dict_config/params/double_minus_3_14", 1.0), -3.14)
        self.assertEqual(get_param("test_dict_config/params/double_lowest", 1.0), -1.79769e+308)
        self.assertEqual(get_param("test_dict_config/params/double_min", 1.0), 2.22507e-308)
        self.assertEqual(get_param("test_dict_config/params/double_max", 1.0), 1.79769e+308)
        self.assertTrue(math.isnan(get_param("test_dict_config/params/double_nan", 1.0)))
        self.assertEqual(get_param("test_dict_config/params/double_inf", 1.0), float('inf'))
        self.assertEqual(get_param("test_dict_config/params/double_minus_inf", 1.0), -float('inf'))
        self.assertEqual(get_param("test_dict_config/params/str_empty", "a"), "")
        self.assertEqual(get_param("test_dict_config/params/str_a", ""), "a")
        self.assertEqual(get_param("test_dict_config/params/str_asd", ""), "asd")
        self.assertEqual(get_param("test_dict_config/params/str_asd", TestEnum.a), TestEnum.asd)
        self.assertEqual(get_param("test_dict_config/params/list_empty", [1]), list())
        self.assertEqual(get_param("test_dict_config/params/list_bool", []), [True, False, True])
        self.assertEqual(get_param("test_dict_config/params/list_int", []), [0, 1, -1])
        self.assertEqual(get_param("test_dict_config/params/list_uint", []), [0, 1, 2])
        self.assertEqual(get_param("test_dict_config/params/list_double", []), [0.0, 1.0, -1.0])
        self.assertEqual(get_param("test_dict_config/params/list_str", []), ["a", "b", "cde"])
        self.assertEqual(get_param("test_dict_config/params/dict_bool", {}), {"a": True, "b": False, "c": True})
        self.assertEqual(get_param("test_dict_config/params/dict_int", {}), {"a": 0, "b": 1, "c": -1})
        self.assertEqual(get_param("test_dict_config/params/dict_uint", {}), {"a": 0, "b": 1, "c": 2})
        self.assertEqual(get_param("test_dict_config/params/dict_double", {}), {"aaa": 0.0, "bbb": 1.0, "ccc": -1.0})
        self.assertEqual(get_param("test_dict_config/params/dict_str", {}), {"aaa": "a", "bbb": "b", "ccc": "c"})
        self.assertEqual(get_param("test_dict_config/params/dict_mixed", {}), {"bbb": 1.0, "ccc": False})
        self.assertEqual(get_param("test_dict_config/params/dict_str_recursive", {}),
                         {"a": {"b": 1, "c": 2}, "d": {"e": 3, "f": 4, "g": 5}})
        self.assertEqual(get_param("test_dict_config/params/dict_crazy", {}),
                         {"a": {"aa": [0, 1]}, "b": {"bb": [-2, -3]}})

    def test_get_param_default_used(self):
        self.assertIsInstance(get_param("nonexistent", False), bool)
        self.assertEqual(get_param("nonexistent", False), False)
        self.assertEqual(get_param("nonexistent", True), True)
        self.assertIsInstance(get_param("nonexistent", 1), int)
        self.assertEqual(get_param("nonexistent", 1), 1)
        self.assertEqual(get_param("nonexistent", 0), 0)
        self.assertEqual(get_param("nonexistent", 2), 2)
        self.assertEqual(get_param("nonexistent", -1), -1)
        self.assertIsInstance(get_param("nonexistent", 0.0), float)
        self.assertEqual(get_param("nonexistent", 1.0), 1.0)
        self.assertEqual(get_param("nonexistent", 0.0), 0.0)
        self.assertEqual(get_param("nonexistent", 3.14), 3.14)
        self.assertEqual(get_param("nonexistent", -1.0), -1.0)
        self.assertTrue(math.isnan(get_param("nonexistent", float('nan'))))
        self.assertEqual(get_param("nonexistent", float('inf')), float('inf'))
        self.assertEqual(get_param("nonexistent", -float('inf')), -float('inf'))
        self.assertEqual(get_param("nonexistent", ""), "")
        self.assertEqual(get_param("nonexistent", "a"), "a")
        self.assertEqual(get_param("nonexistent", "asd"), "asd")
        self.assertEqual(get_param("nonexistent", TestEnum.asd), TestEnum.asd)
        self.assertEqual(get_param("nonexistent", []), list())
        self.assertEqual(get_param("nonexistent", [True, False, True]), [True, False, True])
        self.assertEqual(get_param("nonexistent", [0, 1, -1]), [0, 1, -1])
        self.assertEqual(get_param("nonexistent", [0.0, 1.0, -1.0]), [0.0, 1.0, -1.0])
        self.assertEqual(get_param("nonexistent", ["a", "b", "cde"]), ["a", "b", "cde"])
        self.assertEqual(get_param("nonexistent", {"a": True, "b": False, "c": True}),
                         {"a": True, "b": False, "c": True})
        self.assertEqual(get_param("nonexistent", {"a": 0, "b": 1, "c": -1}), {"a": 0, "b": 1, "c": -1})
        self.assertEqual(get_param("nonexistent", {"bbb": 1.0, "ccc": False}), {"bbb": 1.0, "ccc": False})

    def test_get_param_default_convert(self):
        self.assertIsInstance(get_param("test_dict_config/params/bool_True", 0), int)
        self.assertEqual(get_param("test_dict_config/params/bool_True", 0), 1)
        self.assertEqual(get_param("test_dict_config/params/bool_False", 1), 0)
        self.assertIsInstance(get_param("test_dict_config/params/int_0", True), bool)
        self.assertEqual(get_param("test_dict_config/params/int_0", True), False)
        self.assertEqual(get_param("test_dict_config/params/int_1", False), True)
        self.assertEqual(get_param("test_dict_config/params/int_2", False), False)
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", False), False)
        self.assertIsInstance(get_param("test_dict_config/params/int_0", 1.0), float)
        self.assertEqual(get_param("test_dict_config/params/int_0", 1.0), 0.0)
        self.assertEqual(get_param("test_dict_config/params/int_1", 0.0), 1.0)
        self.assertEqual(get_param("test_dict_config/params/int_2", 0.0), 2.0)
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", 0.0), -1.0)
        self.assertEqual(get_param("test_dict_config/params/str_asd", TestEnum.a), TestEnum.asd)

    def test_get_param_result_convert(self):
        self.assertIsInstance(get_param("test_dict_config/params/bool_True", result_type=int), int)
        self.assertEqual(get_param("test_dict_config/params/bool_True", result_type=int), 1)
        self.assertEqual(get_param("test_dict_config/params/bool_False", result_type=int), 0)
        self.assertIsInstance(get_param("test_dict_config/params/int_0", result_type=bool), bool)
        self.assertEqual(get_param("test_dict_config/params/int_0", result_type=bool), False)
        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=bool), True)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_2", result_type=bool)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_minus_1", result_type=bool)
        self.assertIsInstance(get_param("test_dict_config/params/int_0", result_type=float), float)
        self.assertEqual(get_param("test_dict_config/params/int_0", result_type=float), 0.0)
        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=float), 1.0)
        self.assertEqual(get_param("test_dict_config/params/int_2", result_type=float), 2.0)
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", result_type=float), -1.0)
        self.assertEqual(get_param("test_dict_config/params/str_asd", result_type=TestEnum), TestEnum.asd)

    def test_get_param_result_vs_default_convert(self):
        # result_type wins over defult_value
        self.assertIsInstance(get_param("test_dict_config/params/bool_True", "", result_type=int), int)
        self.assertEqual(get_param("test_dict_config/params/bool_True", "", result_type=int), 1)
        self.assertEqual(get_param("test_dict_config/params/bool_False", "", result_type=int), 0)
        self.assertIsInstance(get_param("test_dict_config/params/int_0", "", result_type=bool), bool)
        self.assertEqual(get_param("test_dict_config/params/int_0", "", result_type=bool), False)
        self.assertEqual(get_param("test_dict_config/params/int_1", "", result_type=bool), True)
        # conversion 2->bool fails, so the default is used
        self.assertEqual(get_param("test_dict_config/params/int_2", "", result_type=bool), "")
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", "", result_type=bool), "")
        self.assertIsInstance(get_param("test_dict_config/params/int_0", "", result_type=float), float)
        self.assertEqual(get_param("test_dict_config/params/int_0", "", result_type=float), 0.0)
        self.assertEqual(get_param("test_dict_config/params/int_1", "", result_type=float), 1.0)
        self.assertEqual(get_param("test_dict_config/params/int_2", "", result_type=float), 2.0)
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", "", result_type=float), -1.0)

    def test_get_param_throw_if_convert_fails(self):
        self.assertEqual(get_param("test_dict_config/params/int_2", False, throw_if_convert_fails=False), False)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_2", False,
                          throw_if_convert_fails=True)

        try:
            get_param("test_dict_config/params/int_2", False, throw_if_convert_fails=True)
            self.fail("Expected GetParamException")
        except GetParamException as e:
            assert isinstance(e.info, GetParamResultInfo)
            self.assertTrue(e.info.convert_failed)
            self.assertFalse(e.info.required_missing)
            self.assertFalse(e.info.default_used)
            self.assertEqual(Log.ERROR, e.info.message_level)
            self.assertEqual("/param_utils_test: Parameter test_dict_config/params/int_2 found with correct XmlRpc "
                             "type int and value 2, but its conversion to type bool has failed due to the following "
                             "errors: Cannot convert int value 2 to bool.", e.info.message)
            self.assertEqual(e.info.message, str(e))

        try:
            get_param("test_dict_config/params/int_2", throw_if_convert_fails=True, result_type=bool)
            self.fail("Expected GetParamException")
        except GetParamException as e:
            assert isinstance(e.info, GetParamResultInfo)
            self.assertTrue(e.info.convert_failed)
            self.assertTrue(e.info.required_missing)
            self.assertFalse(e.info.default_used)
            self.assertEqual(Log.ERROR, e.info.message_level)
            self.assertEqual("/param_utils_test: Parameter test_dict_config/params/int_2 found with correct XmlRpc "
                             "type int and value 2, but its conversion to type bool has failed due to the following "
                             "errors: Cannot convert int value 2 to bool.", e.info.message)
            self.assertEqual(e.info.message, str(e))

        try:
            get_param("test_dict_config/name", throw_if_convert_fails=True, result_type=TestEnum)
            self.fail("Expected GetParamException")
        except GetParamException as e:
            assert isinstance(e.info, GetParamResultInfo)
            self.assertTrue(e.info.convert_failed)
            self.assertTrue(e.info.required_missing)
            self.assertFalse(e.info.default_used)
            self.assertEqual(Log.ERROR, e.info.message_level)
            self.assertIn(e.info.message,
                          ["/param_utils_test: Parameter test_dict_config/name found with correct XmlRpc type str "
                           "and value dict, but its conversion to type TestEnum has failed due to the following "
                           "errors: Cannot convert 'dict' to <enum 'TestEnum'>. Allowed values are: %s." % v
                           for v in (("asd,a",), ("a,asd",))])
            self.assertEqual(e.info.message, str(e))

    def test_get_param_verbose(self):
        self.assertIsInstance(get_param_verbose("test_dict_config"), GetParamResult)
        self.assertIsInstance(get_param_verbose("test_dict_config").value, dict)
        self.assertIsInstance(get_param_verbose("test_dict_config").info, GetParamResultInfo)

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent. " \
              "Assigning default: test."
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/nonexistent", "test")
            self.assertEqual(res.value, "test")
            self.assertEqual(res.info.default_used, True)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent2. " \
              "Assigning default: test."
        with RosconsoleCapture(self, Log.WARN, msg):
            res = get_param_verbose("test_dict_config/nonexistent2", "test", print_default_as_warn=True)
            self.assertEqual(res.value, "test")
            self.assertEqual(res.info.default_used, True)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.WARN)

        found_param = "/param_utils_test: Found parameter: %s, value: %s."

        msg = found_param % ("test_dict_config/params/list_bool", "[True, False, True]")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/list_bool")
            self.assertEqual(res.value, [True, False, True])
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/list_int", "[0, 1, -1]")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/list_int")
            self.assertEqual(res.value, [0, 1, -1])
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/list_double", "[0.0, 1.0, -1.0]")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/list_double")
            self.assertEqual(res.value, [0.0, 1.0, -1.0])
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/list_str", "['a', 'b', 'cde']")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/list_str")
            self.assertEqual(res.value, ["a", "b", "cde"])
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/dict_bool", "{'a': True, 'b': False, 'c': True}")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/dict_bool")
            self.assertEqual(res.value, {"a": True, "b": False, "c": True})
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)

        msg = found_param % ("test_dict_config/params/dict_int", "{'a': 0, 'b': 1, 'c': -1}")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/dict_int")
            self.assertEqual(res.value, {"a": 0, "b": 1, "c": -1})
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/dict_double", "{'aaa': 0.0, 'bbb': 1.0, 'ccc': -1.0}")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/dict_double")
            self.assertEqual(res.value, {"aaa": 0.0, "bbb": 1.0, "ccc": -1.0})
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/dict_str", "{'aaa': 'a', 'bbb': 'b', 'ccc': 'c'}")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/dict_str")
            self.assertEqual(res.value, {"aaa": "a", "bbb": "b", "ccc": "c"})
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/dict_mixed", "{'bbb': 1.0, 'ccc': False}")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/dict_mixed")
            self.assertEqual(res.value, {"bbb": 1.0, "ccc": False})
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = found_param % ("test_dict_config/params/dict_str_recursive",
                             "{'a': {'b': 1, 'c': 2}, 'd': {'e': 3, 'f': 4, 'g': 5}}")
        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/params/dict_str_recursive")
            self.assertEqual(res.value, {"a": {"b": 1, "c": 2}, "d": {"e": 3, "f": 4, "g": 5}})
            self.assertEqual(res.info.default_used, False)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent. " \
              "Assigning default: test."
        with RosconsoleCapture(self):
            res = get_param_verbose("test_dict_config/nonexistent", "test", print_messages=False)
            self.assertEqual(res.value, "test")
            self.assertEqual(res.info.default_used, True)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        with RosconsoleCapture(self, Log.INFO, msg):
            res = get_param_verbose("test_dict_config/nonexistent", "test", print_messages=True)
            self.assertEqual(res.value, "test")
            self.assertEqual(res.info.default_used, True)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.INFO)

        with RosconsoleCapture(self):
            res = get_param_verbose("test_dict_config/nonexistent", "test", print_default_as_warn=True,
                                    print_messages=False)
            self.assertEqual(res.value, "test")
            self.assertEqual(res.info.default_used, True)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.WARN)

        with RosconsoleCapture(self, Log.WARN, msg):
            res = get_param_verbose("test_dict_config/nonexistent", "test", print_default_as_warn=True,
                                    print_messages=True)
            self.assertEqual(res.value, "test")
            self.assertEqual(res.info.default_used, True)
            self.assertEqual(res.info.message, msg)
            self.assertEqual(res.info.message_level, Log.WARN)

    def test_get_param_units(self):
        msg = "/param_utils_test: Found parameter: test_dict_config/params/double_1, value: 1.0 units."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/params/double_1", unit="units")

        msg = "/param_utils_test: Found parameter: test_dict_config/params/double_1, value: 1.0."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/params/double_1", unit="")

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent. " \
              "Assigning default: custom."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/nonexistent", CustomClass())

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent. " \
              "Assigning default: custom units."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/nonexistent", CustomClass(), unit="units")

        register_default_unit(CustomClass, "cras")

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent. " \
              "Assigning default: custom cras."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/nonexistent", CustomClass())

        msg = "/param_utils_test: Cannot find value for parameter: test_dict_config/nonexistent. " \
              "Assigning default: custom units."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/nonexistent", CustomClass(), unit="units")

    def test_get_param_custom_conversion(self):
        err = ""
        try:
            CustomClass(1)
        except Exception as e:
            err = str(e)
        msg = "/param_utils_test: Parameter test_dict_config/params/double_1 found with correct XmlRpc type float " \
              "and value 1.0, but its conversion to type CustomClass has failed due to the following " \
              "errors: %s." % (err,)
        with RosconsoleCapture(self, Log.ERROR, msg):
            self.assertRaises(GetParamException, get_param, "test_dict_config/params/double_1", result_type=CustomClass)

        register_param_conversion(CustomClass, float, lambda _: CustomClass())

        msg = "/param_utils_test: Found parameter: test_dict_config/params/double_1, value: custom."
        with RosconsoleCapture(self, Log.INFO, msg):
            get_param("test_dict_config/params/double_1", result_type=CustomClass)

    def test_get_param_convert_rospy(self):
        default = Duration(2)
        self.assertEqual(get_param("test_dict_config/params/double_1", default), Duration(1))
        self.assertEqual(get_param("test_dict_config/params/double_minus_1", default), Duration(-1))
        self.assertEqual(get_param("test_dict_config/params/int_1", default), Duration(1))
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", default), Duration(-1))
        self.assertEqual(get_param("test_dict_config/params/timeout", default), Duration(1, 2))
        self.assertEqual(get_param("test_dict_config/params/timeout_negative", default), Duration(-1, 2))
        # wrong XmlRpc type, default should be used
        self.assertEqual(get_param("test_dict_config/params/bool_True", default), default)
        self.assertEqual(get_param("test_dict_config/params/marker", default), default)

        # Time cannot be negative
        default = Time(2)
        self.assertEqual(get_param("test_dict_config/params/double_1", default), Time(1))
        self.assertEqual(get_param("test_dict_config/params/double_minus_1", default), default)
        self.assertEqual(get_param("test_dict_config/params/int_1", default), Time(1))
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", default), default)
        self.assertEqual(get_param("test_dict_config/params/timeout", default), Time(1, 2))
        self.assertEqual(get_param("test_dict_config/params/timeout_negative", default), default)
        # wrong XmlRpc type, default should be used
        self.assertEqual(get_param("test_dict_config/params/bool_True", default), default)
        self.assertEqual(get_param("test_dict_config/params/marker", default), default)

        default = Rate(2)
        self.assertTrue(rate_equals(get_param("test_dict_config/params/double_1", default), Rate(1)))
        self.assertTrue(rate_equals(get_param("test_dict_config/params/double_minus_1", default), Rate(-1)))
        self.assertTrue(rate_equals(get_param("test_dict_config/params/int_1", default), Rate(1)))
        self.assertTrue(rate_equals(get_param("test_dict_config/params/int_minus_1", default), Rate(-1)))
        # Rate cannot be read from sec+nsec
        self.assertTrue(rate_equals(get_param("test_dict_config/params/timeout", default), default))
        self.assertTrue(rate_equals(get_param("test_dict_config/params/timeout_negative", default), default))
        # wrong XmlRpc type, default should be used
        self.assertTrue(rate_equals(get_param("test_dict_config/params/bool_True", default), default))
        self.assertTrue(rate_equals(get_param("test_dict_config/params/marker", default), default))

        # Rate has a default unit Hz
        msg = "/param_utils_test: Found parameter: test_dict_config/params/double_1, value: 1.0 Hz."
        with RosconsoleCapture(self, Log.INFO, msg):
            self.assertTrue(rate_equals(get_param("test_dict_config/params/double_1", default), Rate(1)))

        # Rate with 0 frequency should be safely read as the slowest possible representable rate.
        msg = "/param_utils_test: Found parameter: test_dict_config/params/double_0, value: %s Hz." % (
            to_str(slowest_rate()),)
        with RosconsoleCapture(self, Log.INFO, msg):
            self.assertTrue(rate_equals(get_param("test_dict_config/params/double_0", default), slowest_rate()))

        default = Time(2)
        msg = "/param_utils_test: Found parameter: test_dict_config/params/double_1, value: 1.000000000."
        with RosconsoleCapture(self, Log.INFO, msg):
            self.assertEqual(get_param("test_dict_config/params/double_1", default), Time(1))

        msg = "/param_utils_test: Cannot find value for parameter: nonexistent. Assigning default: 2.000000000."
        with RosconsoleCapture(self, Log.INFO, msg):
            self.assertEqual(get_param("nonexistent", default), default)

        msg = "/param_utils_test: Parameter test_dict_config/params/double_minus_1 found with correct XmlRpc type " \
              "float and value -1.0, but its conversion to type Time has failed due to the following errors: " \
              "time values must be positive. Assigning default: 2.000000000."
        with RosconsoleCapture(self, Log.ERROR, msg):
            self.assertEqual(get_param("test_dict_config/params/double_minus_1", default), default)

        default = Duration(2)
        msg = "/param_utils_test: Parameter test_dict_config/params/str_a found, but it has wrong XmlRpc type. " \
              "Expected one of dict, float, int, got type str with value a. Assigning default: 2.000000000 s."
        with RosconsoleCapture(self, Log.ERROR, msg):
            self.assertEqual(get_param("test_dict_config/params/str_a", default), default)

        msg = "/param_utils_test: Required parameter nonexistent is not set."
        with RosconsoleCapture(self, Log.ERROR, msg):
            self.assertRaises(GetParamException, get_param, "nonexistent")

    def test_get_param_convert_numpy(self):
        self.assertEqual(get_param("test_dict_config/params/bool_True", result_type=np.bool), np.bool(True))
        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.int), np.int(1))
        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.long), np.long(1))
        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.float), np.float(1))
        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.double), np.double(1))
        self.assertEqual(get_param("test_dict_config/params/double_1", result_type=np.float), np.float(1))
        self.assertEqual(get_param("test_dict_config/params/double_1", result_type=np.double), np.double(1))

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.int8), np.int8(1))
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", result_type=np.int8), np.int8(-1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_max", result_type=np.int8)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.uint8), np.uint8(1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_minus_1", result_type=np.uint8)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_max", result_type=np.uint8)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.int16), np.int16(1))
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", result_type=np.int16), np.int16(-1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_max", result_type=np.int16)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.uint16), np.uint16(1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_minus_1", result_type=np.uint16)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_max", result_type=np.uint16)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.int32), np.int32(1))
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", result_type=np.int32), np.int32(-1))
        self.assertEqual(get_param("test_dict_config/params/int_max", result_type=np.int32), np.iinfo(np.int32).max)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.uint32), np.uint32(1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_minus_1", result_type=np.uint32)
        self.assertEqual(get_param("test_dict_config/params/int_max", result_type=np.uint32), np.iinfo(np.int32).max)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.int64), np.int64(1))
        self.assertEqual(get_param("test_dict_config/params/int_minus_1", result_type=np.int64), np.int64(-1))
        self.assertEqual(get_param("test_dict_config/params/int_max", result_type=np.int64), np.iinfo(np.int32).max)

        self.assertEqual(get_param("test_dict_config/params/int_1", result_type=np.uint64), np.uint64(1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/int_minus_1", result_type=np.uint64)
        self.assertEqual(get_param("test_dict_config/params/int_max", result_type=np.uint64), np.iinfo(np.int32).max)

        self.assertEqual(get_param("test_dict_config/params/double_1", result_type=np.float32), np.float32(1))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/double_max", result_type=np.float32)

        self.assertEqual(get_param("test_dict_config/params/double_1", result_type=np.float64), np.float64(1))
        self.assertEqual(get_param("test_dict_config/params/double_max", result_type=np.float64), 1.79769e+308)

        self.assertTrue(isinstance(get_param("test_dict_config/params/list_int", result_type=np.ndarray), np.ndarray))
        self.assertTrue(
            (get_param("test_dict_config/params/list_int", result_type=np.ndarray) == np.array([0, 1, -1])).all())

        self.assertTrue(isinstance(get_param("test_dict_config/params/list_int", result_type=np.matrix), np.matrix))
        self.assertTrue(
            (get_param("test_dict_config/params/list_int", result_type=np.matrix) == np.matrix([0, 1, -1])).all())

    def test_get_param_convert_geometry_msgs(self):
        self.assertEqual(get_param("test_dict_config/params/list_int", result_type=gm.Vector3), gm.Vector3(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/point", result_type=gm.Vector3), gm.Vector3(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/list_int", result_type=gm.Point), gm.Point(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/point", result_type=gm.Point), gm.Point(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/list_int", result_type=gm.Point32), gm.Point32(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/point", result_type=gm.Point32), gm.Point32(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/list_int", result_type=gm.Pose2D), gm.Pose2D(0, 1, -1))
        self.assertEqual(get_param("test_dict_config/params/pose_2d", result_type=gm.Pose2D), gm.Pose2D(0, 1, -1))

        self.assertRaises(GetParamException, get_param, "test_dict_config/params/quat4", result_type=gm.Vector3)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/pose_2d", result_type=gm.Vector3)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/timeout", result_type=gm.Vector3)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/str_a", result_type=gm.Vector3)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/point", result_type=gm.Pose2D)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/timeout", result_type=gm.Pose2D)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/str_a", result_type=gm.Pose2D)

        def quat_almost_equal(q1, q2):
            self.assertAlmostEqual(q1.x, q2.x, delta=1e-6)
            self.assertAlmostEqual(q1.y, q2.y, delta=1e-6)
            self.assertAlmostEqual(q1.z, q2.z, delta=1e-6)
            self.assertAlmostEqual(q1.w, q2.w, delta=1e-6)

        self.assertEqual(
            get_param("test_dict_config/params/quat4", result_type=gm.Quaternion), gm.Quaternion(1, 0, 0, 0))
        quat_almost_equal(
            get_param("test_dict_config/params/quat3", result_type=gm.Quaternion), gm.Quaternion(1, 0, 0, 0))
        quat_almost_equal(
            get_param("test_dict_config/params/quat9", result_type=gm.Quaternion), gm.Quaternion(1, 0, 0, 0))

        self.assertEqual(
            get_param("test_dict_config/params/quat_dict_4_full", result_type=gm.Quaternion), gm.Quaternion(1, 0, 0, 0))
        self.assertEqual(
            get_param("test_dict_config/params/quat_dict_4_part", result_type=gm.Quaternion), gm.Quaternion(1, 0, 0, 0))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/quat_dict_4_y_only",
                          throw_if_convert_fails=True, result_type=gm.Quaternion)
        quat_almost_equal(
            get_param("test_dict_config/params/quat_dict_3_short_full", result_type=gm.Quaternion),
            gm.Quaternion(0, 1, 0, 0))
        quat_almost_equal(
            get_param("test_dict_config/params/quat_dict_3_short_part", result_type=gm.Quaternion),
            gm.Quaternion(0, 1, 0, 0))
        quat_almost_equal(
            get_param("test_dict_config/params/quat_dict_3_long_full", result_type=gm.Quaternion),
            gm.Quaternion(0, 1, 0, 0))
        quat_almost_equal(
            get_param("test_dict_config/params/quat_dict_3_long_part", result_type=gm.Quaternion),
            gm.Quaternion(0, 1, 0, 0))
        quat_almost_equal(
            get_param("test_dict_config/params/quat_dict_3_long_yaw_only", result_type=gm.Quaternion),
            gm.Quaternion(0, 0, 1, 0))
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/quat_dict_wrong",
                          throw_if_convert_fails=True, result_type=gm.Quaternion)
        self.assertRaises(GetParamException, get_param, "test_dict_config/params/quat_dict_wrong_mixed",
                          throw_if_convert_fails=True, result_type=gm.Quaternion)

        t = get_param("test_dict_config/params/tf7", result_type=gm.Pose)
        self.assertEqual(t.position, gm.Point(1, 2, 3))
        self.assertEqual(t.orientation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/tf6", result_type=gm.Pose)
        self.assertEqual(t.position, gm.Point(1, 2, 3))
        quat_almost_equal(t.orientation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/tf16", result_type=gm.Pose)
        self.assertEqual(t.position, gm.Point(1, 2, 3))
        quat_almost_equal(t.orientation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/tf7", result_type=gm.Transform)
        self.assertEqual(t.translation, gm.Vector3(1, 2, 3))
        self.assertEqual(t.rotation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/tf6", result_type=gm.Transform)
        self.assertEqual(t.translation, gm.Vector3(1, 2, 3))
        quat_almost_equal(t.rotation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/tf16", result_type=gm.Transform)
        self.assertEqual(t.translation, gm.Vector3(1, 2, 3))
        quat_almost_equal(t.rotation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/pose_dict_lists", result_type=gm.Pose)
        self.assertEqual(t.position, gm.Point(1, 2, 3))
        quat_almost_equal(t.orientation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/pose_dict_dicts", result_type=gm.Pose)
        self.assertEqual(t.position, gm.Point(1, 2, 3))
        quat_almost_equal(t.orientation, gm.Quaternion(0, 0, 0, 1))

        t = get_param("test_dict_config/params/transform_dict_lists", result_type=gm.Transform)
        self.assertEqual(t.translation, gm.Vector3(1, 2, 3))
        quat_almost_equal(t.rotation, gm.Quaternion(1, 0, 0, 0))

        t = get_param("test_dict_config/params/transform_dict_dicts", result_type=gm.Transform)
        self.assertEqual(t.translation, gm.Vector3(1, 2, 3))
        quat_almost_equal(t.rotation, gm.Quaternion(0, 0, 0, 1))


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_param_utils", NodeUtils)
