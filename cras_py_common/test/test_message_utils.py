#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for message_utils."""

import unittest

from cras import dict_to_dynamic_config_msg, get_msg_field, get_msg_type, get_srv_types, get_cfg_module


class MessageUtils(unittest.TestCase):

    def test_get_msg_field(self):
        from std_msgs.msg import Int32MultiArray, MultiArrayDimension
        msg = Int32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="x", size=1, stride=1))
        msg.layout.dim.append(MultiArrayDimension(label="y", size=2, stride=2))
        msg.data.append(1)
        msg.data.append(2)
        msg.data.append(3)
        self.assertEqual("x", get_msg_field(msg, "layout/dim[0]/label"))
        self.assertEqual(1, get_msg_field(msg, "layout/dim[0]/size"))
        self.assertEqual(1, get_msg_field(msg, "layout/dim[0]/stride"))
        self.assertEqual("y", get_msg_field(msg, "layout/dim[1]/label"))
        self.assertEqual(2, get_msg_field(msg, "layout/dim[1]/size"))
        self.assertEqual(2, get_msg_field(msg, "layout/dim[1]/stride"))
        self.assertEqual(1, get_msg_field(msg, "data[0]"))
        self.assertEqual(2, get_msg_field(msg, "data[1]"))
        self.assertEqual(3, get_msg_field(msg, "data[2]"))

        # test with a different separator
        self.assertEqual("x", get_msg_field(msg, "layout.dim[0].label", sep="."))

    def test_get_msg_type(self):
        cls = get_msg_type("std_msgs/String")
        self.assertEqual(cls._type, "std_msgs/String")
        self.assertTrue(hasattr(cls, "data"))

        msg = cls()
        from std_msgs.msg import String
        self.assertIsInstance(msg, String)

    def test_get_srv_types(self):
        cls, cls_req, cls_resp = get_srv_types("std_srvs/SetBool")
        self.assertEqual(cls._type, "std_srvs/SetBool")
        self.assertEqual(cls_req._type, "std_srvs/SetBoolRequest")
        self.assertEqual(cls_resp._type, "std_srvs/SetBoolResponse")
        self.assertTrue(hasattr(cls_req, "data"))
        self.assertTrue(hasattr(cls_resp, "success"))

        from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
        self.assertIsInstance(cls(), SetBool)
        self.assertIsInstance(cls_req(), SetBoolRequest)
        self.assertIsInstance(cls_resp(), SetBoolResponse)

    def test_get_cfg_type(self):
        module = get_cfg_module("cras_cpp_common/FilterChain")
        self.assertEqual(module.__name__, "cras_cpp_common.cfg.FilterChainConfig")
        self.assertTrue(hasattr(module, "defaults"))
        self.assertTrue(hasattr(module, "min"))

        from cras_cpp_common.cfg import FilterChainConfig
        self.assertEqual(getattr(module, "defaults"), FilterChainConfig.defaults)

    def test_dict_to_dynamic_config(self):
        d = {
            "a": False,
            "b": True,
            "c": 1,
            "d": 2.0,
            "e": "str",
        }
        config = dict_to_dynamic_config_msg(d)

        self.assertEqual(len(config.bools), 2)
        self.assertEqual(len(config.ints), 1)
        self.assertEqual(len(config.doubles), 1)
        self.assertEqual(len(config.strs), 1)

        if config.bools[0].name == "a":
            self.assertEqual(config.bools[0].name, "a")
            self.assertEqual(config.bools[0].value, False)
            self.assertEqual(config.bools[1].name, "b")
            self.assertEqual(config.bools[1].value, True)
        else:
            self.assertEqual(config.bools[0].name, "a")
            self.assertEqual(config.bools[0].value, True)
            self.assertEqual(config.bools[1].name, "b")
            self.assertEqual(config.bools[1].value, False)

        self.assertEqual(config.ints[0].name, "c")
        self.assertEqual(config.ints[0].value, 1)
        self.assertEqual(config.doubles[0].name, "d")
        self.assertEqual(config.doubles[0].value, 2.0)
        self.assertEqual(config.strs[0].name, "e")
        self.assertEqual(config.strs[0].value, "str")

        self.assertEqual(dict_to_dynamic_config_msg(config), config)


if __name__ == '__main__':
    unittest.main()
