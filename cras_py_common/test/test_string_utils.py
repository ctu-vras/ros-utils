#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.string_utils"""
import os
from enum import Enum
import unittest

import rospy
from rospy import Duration, Rate, Time
from geometry_msgs.msg import Transform

from cras import to_str, register_to_str
from cras.string_utils import BufferStringIO


class TestClass:
    def __str__(self):
        return "test"


class TestEnum(Enum):
    A = 1,
    B = 2,


class TestEnum2(Enum):
    A = 1,
    B = 2,


class StringUtils(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(StringUtils, self).__init__(*args, **kwargs)
        rospy.rostime.set_rostime_initialized(True)

    def test_to_str(self):
        self.assertEqual("True", to_str(True))
        self.assertEqual("1", to_str(1))
        self.assertEqual("1.0", to_str(1.0))
        self.assertEqual("3.14", to_str(3.14))
        self.assertEqual("abc", to_str("abc"))
        self.assertEqual("A", to_str(TestEnum.A))
        self.assertEqual("B", to_str(TestEnum.B))

        self.assertEqual("1.000000000", to_str(Duration(1)))
        self.assertEqual("1.000000001", to_str(Duration(1, 1)))
        self.assertEqual("1.100000000", to_str(Duration(1, 100000000)))
        self.assertEqual("-1.000000000", to_str(-Duration(1)))
        self.assertEqual("-1.000000001", to_str(Duration(-1, 1)))
        self.assertEqual("-1.100000000", to_str(Duration(-1, 100000000)))
        self.assertEqual("1.000000000", to_str(Time(1)))
        self.assertEqual("1.000000001", to_str(Time(1, 1)))
        self.assertEqual("1.100000000", to_str(Time(1, 100000000)))
        self.assertEqual("1.0", to_str(Rate(1)))
        self.assertEqual("10.0", to_str(Rate(10))[:4])
        self.assertEqual("-10.0", to_str(Rate(-10))[:5])  # on Noetic, the number is not exactly 10

        # use of __name__ if defined
        self.assertEqual("int", to_str(type(1)))

        # test iterables
        self.assertEqual("[True, False]", to_str([True, False]))
        self.assertEqual("[1, 0, -1]", to_str([1, 0, -1]))
        self.assertEqual("[1.0, 0.0, -1.1]", to_str([1.0, 0.0, -1.1]))
        self.assertEqual("['a', 'bc']", to_str(["a", "bc"]))
        self.assertEqual("[1.000000000, 2.000000001]", to_str([Duration(1), Duration(2, 1)]))
        self.assertEqual("[1.000000000, 2.000000001]", to_str([Time(1), Time(2, 1)]))
        self.assertEqual("[1.000000000, 2.000000001]", to_str([Time(1), Duration(2, 1)]))
        self.assertEqual("[1.0, 10.0]", to_str([Rate(1), Rate(10)]))

        self.assertEqual("(True, False)", to_str((True, False)))
        self.assertEqual("(1, 0, -1)", to_str((1, 0, -1)))
        self.assertEqual("(1.0, 0.0, -1.1)", to_str((1.0, 0.0, -1.1)))
        self.assertEqual("('a', 'bc')", to_str(("a", "bc")))
        self.assertEqual("(1.000000000, 2.000000001)", to_str((Duration(1), Duration(2, 1))))
        self.assertEqual("(1.000000000, 2.000000001)", to_str((Time(1), Time(2, 1))))
        self.assertEqual("(1.000000000, 2.000000001)", to_str((Time(1), Duration(2, 1))))
        self.assertEqual("(1.0, 10.0)", to_str((Rate(1), Rate(10))))

        self.assertEqual("{False, True}", to_str({True, False}))
        self.assertEqual("{-1, 0, 1}", to_str({1, 0, -1}))
        self.assertEqual("{-1.1, 0.0, 1.0}", to_str({1.0, 0.0, -1.1}))
        self.assertEqual("{'a', 'bc'}", to_str({"bc", "a"}))
        self.assertEqual("{1.000000000, 2.000000001}", to_str({Duration(1), Duration(2, 1)}))
        self.assertEqual("{1.000000000, 2.000000001}", to_str({Time(1), Time(2, 1)}))
        self.assertEqual("{1.000000000, 2.000000001}", to_str({Time(1), Duration(2, 1)}))
        self.assertEqual("{1.0, 10.0}", to_str({Rate(1), Rate(10)}))

        self.assertEqual("{False: True, True: False}", to_str({True: False, False: True}))
        self.assertEqual("{-1: 'cc', 0: 'b', 1: 'A'}", to_str({1: "A", 0: "b", -1: "cc"}))
        self.assertEqual("{-1.1: 'cc', 0.0: 'b', 1.0: 'A'}", to_str({1.0: "A", 0.0: "b", -1.1: "cc"}))
        self.assertEqual("{'a': 2, 'bc': 1}", to_str({"bc": 1, "a": 2}))
        self.assertEqual("{1.000000000: 'A', 2.000000001: 'b'}", to_str({Duration(1): "A", Duration(2, 1): "b"}))
        self.assertEqual("{1.000000000: 'A', 2.000000001: 'b'}", to_str({Time(1): "A", Time(2, 1): "b"}))
        self.assertEqual("{1.000000000: 'A', 2.000000001: 'b'}", to_str({Time(1): "A", Duration(2, 1): "b"}))
        self.assertEqual("{1.0: 1, 10.0: 2}", to_str({Rate(1): 1, Rate(10): 2}))

        # recursive iterables
        self.assertEqual("[(True, False), {1, 2}]", to_str([(True, False), {2, 1}]))
        self.assertEqual("{'a': [1.000000000, {2.000000000}]}", to_str({"a": [Duration(1), {Time(2)}]}))

        # ROS messages (not very nice for recursive messages)
        self.assertEqual(
            "translation: ,   x: 0.0,   y: 0.0,   z: 0.0, rotation: ,   x: 0.0,   y: 0.0,   z: 0.0,   w: 0.0",
            to_str(Transform()))

    def test_register_to_str(self):
        self.assertEqual("test", to_str(TestClass()))
        register_to_str(TestClass, lambda _: "new")
        self.assertEqual("new", to_str(TestClass()))
        register_to_str(TestEnum2, lambda _: "test")
        self.assertEqual("test", to_str(TestEnum2.A))

    def test_buffer_string_io(self):
        buf = BufferStringIO(b'test')
        self.assertEqual(type(buf.getvalue()), bytes)
        self.assertEqual(buf.getvalue(), b'test')
        self.assertEqual(buf.tell(), 0)
        buf.seek(0, os.SEEK_END)
        self.assertEqual(buf.tell(), 4)

        buf = BufferStringIO()
        buf.write(b'test')
        self.assertEqual(type(buf.getvalue()), bytes)
        self.assertEqual(buf.getvalue(), b'test')
        self.assertEqual(buf.tell(), 4)
        buf.seek(0, os.SEEK_SET)
        self.assertEqual(buf.tell(), 0)
        self.assertEqual(buf.getvalue(), b'test')


if __name__ == '__main__':
    unittest.main()
