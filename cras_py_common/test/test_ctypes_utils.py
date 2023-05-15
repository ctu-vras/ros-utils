#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.ctypes_utils"""

from ctypes import c_double, string_at, Array
import math
import unittest

import rospy
from std_msgs.msg import Header
from rosgraph_msgs.msg import Log

from cras.ctypes_utils import load_library, StringAllocator, BytesAllocator, RosMessagesAllocator, \
    LogMessagesAllocator, ScalarAllocator, get_ro_c_buffer, c_array
from cras.string_utils import BufferStringIO
from cras.test_utils import RosconsoleCapture


class TestLogAllocator(LogMessagesAllocator):
    def format_log_message(self, msg):
        return str(msg)


class CtypesUtils(unittest.TestCase):

    def test_load_library(self):
        libm = load_library("m")
        self.assertIsNotNone(libm)

        libm.cos.restype = c_double
        libm.cos.argtypes = [c_double]

        self.assertAlmostEqual(libm.cos(0), math.cos(0))
        self.assertAlmostEqual(libm.cos(0.5), math.cos(0.5))
        self.assertAlmostEqual(libm.cos(1.5), math.cos(1.5))

    def test_string_allocator(self):
        a = StringAllocator()
        self.assertEqual(0, len(a.allocated))
        self.assertEqual(0, len(a.allocated_sizes))
        self.assertIsNone(a.value)
        self.assertEqual(0, len(a.values))
        a(4)
        a.allocated[0][0] = 'a'.encode()
        a.allocated[0][1] = 'b'.encode()
        a.allocated[0][2] = 'c'.encode()
        a.allocated[0][3] = 'd'.encode()
        self.assertEqual(len(a.allocated), 1)
        self.assertEqual(len(a.allocated_sizes), 1)
        self.assertEqual(len(a.value), 4)
        self.assertEqual(a.allocated_sizes[0], 4)
        self.assertEqual(a.value, "abcd")
        self.assertEqual(a.values, ["abcd"])
        a(5)
        a.allocated[1][0] = 'e'.encode()
        a.allocated[1][1] = 'f'.encode()
        a.allocated[1][2] = 'g'.encode()
        a.allocated[1][3] = 'h'.encode()
        a.allocated[1][4] = 'i'.encode()
        self.assertEqual(len(a.allocated), 2)
        self.assertEqual(len(a.allocated_sizes), 2)
        self.assertEqual(len(a.value), 4)
        self.assertEqual(len(a.allocated[0]), 4)
        self.assertEqual(len(a.allocated[1]), 5)
        self.assertEqual(a.allocated_sizes[0], 4)
        self.assertEqual(a.allocated_sizes[1], 5)
        self.assertEqual(a.value, "abcd")
        self.assertEqual(a.values, ["abcd", "efghi"])

    def test_bytes_allocator(self):
        a = BytesAllocator()
        self.assertEqual(0, len(a.allocated))
        self.assertEqual(0, len(a.allocated_sizes))
        self.assertIsNone(a.value)
        self.assertEqual(0, len(a.values))
        a(4)
        a.allocated[0][0] = 0
        a.allocated[0][1] = 1
        a.allocated[0][2] = 2
        a.allocated[0][3] = 3
        self.assertEqual(len(a.allocated), 1)
        self.assertEqual(len(a.allocated_sizes), 1)
        self.assertEqual(len(a.value), 4)
        self.assertEqual(a.allocated_sizes[0], 4)
        a(5)
        a.allocated[1][0] = 4
        a.allocated[1][1] = 5
        a.allocated[1][2] = 6
        a.allocated[1][3] = 7
        a.allocated[1][4] = 8
        self.assertEqual(len(a.allocated), 2)
        self.assertEqual(len(a.allocated_sizes), 2)
        self.assertEqual(len(a.value), 4)
        self.assertEqual(len(a.allocated[0]), 4)
        self.assertEqual(len(a.allocated[1]), 5)
        self.assertEqual(a.allocated_sizes[0], 4)
        self.assertEqual(a.allocated_sizes[1], 5)
        self.assertEqual(a.value, b'\x00\x01\x02\x03')
        self.assertEqual(a.values, [b'\x00\x01\x02\x03', b'\x04\x05\x06\x07\x08'])

    def test_scalar_allocator(self):
        a = ScalarAllocator(c_double)
        self.assertEqual(0, len(a.allocated))
        self.assertEqual(0, len(a.allocated_sizes))
        self.assertIsNone(a.value)
        self.assertEqual(0, len(a.values))
        a(8)
        self.assertEqual(len(a.allocated), 1)
        a.allocated[0][0] = 4.0
        self.assertEqual(len(a.allocated_sizes), 1)
        self.assertEqual(a.value, 4.0)
        self.assertEqual(a.values[0], 4.0)
        self.assertEqual(a.allocated_sizes[0], 8)
        a(8)
        self.assertEqual(len(a.allocated), 2)
        a.allocated[1][0] = 5.0
        self.assertEqual(len(a.allocated_sizes), 2)
        self.assertEqual(a.value, 4.0)
        self.assertEqual(a.values[0], 4.0)
        self.assertEqual(a.values[1], 5.0)
        self.assertEqual(a.allocated_sizes[0], 8)
        self.assertEqual(a.allocated_sizes[1], 8)

        self.assertRaises(RuntimeError, a, 7)
        self.assertRaises(RuntimeError, a, 4)
        self.assertRaises(RuntimeError, a, 16)

    def test_ros_messages_allocator(self):
        a = RosMessagesAllocator(Header)
        self.assertEqual(0, len(a.allocated))
        self.assertEqual(0, len(a.allocated_sizes))
        self.assertIsNone(a.value)
        self.assertEqual(0, len(a.values))
        h = Header(seq=1, stamp=rospy.Time(2, 3), frame_id="cras")
        buf = BufferStringIO()
        h.serialize(buf)
        size = len(buf.getvalue())
        a(size)
        for i in range(size):
            val = buf.getvalue()[i]
            a.allocated[0][i] = ord(val) if not isinstance(val, int) else val
        self.assertEqual(len(a.allocated), 1)
        self.assertEqual(len(a.allocated_sizes), 1)
        self.assertEqual(len(list(a.messages)), 1)
        self.assertEqual(len(a.value), size)
        self.assertEqual(a.allocated_sizes[0], size)
        self.assertEqual(h, list(a.messages)[0])

        h2 = Header(seq=4, stamp=rospy.Time(5, 6), frame_id="test")
        buf2 = BufferStringIO()
        h2.serialize(buf2)
        size2 = len(buf2.getvalue())
        a(size2)
        for i in range(size2):
            val = buf2.getvalue()[i]
            a.allocated[1][i] = ord(val) if not isinstance(val, int) else val
        self.assertEqual(len(a.allocated), 2)
        self.assertEqual(len(a.allocated_sizes), 2)
        self.assertEqual(len(list(a.messages)), 2)
        self.assertEqual(len(a.value), size)
        self.assertEqual(len(a.values), 2)
        self.assertEqual(a.allocated_sizes[0], size)
        self.assertEqual(a.allocated_sizes[1], size2)
        self.assertEqual(2, len(list(a.messages)))
        self.assertEqual(h, list(a.messages)[0])
        self.assertEqual(h2, list(a.messages)[1])

    def test_log_messages_allocator(self):
        a = TestLogAllocator()
        self.assertEqual(0, len(a.allocated))
        self.assertEqual(0, len(a.allocated_sizes))
        self.assertIsNone(a.value)
        self.assertEqual(0, len(a.values))
        log = Log(header=Header(seq=1, stamp=rospy.Time(2, 3), frame_id="cras"), level=Log.WARN, name="log", msg="cras",
                  file="file", function="fun", line=4)
        buf = BufferStringIO()
        log.serialize(buf)
        size = len(buf.getvalue())
        a(size)
        for i in range(size):
            val = buf.getvalue()[i]
            a.allocated[0][i] = ord(val) if not isinstance(val, int) else val
        self.assertEqual(len(a.allocated), 1)
        self.assertEqual(len(a.allocated_sizes), 1)
        self.assertEqual(len(list(a.messages)), 1)
        self.assertEqual(len(a.value), size)
        self.assertEqual(a.allocated_sizes[0], size)
        self.assertEqual(log, list(a.messages)[0])
        self.assertEqual(Log.WARN, list(a.get_formatted_log_messages())[0][0])
        self.assertEqual(str(log), list(a.get_formatted_log_messages())[0][1])
        with RosconsoleCapture(self, Log.WARN, str(log)):
            a.print_log_messages()

        log2 = Log(header=Header(seq=5, stamp=rospy.Time(6, 7), frame_id="test"), level=Log.ERROR, name="logger",
                   msg="test", file="file2", function="fun2", line=8)
        buf2 = BufferStringIO()
        log2.serialize(buf2)
        size2 = len(buf2.getvalue())
        a(size2)
        for i in range(size2):
            val = buf2.getvalue()[i]
            a.allocated[1][i] = ord(val) if not isinstance(val, int) else val
        self.assertEqual(len(a.allocated), 2)
        self.assertEqual(len(a.allocated_sizes), 2)
        self.assertEqual(len(list(a.messages)), 2)
        self.assertEqual(len(a.value), size)
        self.assertEqual(len(a.values), 2)
        self.assertEqual(a.allocated_sizes[0], size)
        self.assertEqual(a.allocated_sizes[1], size2)
        self.assertEqual(2, len(list(a.messages)))
        self.assertEqual(log, list(a.messages)[0])
        self.assertEqual(log2, list(a.messages)[1])

        with RosconsoleCapture(self, (Log.WARN, Log.ERROR), (str(log), str(log2))):
            a.print_log_messages()

    def test_get_ro_c_buffer(self):
        buf = get_ro_c_buffer([1, 2])
        self.assertEqual(bytearray(buf), bytearray(b'\x01\x02'))

        buf = get_ro_c_buffer((1, 2))
        self.assertEqual(bytearray(buf), bytearray(b'\x01\x02'))

        buf = get_ro_c_buffer(b'\x01\x02')
        self.assertEqual(string_at(buf, 2), b'\x01\x02')

        buf = get_ro_c_buffer(BufferStringIO(b'\x01\x02'))
        self.assertEqual(string_at(buf, 2), b'\x01\x02')

    def test_c_array(self):
        py_arr = [1, 2]
        c_arr = c_array(py_arr, c_double)
        self.assertTrue(isinstance(c_arr, Array))
        self.assertEqual(len(c_arr), len(py_arr) + 1)
        self.assertEqual(c_arr[-1], 0)


if __name__ == '__main__':
    unittest.main()
