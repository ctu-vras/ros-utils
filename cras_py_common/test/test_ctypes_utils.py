#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.ctypes_utils"""

from ctypes import c_double, string_at
import math
import unittest

from cras.ctypes_utils import load_library, StringAllocator, BytesAllocator, get_ro_c_buffer
from cras.string_utils import BufferStringIO


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
        self.assertIsNone(a.allocated)
        self.assertEqual(a.allocated_size, 0)
        self.assertIsNone(a.value)
        a(4)
        a.allocated[0] = 'a'.encode()
        a.allocated[1] = 'b'.encode()
        a.allocated[2] = 'c'.encode()
        a.allocated[3] = 'd'.encode()
        self.assertEqual(len(a.value), 4)
        self.assertEqual(a.allocated_size, 4)
        self.assertEqual(a.value, "abcd")

    def test_bytes_allocator(self):
        a = BytesAllocator()
        self.assertIsNone(a.allocated)
        self.assertEqual(a.allocated_size, 0)
        self.assertIsNone(a.value)
        a(4)
        a.allocated[0] = 0
        a.allocated[1] = 1
        a.allocated[2] = 2
        a.allocated[3] = 3
        self.assertEqual(len(a.value), 4)
        self.assertEqual(a.allocated_size, 4)
        self.assertEqual(a.value, b'\x00\x01\x02\x03')

    def test_get_ro_c_buffer(self):
        buf = get_ro_c_buffer([1, 2])
        self.assertEqual(bytearray(buf), bytearray(b'\x01\x02'))

        buf = get_ro_c_buffer((1, 2))
        self.assertEqual(bytearray(buf), bytearray(b'\x01\x02'))

        buf = get_ro_c_buffer(b'\x01\x02')
        self.assertEqual(string_at(buf, 2), b'\x01\x02')

        buf = get_ro_c_buffer(BufferStringIO(b'\x01\x02'))
        self.assertEqual(string_at(buf, 2), b'\x01\x02')


if __name__ == '__main__':
    unittest.main()
