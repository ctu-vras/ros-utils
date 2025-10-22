#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.heap"""

import unittest

from cras.heap import Heap


class HeapTest(unittest.TestCase):

    def test_heap(self):
        h = Heap(key=lambda x: x[1])

        self.assertEqual(0, len(h))
        self.assertRaises(IndexError, Heap.pop, h)
        self.assertRaises(IndexError, Heap.peek, h)

        h.push(("test", 1))
        self.assertEqual(1, len(h))
        self.assertEqual(("test", 1), h.peek())
        self.assertEqual(("test", 1), h.pop())
        self.assertEqual(0, len(h))

        h.push(("test", 1))
        h.push(("test2", 0))
        self.assertEqual(2, len(h))
        self.assertEqual(("test2", 0), h.peek())
        self.assertEqual(("test2", 0), h.pop())
        self.assertEqual(1, len(h))
        self.assertEqual(("test", 1), h.pop())
        self.assertEqual(0, len(h))

        # Test that the sort is stable (values with the same keys are ordered as they were inserted).
        h.push(("test", 0))
        h.push(("test2", 0))
        h.push(("test3", 0))
        self.assertEqual(3, len(h))
        self.assertEqual(("test", 0), h.pop())
        self.assertEqual(("test2", 0), h.pop())
        self.assertEqual(("test3", 0), h.pop())
        self.assertEqual(0, len(h))


if __name__ == '__main__':
    unittest.main()
