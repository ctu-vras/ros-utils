#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for type_utils."""

import unittest

from cras import get_msg_type


class TypeUtils(unittest.TestCase):

    def test_get_msg_type(self):
        cls = get_msg_type("std_msgs/String")
        self.assertEqual(cls._type, "std_msgs/String")
        self.assertTrue(hasattr(cls, "data"))

        msg = cls()
        from std_msgs.msg import String
        self.assertIsInstance(msg, String)


if __name__ == '__main__':
    unittest.main()
