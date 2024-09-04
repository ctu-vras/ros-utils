#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for python_utils."""

import locale
import os
import rostest
import unittest

from cras.ctypes_utils import libc_getenv, libc_setenv, libc_unsetenv
from cras.python_utils import temp_environ, temp_locale


class PythonUtils(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(PythonUtils, self).__init__(*args, **kwargs)

    def test_temp_environ(self):
        self.assertIsNone(os.environ.get("TEST", None))
        self.assertIsNone(os.environ.get("TEST6", None))
        os.environ["TEST2"] = "Test2"
        os.environ["TEST4"] = "Test4"
        libc_setenv("TEST3", "Test3")
        libc_setenv("TEST5", "Test5")
        with temp_environ(unset=["TEST2", "TEST5"], TEST="Test", TEST6="Test6"):
            self.assertEqual(os.environ.get("TEST", None), "Test")
            self.assertEqual(os.environ.get("TEST6", None), "Test6")
            self.assertEqual(os.getenv("TEST"), "Test")
            self.assertEqual(os.environ.get("TEST4", None), "Test4")
            self.assertEqual(libc_getenv("TEST3", None), "Test3")
            self.assertIsNone(os.environ.get("TEST2", None))
            self.assertIsNone(libc_getenv("TEST5", None))
        self.assertIsNone(os.environ.get("TEST", None))
        self.assertIsNone(os.environ.get("TEST6", None))
        self.assertEqual(os.environ.get("TEST2", None), "Test2")
        self.assertEqual(os.environ.get("TEST4", None), "Test4")
        self.assertEqual(libc_getenv("TEST3", None), "Test3")

    def test_temp_locale(self):
        locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
        self.assertEqual("1000.00", locale.format_string("%3.2f", 1000.0, monetary=True))
        with temp_locale(locale.LC_ALL, 'C'):
            self.assertEqual("100000", locale.format_string("%3.2f", 1000.0, monetary=True))
        self.assertEqual("1000.00", locale.format_string("%3.2f", 1000.0, monetary=True))

        # unfortunately, the tests cannot rely on any other specific locales to be installed


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_python_utils", PythonUtils)
