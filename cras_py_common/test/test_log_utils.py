#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.log_utils"""

import time
import unittest

import rospy
import rostest
from rosgraph_msgs.msg import Log

import cras.log_utils
from cras.test_utils import RosconsoleCapture


class LogUtils(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(LogUtils, self).__init__(*args, **kwargs)
        rospy.init_node("test_log_utils", anonymous=True, log_level=rospy.DEBUG)

    def test_log(self):
        with RosconsoleCapture(self):
            self.assertRaises(KeyError, cras.log, Log.FATAL+1, "Test")

        with RosconsoleCapture(self, Log.DEBUG, "Debug test"):
            cras.log(Log.DEBUG, "%s %s", "Debug", "test")

        with RosconsoleCapture(self, Log.INFO, "Info test"):
            cras.log(Log.INFO, "%s %s", "Info", "test")

        with RosconsoleCapture(self, Log.WARN, "Warn test"):
            cras.log(Log.WARN, "%s %s", "Warn", "test")

        with RosconsoleCapture(self, Log.ERROR, "Error test"):
            cras.log(Log.ERROR, "%s %s", "Error", "test")

        with RosconsoleCapture(self, Log.FATAL, "Fatal test"):
            cras.log(Log.FATAL, "%s %s", "Fatal", "test")

    def test_log_once(self):
        with RosconsoleCapture(self):
            self.assertRaises(KeyError, cras.log_once, Log.FATAL+1, "Test")

        with RosconsoleCapture(self, Log.DEBUG, "Debug test"):
            for _ in range(10):
                cras.log_once(Log.DEBUG, "%s %s", "Debug", "test")

        with RosconsoleCapture(self, Log.INFO, "Info test"):
            for _ in range(10):
                cras.log_once(Log.INFO, "%s %s", "Info", "test")

        with RosconsoleCapture(self, Log.WARN, "Warn test"):
            for _ in range(10):
                cras.log_once(Log.WARN, "%s %s", "Warn", "test")

        with RosconsoleCapture(self, Log.ERROR, "Error test"):
            for _ in range(10):
                cras.log_once(Log.ERROR, "%s %s", "Error", "test")

        with RosconsoleCapture(self, Log.FATAL, "Fatal test"):
            for _ in range(10):
                cras.log_once(Log.FATAL, "%s %s", "Fatal", "test")

    def test_log_once_identical(self):
        with RosconsoleCapture(self):
            self.assertRaises(KeyError, cras.log_once_identical, Log.FATAL+1, "Test")

        with RosconsoleCapture(self, (Log.DEBUG,)*2, ("Debug identical test", "Debug identical test 2")):
            for _ in range(10):
                cras.log_once_identical(Log.DEBUG, "%s %s", "Debug identical", "test")
            for _ in range(10):
                cras.log_once_identical(Log.DEBUG, "%s %s %i", "Debug identical", "test", 2)

        with RosconsoleCapture(self, (Log.INFO,)*2, ("Info identical test", "Info identical test 2")):
            for _ in range(10):
                cras.log_once_identical(Log.INFO, "%s %s", "Info identical", "test")
            for _ in range(10):
                cras.log_once_identical(Log.INFO, "%s %s %i", "Info identical", "test", 2)

        with RosconsoleCapture(self, (Log.WARN,)*2, ("Warn identical test", "Warn identical test 2")):
            for _ in range(10):
                cras.log_once_identical(Log.WARN, "%s %s", "Warn identical", "test")
            for _ in range(10):
                cras.log_once_identical(Log.WARN, "%s %s %i", "Warn identical", "test", 2)

        with RosconsoleCapture(self, (Log.ERROR,)*2, ("Error test", "Error test 2")):
            for _ in range(10):
                cras.log_once_identical(Log.ERROR, "%s %s", "Error", "test")
            for _ in range(10):
                cras.log_once_identical(Log.ERROR, "%s %s %i", "Error", "test", 2)

        with RosconsoleCapture(self, (Log.FATAL,)*2, ("Fatal test", "Fatal test 2")):
            for _ in range(10):
                cras.log_once_identical(Log.FATAL, "%s %s", "Fatal", "test")
            for _ in range(10):
                cras.log_once_identical(Log.FATAL, "%s %s %i", "Fatal", "test", 2)

        # Now the direct functions

        with RosconsoleCapture(self, (Log.DEBUG, Log.DEBUG), ("Debug test", "Debug test 2")):
            for _ in range(10):
                cras.logdebug_once_identical("%s %s", "Debug", "test")
            for _ in range(10):
                cras.logdebug_once_identical("%s %s %i", "Debug", "test", 2)

        with RosconsoleCapture(self, (Log.INFO, Log.INFO), ("Info test", "Info test 2")):
            for _ in range(10):
                cras.loginfo_once_identical("%s %s", "Info", "test")
            for _ in range(10):
                cras.loginfo_once_identical("%s %s %i", "Info", "test", 2)

        with RosconsoleCapture(self, (Log.WARN, Log.WARN), ("Warn test", "Warn test 2")):
            for _ in range(10):
                cras.logwarn_once_identical("%s %s", "Warn", "test")
            for _ in range(10):
                cras.logwarn_once_identical("%s %s %i", "Warn", "test", 2)

        with RosconsoleCapture(self, (Log.ERROR, Log.ERROR), ("Error test", "Error test 2")):
            for _ in range(10):
                cras.logerr_once_identical("%s %s", "Error", "test")
            for _ in range(10):
                cras.logerr_once_identical("%s %s %i", "Error", "test", 2)

        with RosconsoleCapture(self, (Log.FATAL, Log.FATAL), ("Fatal test", "Fatal test 2")):
            for _ in range(10):
                cras.logfatal_once_identical("%s %s", "Fatal", "test")
            for _ in range(10):
                cras.logfatal_once_identical("%s %s %i", "Fatal", "test", 2)

    def test_log_throttle(self):
        period = 0.1
        sleep_times = (0.01, 0.1, 0.01, 0.01)

        with RosconsoleCapture(self):
            self.assertRaises(KeyError, cras.log_throttle, Log.FATAL+1, period, "Test")

        with RosconsoleCapture(self, (Log.DEBUG,)*2, ("Debug test 0", "Debug test 2")):
            for i in range(len(sleep_times)):
                cras.log_throttle(Log.DEBUG, period, "%s %s %i", "Debug", "test", i)
                time.sleep(sleep_times[i])

        with RosconsoleCapture(self, (Log.INFO,)*2, ("Info test 0", "Info test 2")):
            for i in range(len(sleep_times)):
                cras.log_throttle(Log.INFO, period, "%s %s %i", "Info", "test", i)
                time.sleep(sleep_times[i])

        with RosconsoleCapture(self, (Log.WARN,)*2, ("Warn test 0", "Warn test 2")):
            for i in range(len(sleep_times)):
                cras.log_throttle(Log.WARN, period, "%s %s %i", "Warn", "test", i)
                time.sleep(sleep_times[i])

        with RosconsoleCapture(self, (Log.ERROR,)*2, ("Error test 0", "Error test 2")):
            for i in range(len(sleep_times)):
                cras.log_throttle(Log.ERROR, period, "%s %s %i", "Error", "test", i)
                time.sleep(sleep_times[i])

        with RosconsoleCapture(self, (Log.FATAL,)*2, ("Fatal test 0", "Fatal test 2")):
            for i in range(len(sleep_times)):
                cras.log_throttle(Log.FATAL, period, "%s %s %i", "Fatal", "test", i)
                time.sleep(sleep_times[i])

    def test_log_throttle_identical(self):
        period = 0.1
        sleep_times = (0.01, 0.1, 0.01, 0.01)

        with RosconsoleCapture(self):
            self.assertRaises(KeyError, cras.log_throttle_identical, Log.FATAL+1, period, "Test")

        # In this test, only the message format (i.e. first string arg to the logging call) is taken into account for
        # comparing identical messages. That is why there is different desired behavior between calling with a format +
        # args or with an already formatted string.

        with RosconsoleCapture(self, (Log.DEBUG,)*2, ("Debug test", "Debug test")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.DEBUG, period, "%s %s", "Debug", "test")
                time.sleep(sleep_times[i])
        with RosconsoleCapture(self, Log.DEBUG, "Debug test 0"):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.DEBUG, period, "%s %s %i", "Debug", "test", i)
                time.sleep(sleep_times[0])
        with RosconsoleCapture(self, (Log.DEBUG,)*4, ("Debug test 0", "Debug test 1", "Debug test 2", "Debug test 3")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.DEBUG, period, "%s %s %i" % ("Debug", "test", i))
                time.sleep(sleep_times[0])

        with RosconsoleCapture(self, (Log.INFO,)*2, ("Info test", "Info test")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.INFO, period, "%s %s", "Info", "test")
                time.sleep(sleep_times[i])
        with RosconsoleCapture(self, Log.INFO, "Info test 0"):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.INFO, period, "%s %s %i", "Info", "test", i)
                time.sleep(sleep_times[0])
        with RosconsoleCapture(self, (Log.INFO,)*4, ("Info test 0", "Info test 1", "Info test 2", "Info test 3")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.INFO, period, "%s %s %i" % ("Info", "test", i))
                time.sleep(sleep_times[0])

        with RosconsoleCapture(self, (Log.WARN,)*2, ("Warn test", "Warn test")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.WARN, period, "%s %s", "Warn", "test")
                time.sleep(sleep_times[i])
        with RosconsoleCapture(self, Log.WARN, "Warn test 0"):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.WARN, period, "%s %s %i", "Warn", "test", i)
                time.sleep(sleep_times[0])
        with RosconsoleCapture(self, (Log.WARN,)*4, ("Warn test 0", "Warn test 1", "Warn test 2", "Warn test 3")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.WARN, period, "%s %s %i" % ("Warn", "test", i))
                time.sleep(sleep_times[0])

        with RosconsoleCapture(self, (Log.ERROR,)*2, ("Error test", "Error test")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.ERROR, period, "%s %s", "Error", "test")
                time.sleep(sleep_times[i])
        with RosconsoleCapture(self, Log.ERROR, "Error test 0"):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.ERROR, period, "%s %s %i", "Error", "test", i)
                time.sleep(sleep_times[0])
        with RosconsoleCapture(self, (Log.ERROR,)*4, ("Error test 0", "Error test 1", "Error test 2", "Error test 3")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.ERROR, period, "%s %s %i" % ("Error", "test", i))
                time.sleep(sleep_times[0])

        with RosconsoleCapture(self, (Log.FATAL,)*2, ("Fatal test", "Fatal test")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.FATAL, period, "%s %s", "Fatal", "test")
                time.sleep(sleep_times[i])
        with RosconsoleCapture(self, Log.FATAL, "Fatal test 0"):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.FATAL, period, "%s %s %i", "Fatal", "test", i)
                time.sleep(sleep_times[0])
        with RosconsoleCapture(self, (Log.FATAL,)*4, ("Fatal test 0", "Fatal test 1", "Fatal test 2", "Fatal test 3")):
            for i in range(len(sleep_times)):
                cras.log_throttle_identical(Log.FATAL, period, "%s %s %i" % ("Fatal", "test", i))
                time.sleep(sleep_times[0])


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_log_utils", LogUtils)
