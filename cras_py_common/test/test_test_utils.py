#!/usr/bin/env python

"""Unit test for cras.test_utils"""

import unittest

import rospy

from rosgraph_msgs.msg import Log

from cras.test_utils import RosconsoleCapture


class MockTest:
    def __init__(self):
        self.failed = False

    def fail(self, _=None):
        self.failed = True


class TestUtils(unittest.TestCase):

    def test_rosconsole_capture(self):
        with RosconsoleCapture(self):
            pass

        with RosconsoleCapture(self, Log.DEBUG, "Debug test"):
            rospy.logdebug("%s %s", "Debug", "test")

        with RosconsoleCapture(self, Log.INFO, "Info test"):
            rospy.loginfo("%s %s", "Info", "test")

        with RosconsoleCapture(self, Log.WARN, "Warn test"):
            rospy.logwarn("%s %s", "Warn", "test")

        with RosconsoleCapture(self, Log.ERROR, "Error test"):
            rospy.logerr("%s %s", "Error", "test")

        with RosconsoleCapture(self, Log.FATAL, "Fatal test"):
            rospy.logfatal("%s %s", "Fatal", "test")

        # multiple messages
        with RosconsoleCapture(self, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.logdebug("Debug test")
            rospy.loginfo("Info test")

        # different number of log levels and messages is an error
        self.assertRaises(RuntimeError, RosconsoleCapture, self, (Log.DEBUG, Log.DEBUG), "Message")

        mock_test = MockTest()

        # wrong log message level
        mock_test.failed = False
        with RosconsoleCapture(mock_test, Log.DEBUG, "Debug test"):
            rospy.loginfo("%s %s", "Debug", "test")
        self.assertTrue(mock_test.failed)

        # wrong log message with correct level
        mock_test.failed = False
        with RosconsoleCapture(mock_test, Log.DEBUG, "Debug test"):
            rospy.logdebug("%s %s", "Info", "test")
        self.assertTrue(mock_test.failed)

        # nothing logged
        mock_test.failed = False
        with RosconsoleCapture(mock_test, Log.DEBUG, "Debug test"):
            pass
        self.assertTrue(mock_test.failed)

        # message logged but nothing was expected
        mock_test.failed = False
        with RosconsoleCapture(mock_test):
            rospy.loginfo("%s %s", "Info", "test")
        self.assertTrue(mock_test.failed)

        # log more messages than expected
        mock_test.failed = False
        with RosconsoleCapture(mock_test, Log.DEBUG, "Debug test"):
            rospy.loginfo("Debug")
            rospy.logdebug("test")
        self.assertTrue(mock_test.failed)

        # log fewer messages than expected
        mock_test.failed = False
        with RosconsoleCapture(mock_test, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.loginfo("Debug test")
        self.assertTrue(mock_test.failed)

        # log wrong order of messages
        mock_test.failed = False
        with RosconsoleCapture(mock_test, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.loginfo("Info test")
            rospy.logdebug("Debug test")
        self.assertTrue(mock_test.failed)

        # log wrong messages
        mock_test.failed = False
        with RosconsoleCapture(mock_test, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.logdebug("Debug test fail")
            rospy.loginfo("Info test")
        self.assertTrue(mock_test.failed)

        # log wrong messages
        mock_test.failed = False
        with RosconsoleCapture(mock_test, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.logdebug("Debug test")
            rospy.loginfo("Info test fail")
        self.assertTrue(mock_test.failed)

        # log correct messages with wrong levels
        mock_test.failed = False
        with RosconsoleCapture(mock_test, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.loginfo("Debug test")
            rospy.loginfo("Info test fail")
        self.assertTrue(mock_test.failed)

        # log correct messages with wrong levels
        mock_test.failed = False
        with RosconsoleCapture(mock_test, (Log.DEBUG, Log.INFO), ("Debug test", "Info test")):
            rospy.logdebug("Debug test")
            rospy.logdebug("Info test fail")
        self.assertTrue(mock_test.failed)


if __name__ == '__main__':
    unittest.main()
