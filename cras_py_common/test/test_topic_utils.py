#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for topic_utils."""

import time

import rospy
import rostest
import unittest

from std_msgs.msg import String

import cras


class TopicUtils(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TopicUtils, self).__init__(*args, **kwargs)
        rospy.init_node("topic_utils_test")

    def cb_generic_sub(self, msg):
        self._last_generic_msg = msg

    def test_generic_subscriber(self):
        self._last_generic_msg = None
        sub = cras.GenericMessageSubscriber("test", self.cb_generic_sub, queue_size=1)
        for i in range(100):
            time.sleep(0.01)
            if rospy.is_shutdown():
                return
            if self._last_generic_msg is not None:
                break
        self.assertIsNotNone(self._last_generic_msg)

        msg = self._last_generic_msg
        self.assertIsInstance(msg, String)
        self.assertEqual(msg.data, "test")


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_topic_utils", TopicUtils)
    unittest.main()
