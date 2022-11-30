#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for node_utils."""
import time

import rospy
import rostest
import unittest

import cras


class TestNode(cras.Node):

    def __init__(self):
        super(TestNode, self).__init__()
        self.num_reset_calls = 0

    def reset(self):
        self.num_reset_calls += 1
        super(TestNode, self).reset()


class NodeUtils(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(NodeUtils, self).__init__(*args, **kwargs)
        rospy.init_node("node_utils_test")

    def test_reset(self):
        node = TestNode()

        self.assertEqual(0, node.num_reset_calls)
        node.reset()
        self.assertEqual(1, node.num_reset_calls)

        node.check_time_jump(rospy.Time(10))
        self.assertEqual(1, node.num_reset_calls)
        node.check_time_jump(rospy.Time(11))
        self.assertEqual(1, node.num_reset_calls)
        node.check_time_jump(rospy.Time(12))
        self.assertEqual(1, node.num_reset_calls)
        # by default, in simtime the jump back threshold is 3 seconds
        node.check_time_jump(rospy.Time(9))
        self.assertEqual(2, node.num_reset_calls)
        node.check_time_jump(rospy.Time(10))
        self.assertEqual(2, node.num_reset_calls)
        # by default, in simtime the jump forward threshold is 10 seconds
        node.check_time_jump(rospy.Time(21))
        self.assertEqual(3, node.num_reset_calls)

        node.check_time_jump(rospy.Time(10))
        rospy.rostime._set_rostime(rospy.Time(10))

        node.start_auto_check_time_jump(10)
        self.assertEqual(4, node.num_reset_calls)
        rospy.rostime._set_rostime(rospy.Time(10.1))
        self.assertEqual(4, node.num_reset_calls)
        time.sleep(0.1)
        self.assertEqual(4, node.num_reset_calls)
        rospy.rostime._set_rostime(rospy.Time(10.2))
        self.assertEqual(4, node.num_reset_calls)
        time.sleep(0.1)
        self.assertEqual(4, node.num_reset_calls)
        rospy.rostime._set_rostime(rospy.Time(7))
        self.assertEqual(4, node.num_reset_calls)
        time.sleep(0.5)
        self.assertEqual(5, node.num_reset_calls)


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_node_utils", NodeUtils)
