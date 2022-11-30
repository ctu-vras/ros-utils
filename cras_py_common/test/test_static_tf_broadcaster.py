#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.static_transform_broadcaster"""

import time
import unittest

import rospy
import rostest
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


class StaticTfBroadcasterTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(StaticTfBroadcasterTest, self).__init__(*args, **kwargs)
        rospy.init_node("test_static_tf_broadcaster")
        # first wait for at least one message on /tf_static
        rospy.wait_for_message("/tf_static", TFMessage, timeout=rospy.Duration(10))
        time.sleep(0.1)
        # only after that start the tf listener
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf)

    def test_receive_transforms(self):
        # We give a 10-second opportunity to read the composed transform.
        # Node publish_static_tfs publishes two static transforms (a->b, b->c) via cras.StaticTransformBroadcaster.
        # The normal tf2 broadcaster would fail here if the tf listener is started later than the second transform
        # is published, because the latched publisher would only tell the listener about the second transform.
        success = False
        error = ""
        for i in range(10):
            success, error = self.tf.can_transform("c", "a", rospy.Time(0), return_debug_tuple=True)
            if success:
                break
            else:
                time.sleep(1)
        self.assertTrue(success, "Transform a->c not found after 10 tries. Error was: " + error)


if __name__ == '__main__':
    rostest.rosrun("cras_py_common", "test_static_tf_broadcaster", StaticTfBroadcasterTest)
