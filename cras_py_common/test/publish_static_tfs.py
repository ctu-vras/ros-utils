#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Publish two static tfs using cras.StaticTransformBroadcaster to test if both are available for clients."""

import rospy
from geometry_msgs.msg import TransformStamped

from cras.static_transform_broadcaster import StaticTransformBroadcaster

rospy.init_node("publish_static_tfs")

b = StaticTransformBroadcaster()

tf1 = TransformStamped()
tf1.header.stamp = rospy.Time.now()
tf1.header.frame_id = "a"
tf1.child_frame_id = "b"
tf1.transform.rotation.w = 1
b.sendTransform(tf1)

tf2 = TransformStamped()
tf2.header.stamp = rospy.Time.now()
tf2.header.frame_id = "b"
tf2.child_frame_id = "c"
tf2.transform.rotation.w = 1
b.sendTransform(tf2)

rospy.spin()
