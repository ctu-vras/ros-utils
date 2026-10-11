# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import rospy
from tf2_msgs.msg import TFMessage
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster as tfBroadcaster


class StaticTransformBroadcaster(tfBroadcaster):
    """
    :class:`StaticTransformBroadcaster`
    Allows broadcasting more than one static TF (accumulates all that have been
    sent to this class and publishes the aggregate message).
    """
    # Since only one latched publisher should exist per node, this is a class object
    pub_tf = rospy.Publisher("/tf_static", TFMessage, queue_size=100, latch=True)
    tfs = dict()

    def __init__(self):
        # explicitly do not call parent constructor - it only creates the publisher,
        # but we already have the static one
        # super(StaticTransformBroadcaster, self).__init__()
        self.pub_tf = StaticTransformBroadcaster.pub_tf

    def sendTransform(self, transform):
        if not isinstance(transform, list):
            transform = [transform]
        for tf in transform:
            StaticTransformBroadcaster.tfs[(tf.header.frame_id, tf.child_frame_id)] = tf

        super(StaticTransformBroadcaster, self).sendTransform(list(StaticTransformBroadcaster.tfs.values()))
