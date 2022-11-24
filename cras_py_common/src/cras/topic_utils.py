# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when subscribing or publishing topics."""

import rospy

from .type_utils import get_msg_type


class GenericMessageSubscriber(object):
    """Drop-in replacement for rospy.Subscriber that subscribes to any type message, automatically deserializes it
    and calls the callback with the deserialized message instance."""

    def __init__(self, topic_name, callback, *args, **kwargs):
        """Create the subscriber.

        :param str topic_name: Name of the topic to subscribe.
        :param Callable[[Any],None] callback: The callback expecting the deserialized message.
        :param List[Any] args: rospy.Subscriber further arguments.
        :param Dict[str, Any] kwargs: rospy.Subscriber further arguments.
        """
        self._sub = rospy.Subscriber(topic_name, rospy.AnyMsg, self._raw_cb, *args, **kwargs)
        self._callback = callback
        self._msg_class = None

    def _raw_cb(self, msg):
        """Callback receiving the raw (serialized) messages, deserializing them and calling the user callback.

        :param rospy.AnyMsg msg: The raw message.
        """
        if self._msg_class is None:
            # noinspection PyProtectedMember
            self._msg_class = get_msg_type(msg._connection_header['type'])
        # noinspection PyProtectedMember
        msg = self._msg_class().deserialize(msg._buff)
        self._callback(msg)
