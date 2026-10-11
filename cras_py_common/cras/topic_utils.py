# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Utilities helpful when subscribing or publishing topics."""

import rospy

from .message_utils import get_msg_type


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

    def _raw_cb(self, raw_msg):
        """Callback receiving the raw (serialized) messages, deserializing them and calling the user callback.

        :param rospy.AnyMsg raw_msg: The raw message.
        """
        if self._msg_class is None:
            # noinspection PyProtectedMember
            self._msg_class = get_msg_type(raw_msg._connection_header['type'])
        # noinspection PyProtectedMember
        msg = self._msg_class().deserialize(raw_msg._buff)
        msg._connection_header = raw_msg._connection_header
        self._callback(msg)

    # allow calling unregister(), get_num_subscribers(), .name etc.
    def __getattr__(self, item):
        if self._msg_class is not None:
            if item == "data_class":
                return self._msg_class
            elif item == "type":
                # noinspection PyProtectedMember
                return self._msg_class._type
            elif item == "md5sum":
                # noinspection PyProtectedMember
                return self._msg_class._md5sum
        return getattr(self._sub, item)
