# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Filter a bag file using a MessageFilter."""

from queue import Queue

import rosbag.bag

from .message_filter import MessageFilter, Passthrough, filter_message


def filter_bag(bag, out, filter=Passthrough()):
    """Filter the given bagfile 'bag' into bagfile 'out' using message filter 'filter'.

    :param rosbag.bag.Bag bag: The input bag (open in read mode).
    :param rosbag.bag.Bag out: The output bag (open in write mode).
    :param MessageFilter filter: The filter to apply.
    """
    filter.set_bag(bag)

    topics = [c.topic for c in bag._get_connections()]  # get all topics
    topics = [t for t in topics if filter.topic_filter(t)]  # apply topic filters
    topics = [c.topic for c in bag._get_connections(topics, filter.connection_filter)]  # apply connection filters

    queue = Queue()
    connection_filter = filter.connection_filter
    for topic, msg, stamp, connection_header in bag.read_messages(
            topics=topics, return_connection_header=True, raw=filter.is_raw, connection_filter=connection_filter):
        queue.put((topic, msg, stamp, connection_header))
        while not queue.empty():
            _topic, _msg, _stamp, _connection_header = queue.get()
            ret = filter_message(_topic, _msg, _stamp, _connection_header, filter, True)
            if not isinstance(ret, list):
                ret = [ret]
            for data in ret[1:]:
                queue.put(data)
            if ret[0] is None:
                continue
            _topic, _raw_msg, _stamp, _connection_header = ret[0]
            out.write(_topic, _raw_msg, _stamp, connection_header=_connection_header, raw=True)

    filter.reset()
