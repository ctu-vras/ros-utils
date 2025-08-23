# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Filter a bag file using a MessageFilter."""

import copy
import sys

import rosbag.bag

from .bag_utils import MultiBag
from .message_filter import MessageFilter, Passthrough, filter_message

if sys.version[0] == '2':
    from Queue import Queue  # noqa
else:
    from queue import Queue  # noqa


def filter_bag(bags, out, bag_filter=Passthrough(), params=None):
    """Filter the given bagfile 'bag' into bagfile 'out' using message filter 'filter'.

    :param bags: The input bag (open in read mode).
    :type bags: rosbag.bag.Bag or MultiBag
    :param rosbag.bag.Bag out: The output bag (open in write mode).
    :param MessageFilter bag_filter: The filter to apply.
    :param dict params: Loaded ROS parameters.
    """
    bag_filter.set_params(params)

    if isinstance(bags, MultiBag):
        bag = bags.bags[0]
    else:
        bag = bags

    bag_filter.set_bag(bag)

    # get all topics
    topics = [c.topic for c in bags._get_connections()]  # noqa
    # apply topic filters
    topics = [t for t in topics if bag_filter.topic_filter(t)]
    # apply connection filters
    topics = [c.topic for c in bags._get_connections(topics, bag_filter.connection_filter)]  # noqa

    queue = Queue()
    connection_filter = bag_filter.connection_filter
    for topic, msg, stamp, connection_header in bags.read_messages(
            topics=topics, return_connection_header=True, raw=bag_filter.is_raw, connection_filter=connection_filter):
        queue.put((topic, msg, stamp, connection_header))
        while not queue.empty():
            _topic, _msg, _stamp, _connection_header = queue.get()
            _connection_header = copy.copy(_connection_header)  # Prevent modifying connection records from in bag
            ret = filter_message(_topic, _msg, _stamp, _connection_header, bag_filter, True)
            if not isinstance(ret, list):
                ret = [ret]
            for data in ret[1:]:
                queue.put(data)
            if ret[0] is None:
                continue
            _topic, _raw_msg, _stamp, _connection_header = ret[0]
            out.write(_topic, _raw_msg, _stamp, connection_header=_connection_header, raw=True)

    bag_filter.reset()
