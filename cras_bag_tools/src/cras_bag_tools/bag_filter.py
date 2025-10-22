# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Filter a bag file using a MessageFilter."""

import copy
import sys

import genpy
import rosbag.bag
import rospy
from cras import Heap

from .bag_utils import BagWrapper, MultiBag
from .message_filter import MessageFilter, Passthrough, filter_message
from .time_range import TimeRange, TimeRanges


def filter_bag(bags, out, bag_filter=Passthrough(), params=None, start_time=None, end_time=None, time_ranges=None):
    """Filter the given bagfile 'bag' into bagfile 'out' using message filter 'filter'.

    :param bags: The input bag (open in read mode).
    :type bags: rosbag.bag.Bag or MultiBag
    :param rosbag.bag.Bag out: The output bag (open in write mode).
    :param MessageFilter bag_filter: The filter to apply.
    :param dict params: Loaded ROS parameters.
    :param genpy.Time start_time: Time from which the bag filtering should be started.
    :param genpy.Time end_time: Time to which the bag filtering should be stopped.
    :param TimeRanges time_ranges: Time ranges of the bag file to process. If start_time and end_time are specified,
                                   they are merged with these ranges. Relative time ranges will be evaluated relative
                                   to each individual bag.
    """
    bag_filter.set_params(params)

    if isinstance(bags, MultiBag):
        bag = bags.bags[0]
    elif not isinstance(bags, BagWrapper):
        bag = bags = BagWrapper(bags)  # support passing TimeRanges as start_time
    else:
        bag = bags

    bag_filter.set_bag(bag)

    if start_time is not None or end_time is not None:
        if time_ranges is None:
            time_ranges = TimeRanges([])
        if start_time is None:
            start_time = genpy.Time(bags.get_start_time())
        if end_time is None:
            end_time = genpy.Time(bags.get_end_time())
        time_ranges.append(TimeRange(start_time, end_time))

    if time_ranges is not None:
        time_ranges.set_base_time(genpy.Time(bags.get_start_time()))

    extra_time_ranges = None
    # If time_ranges is None, we read the whole bags, so we don't need any extra time ranges
    if time_ranges is not None:
        extra_time_ranges = bag_filter.extra_time_ranges(bags)
        if extra_time_ranges is not None:
            extra_time_ranges.set_base_time(genpy.Time(bags.get_start_time()))

    # get all topics
    topics = [c.topic for c in bags._get_connections()]  # noqa
    # apply topic filters
    topics = [t for t in topics if bag_filter.topic_filter(t)]
    # apply connection filters
    topics = [c.topic for c in bags._get_connections(topics, bag_filter.connection_filter)]  # noqa

    bag_filter.on_filtering_start()

    # get stamp from the tuple
    def get_stamp_fn(x):
        return x[2]
    heap = Heap(initial=list(bag_filter.extra_initial_messages()), key=get_stamp_fn)

    connection_filter = bag_filter.connection_filter
    _stamp = rospy.Time(0)
    for topic, msg, stamp, connection_header in bags.read_messages(
            topics=topics, start_time=time_ranges, end_time=extra_time_ranges, return_connection_header=True,
            raw=bag_filter.is_raw, connection_filter=connection_filter):
        heap.push((topic, msg, stamp, connection_header))
        # For each bag message, process the whole heap up to the stamp of the bag message
        while len(heap) > 0 and _stamp <= stamp:
            _topic, _msg, _stamp, _connection_header = heap.pop()
            _connection_header = copy.copy(_connection_header)  # Prevent modifying connection records from in bag
            is_from_extra_time_ranges = False
            if extra_time_ranges is not None and time_ranges is not None:
                is_from_extra_time_ranges = _stamp in extra_time_ranges and _stamp not in time_ranges
            ret = filter_message(_topic, _msg, _stamp, _connection_header, bag_filter, True, is_from_extra_time_ranges)
            if not isinstance(ret, list):
                ret = [ret]
            for data in ret[1:]:
                heap.push(data)
            if ret[0] is None:
                continue
            _topic, _raw_msg, _stamp, _connection_header = ret[0]
            out.write(_topic, _raw_msg, _stamp, connection_header=_connection_header, raw=True)

    # Push all final messages before processing the rest of the heap
    for m in bag_filter.extra_final_messages():
        heap.push(m)

    # Finish the rest of the heap if there is something left (messages with stamp higher than last message from bag)
    while len(heap) > 0:
        _topic, _msg, _stamp, _connection_header = heap.pop()
        _connection_header = copy.copy(_connection_header)  # Prevent modifying connection records from in bag
        is_from_extra_time_ranges = False
        if extra_time_ranges is not None and time_ranges is not None:
            is_from_extra_time_ranges = _stamp in extra_time_ranges and _stamp not in time_ranges
        ret = filter_message(_topic, _msg, _stamp, _connection_header, bag_filter, True, is_from_extra_time_ranges)
        if not isinstance(ret, list):
            ret = [ret]
        for data in ret[1:]:
            heap.push(data)
        if ret[0] is None:
            continue
        _topic, _raw_msg, _stamp, _connection_header = ret[0]
        out.write(_topic, _raw_msg, _stamp, connection_header=_connection_header, raw=True)

    bag_filter.on_filtering_end()

    bag_filter.reset()
