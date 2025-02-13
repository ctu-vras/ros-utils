# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A message filter that can decide whether a message should be kept or not, and possibly alter it."""

from __future__ import absolute_import, division, print_function

import sys
from typing import List, Optional, Tuple, Union

import genpy
import rospy
from cras.message_utils import raw_to_msg, msg_to_raw
from cras.plugin_utils import get_plugin_implementations

from .topic_set import TopicSet


def is_sequence(o):
    return isinstance(o, (list, tuple))


loaded_filters = None


def get_filters():
    """Get all defined message filters.

    :return: The list of message filters.
    :rtype: list of MessageFilter
    """
    global loaded_filters
    if loaded_filters is not None:
        return loaded_filters

    loaded_filters = {}
    for module_name, name, cls in get_plugin_implementations("cras_bag_tools", "filters", MessageFilter):
        loaded_filters[name] = cls
        loaded_filters["%s.%s" % (module_name, name)] = cls  # Also add a fully qualified name of the filter
    return loaded_filters


RawMessageData = Tuple[str, str, bytes, str, type, rospy.Time, dict]
DeserializedMessageData = Tuple[str, genpy.Message, rospy.Time, dict]
RawFilterResult = Union[None, RawMessageData, List[RawMessageData]]
DeserializedFilterResult = Union[None, DeserializedMessageData, List[DeserializedMessageData]]


class MessageFilter(object):
    """
    Base class for message filters. Do not implement this directly: instead implement either RawMessageFilter or
    DeserializeMessageFilter.

    The workflow of the filters is as follows:
    1. Get the topic filter and connection filter from the message filter and apply these to a bag reader (or other
       message stream provider, as this library is not bound to only processing bag messages).
    2. Read messages that satisfy the topic and connections filters.
    3. If the message does not satisfy consider_message(), it is passed further without a change. This is a kind of
       pre-filter that allows us to not deserialize messages just to tell to throw them away or pass them along.
    4. filter_message() is called. If it returns None, the message should be discarded. Otherwise, it either returns
       one message or a list of messages. The first (or only) message is considered to be the "direct followup" of the
       input message and continues going through the filter (or stops the filter if it is None). The other messages
       in the returned list should be fed into this filter again as new input messages.
    """

    def __init__(self, is_raw, include_topics=None, exclude_topics=None, include_types=None, exclude_types=None,
                 min_stamp=None, max_stamp=None):
        """Constructor.

        :param bool is_raw: Whether the filter works on raw or deserialized messages.
        :param list include_topics: If nonempty, the filter will only work on these topics.
        :param list exclude_topics: If nonempty, the filter will skip these topics (but pass them further).
        :param list include_types: If nonempty, the filter will only work on these message types.
        :param list exclude_types: If nonempty, the filter will skip these message types (but pass them further).
        :param rospy.Time min_stamp: If set, the filter will only work on messages after this timestamp.
        :param rospy.Time max_stamp: If set, the filter will only work on messages before this timestamp.
        """
        self.is_raw = is_raw
        """Whether the filter works on raw or deserialized messages."""
        self._include_topics = TopicSet(include_topics)
        """If nonempty, the filter will only work on these topics."""
        self._exclude_topics = TopicSet(exclude_topics)
        """If nonempty, the filter will skip these topics (but pass them further)."""
        self._include_types = TopicSet(include_types)
        """If nonempty, the filter will only work on these message types."""
        self._exclude_types = TopicSet(exclude_types)
        """If nonempty, the filter will skip these message types (but pass them further)."""
        self._min_stamp = min_stamp
        """If set, the filter will only work on messages after this timestamp."""
        self._max_stamp = max_stamp
        """If set, the filter will only work on messages before this timestamp."""
        self._bag = None
        """If this filter is working on a bag, it should be set here before the filter starts being used on the bag."""

    def set_bag(self, bag):
        """If this filter is working on a bag, it should be set here before the filter starts being used on the bag.

        :param rosbag.bag.Bag bag: The bag file open for reading.
        """
        self._bag = bag

    def __call__(self, *args, **kwargs):
        """Do the filtering.

        This method properly selects the raw/deserialized filter() method and calls it.
        :return: The filtered message or None if it should be discarded. A list of messages can also be returned. In
                 such case, the first message is considered to be the "direct" continuation of the input message and
                 it should be directly used. The remaining messages are considered as additional filter inputs and
                 should be fed back to the filter.
        """
        return self.filter(*args, **kwargs)

    def filter(self, *args, **kwargs):
        # type: (...) -> Union[RawFilterResult, DeserializedFilterResult]
        """Filter the message.

        :param args: The message can be either a RawMessageData or a DeserializedMessageData tuple.
        :param kwargs:
        :return: None if the message should be discarded. The possibly changed message otherwise. Multiple messages can
                 be returned, too. In that case, the additional messages should be passed through the filter as if
                 they are newly read messages.
        """
        raise NotImplementedError()

    def consider_message(self, topic, datatype, stamp, header):
        """This function should be called before calling filter(). If it returns False, filter() should not be called
        and the original message should be used instead.

        :param str topic:
        :param str datatype:
        :param Time stamp:
        :param dict header:
        :return: Whether filter() should be called.
        :rtype: bool
        """
        if self._min_stamp is not None and stamp < self._min_stamp:
            return False
        if self._max_stamp is not None and stamp > self._max_stamp:
            return False
        if self._include_topics and topic not in self._include_topics:
            return False
        if self._exclude_topics and topic in self._exclude_topics:
            return False
        if self._include_types and datatype not in self._include_types:
            return False
        if self._exclude_types and datatype in self._exclude_types:
            return False
        return True

    def connection_filter(self, topic, datatype, md5sum, msg_def, header):
        """Connection filter passed to Bag.read_messages().

        :param topic:
        :param datatype:
        :param md5sum:
        :param msg_def:
        :param header:
        :return: If False, the topic will not be read from the input bag.
        :rtype: bool
        """
        return True

    def topic_filter(self, topic):
        """Filter of topics to be read from the bag file.

        :param topic:
        :return: If False, the topic will not be read from the input bag.
        :rtype: bool
        """
        return True

    def reset(self):
        """Reset the filter. This should be called e.g. before starting a new bag."""
        self._bag = None

    @staticmethod
    def add_cli_args(parser):
        """Subclasses may reimplement this static method to specify extra CLI args they provide.

        :param argparse.ArgumentParser parser: The argument parser to configure.
        """
        pass

    @staticmethod
    def process_cli_args(filters, args):
        """Subclasses may reimplement this static method to process the custom CLI args from add_cli_args().

        :param list filters: The list of loaded filters. This method can add to the list.
        :param argparse.Namespace args: The parsed args.
        """
        pass

    @staticmethod
    def yaml_config_args():
        """Subclasses may reimplement this static method to specify extra YAML keys they provide.

        These keys should correspond to the names of the CLI args and they will be read into CLI args when found in the
        YAML file.

        :return: The list of provided YAML keys.
        :rtype: list
        """
        return []

    @staticmethod
    def from_config(cfg):
        """Create a MessageFilter from a config dict.

        Other filters can be defined by 3rd-party packages via pluginlib. The package has to
        `<exec_depend>cras_bag_tools</exec_depend>` and it has to put this line in its `<export>` tag in package.xml:
        `<cras_bag_tools filters="$PACKAGE.$MODULE" />`. With this in place, `filter_bag` will search the specified
        module for all classes that subclass `cras_bag_tools.MessageFilter` and it will provide these as additional
        filters.

        :param cfg: The filter configuration. If a sequence is given, a FilterChain will be created.
        :type cfg: dict or list or tuple
        :return: The configured filter.
        :rtype: MessageFilter
        """
        if cfg is None:
            return None
        # Assume cfg is either a filter config or a sequence of such configs.
        # If it is a sequence, construct filter chain.
        if is_sequence(cfg):
            filters = []
            for d in cfg:
                f = MessageFilter.from_config(d)
                if f is not None:
                    filters.append(f)
            return FilterChain(filters)
        # Assume one of the following filter config structures:
        # {class: [args, kwargs]}.
        # {class: kwargs}.
        assert len(cfg) == 1
        k, v = list(cfg.items())[0]
        if is_sequence(v):
            args = v[0] if len(v) >= 1 else ()
            kwargs = v[1] if len(v) >= 2 else {}
        else:
            args = ()
            kwargs = v

        filters = get_filters()
        if k not in filters:
            print("Filter %s is not defined. Check that its Python module is properly exported in package.xml." % k,
                  file=sys.stderr)
            return None
        # Eval in the current environment.
        f = filters[k](*args, **kwargs)
        return f

    def __str__(self):
        return "%s(%s)" % (self.__class__.__name__, self._str_params())

    def _str_params(self):
        """Parameters to be printed when stringifying this instance. This is called by __str__().

        :return: The parameters to print.
        :rtype: str
        """
        parts = []
        if self.is_raw:
            parts.append('raw')
        if self._include_topics:
            parts.append('include_topics=%s' % str(self._include_topics))
        if self._exclude_topics:
            parts.append('exclude_topics=%s' % str(self._exclude_topics))
        if self._include_types:
            parts.append('include_types=%s' % str(self._include_types))
        if self._exclude_types:
            parts.append('exclude_types=%s' % str(self._exclude_types))
        if self._min_stamp:
            parts.append('min_stamp=%s' % str(self._min_stamp))
        if self._max_stamp:
            parts.append('max_stamp=%s' % str(self._max_stamp))
        return ",".join(parts)


class RawMessageFilter(MessageFilter):
    """
    Message filter that processes raw messages.
    """

    def __init__(self, *args, **kwargs):
        super(RawMessageFilter, self).__init__(True, *args, **kwargs)

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        # type: (...) -> RawFilterResult
        """Do the filtering.

        :param str topic: Topic of the message.
        :param str datatype: ROS datatype of the message (as string).
        :param bytes data: The raw data.
        :param str md5sum: MD5 sum of the datatype.
        :param type pytype: ROS datatype of the message (as Python type).
        :param rospy.Time stamp: Receive timestamp of the message.
        :param dict header: Connection header.
        :return: None if the message should be discarded, or raw message(s).
        """
        raise NotImplementedError


class DeserializedMessageFilter(MessageFilter):
    """
    Message filter that processes deserialized messages.
    """

    def __init__(self, *args, **kwargs):
        super(DeserializedMessageFilter, self).__init__(False, *args, **kwargs)

    def filter(self, topic, msg, stamp, header):
        # type: (...) -> DeserializedFilterResult
        """Do the filtering.

        :param str topic: Topic of the message.
        :param genpy.Message msg: The decoded message.
        :param rospy.Time stamp: Receive timestamp of the message.
        :param dict header: Connection header.
        :return: None if the message should be discarded, or a deserialized message(s).
        """
        raise NotImplementedError


class Passthrough(RawMessageFilter):
    """
    Just pass all messages through.
    """

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        return topic, datatype, data, md5sum, pytype, stamp, header


class FilterChain(RawMessageFilter):
    """
    A chain of message filters.
    """

    def __init__(self, filters):
        """Constructor.

        :param filters: The filters to add.
        :type filters: list of MessageFilter
        """
        super(FilterChain, self).__init__()
        self.filters = filters

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        msg = None
        last_was_raw = True
        additional_msgs = []
        for f in self.filters:
            if not f.consider_message(topic, datatype, stamp, header):
                continue
            if f.is_raw:
                if not last_was_raw:
                    datatype, data, md5sum, pytype = msg_to_raw(msg)
                ret = f(topic, datatype, data, md5sum, pytype, stamp, header)
                if not isinstance(ret, list):
                    ret = [ret]
                if ret[0] is None:
                    if len(ret) == 1 and len(additional_msgs) == 0:
                        return None
                    return [None] + additional_msgs + ret[1:]
                topic, datatype, data, md5sum, pytype, stamp, header = ret[0]
                additional_msgs.extend(ret[1:])
                last_was_raw = True
            else:
                if last_was_raw:
                    msg = raw_to_msg(datatype, data, md5sum, pytype)
                ret = f(topic, msg, stamp, header)
                if not isinstance(ret, list):
                    ret = [ret]
                if ret[0] is None:
                    if len(ret) == 1 and len(additional_msgs) == 0:
                        return None
                    return [None] + additional_msgs + ret[1:]
                topic, msg, stamp, header = ret[0]
                datatype = msg.__class__._type  # needed in consider_message() above
                additional_msgs.extend(ret[1:])
                last_was_raw = False
        if not last_was_raw:
            datatype, data, md5sum, pytype = msg_to_raw(msg)
        ret = topic, datatype, data, md5sum, pytype, stamp, header
        if len(additional_msgs) == 0:
            return ret
        return [ret] + additional_msgs

    def connection_filter(self, topic, datatype, md5sum, msg_def, header):
        for f in self.filters:
            if not f.connection_filter(topic, datatype, md5sum, msg_def, header):
                return False
        return True

    def topic_filter(self, topic):
        for f in self.filters:
            if not f.topic_filter(topic):
                return False
        return True

    def set_bag(self, bag):
        for f in self.filters:
            f.set_bag(bag)
        super(FilterChain, self).set_bag(bag)

    def reset(self):
        for f in self.filters:
            f.reset()
        super(FilterChain, self).reset()

    def __str__(self):
        return '%s(%s)' % (self.__class__.__name__, ', '.join(str(f) for f in self.filters))

    def __iadd__(self, other):
        if other is None:
            return self
        assert isinstance(other, MessageFilter)
        if isinstance(other, FilterChain):
            self.filters += other.filters
            return self

        self.filters.append(other)
        return self

    def __add__(self, other):
        if other is None:
            return self
        assert isinstance(other, MessageFilter)
        if isinstance(other, FilterChain):
            return FilterChain(self.filters + other.filters)
        return FilterChain(self.filters + [other.filters])


def fix_connection_header(header, datatype, md5sum, pytype):
    header["message_definition"] = pytype._full_text
    header["md5sum"] = md5sum
    header["type"] = datatype
    return header


def filter_message(topic, msg, stamp, connection_header, filter, raw_output=True):
    """Apply the given filter to a message.

    :param str topic: The message topic.
    :param msg: The message (either a deserialized message or a raw message as 4-tuple).
    :type msg: genpy.Message or tuple
    :param rospy.Time stamp: Receive timestamp of the message.
    :param dict connection_header: Connection header.
    :param MessageFilter filter: The filter to apply.
    :param bool raw_output: Whether to output a raw message or a deserialized one.
    :return: None if the message should be discarded, or a message.
    :rtype: tuple or None
    """
    additional_msgs = []
    if filter.is_raw:
        # Convert to raw if decoded message was given
        try:
            datatype, data, md5sum, _, pytype = msg
        except (ValueError, TypeError):
            datatype, data, md5sum, pytype = msg_to_raw(msg)
        if filter.consider_message(topic, datatype, stamp, connection_header):
            ret = filter(topic, datatype, data, md5sum, pytype, stamp, connection_header)
            if not isinstance(ret, list):
                ret = [ret]
            if ret[0] is None:
                return None if len(ret) == 1 else ret
            topic, datatype, data, md5sum, pytype, stamp, connection_header = ret[0]
            additional_msgs.extend(ret[1:])
        out_msg = (datatype, data, md5sum, pytype) if raw_output else raw_to_msg(datatype, data, md5sum, pytype)
    else:
        # Decode the message if raw was given
        if not isinstance(msg, genpy.Message):
            datatype, data, md5sum, _, pytype = msg
            msg = raw_to_msg(datatype, data, md5sum, pytype)
        if filter.consider_message(topic, msg.__class__._type, stamp, connection_header):
            ret = filter(topic, msg, stamp, connection_header)
            if not isinstance(ret, list):
                ret = [ret]
            if ret[0] is None:
                return None if len(ret) == 1 else ret
            topic, msg, stamp, connection_header = ret[0]
            additional_msgs.extend(ret[1:])
        datatype = msg.__class__._type
        md5sum = msg.__class__._md5sum
        pytype = msg.__class__
        out_msg = msg_to_raw(msg) if raw_output else msg

    # make sure connection header corresponds to the actual data type of the message
    # (if the filter forgot to update it)
    connection_header = fix_connection_header(connection_header, datatype, md5sum, pytype)

    ret = topic, out_msg, stamp, connection_header

    if len(additional_msgs) == 0:
        return ret
    return [ret] + additional_msgs
