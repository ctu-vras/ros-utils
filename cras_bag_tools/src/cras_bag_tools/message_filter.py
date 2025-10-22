# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A message filter that can decide whether a message should be kept or not, and possibly alter it."""

from __future__ import absolute_import, division, print_function

import os.path
import re
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

import genpy
import rospkg
import rospy
from cras.message_utils import raw_to_msg, msg_to_raw
from cras.plugin_utils import get_plugin_implementations
from cras.string_utils import to_str, STRING_TYPE

from .bag_utils import MultiBag
from .time_range import TimeRange, TimeRanges
from .topic_set import TopicSet


def is_sequence(o):
    return isinstance(o, (list, tuple))


loaded_filters = None


def get_filters():
    """Get all defined message filters.

    :return: The dictionary of (filter name => filter).
    :rtype: dict
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
    """Base class for message filters. Do not implement this directly: instead implement either RawMessageFilter or
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

    The filter can also override extra_time_ranges(). The "extra" time ranges are time ranges of the bagfile that
    should be read regardless of the normal start/end/min_stamp/max_stamp time ranges. Data from "extra" time ranges
    are not considered by default, so the filter has to override consider_message() to actually receive them.
    This mechanism is meant to support e.g. reading latched static TFs from the start of the bag file even when
    working just on a part of the bag that does not start at the bag beginning.
    """

    def __init__(self, is_raw, include_topics=None, exclude_topics=None, include_types=None, exclude_types=None,
                 min_stamp=None, max_stamp=None, include_time_ranges=None, exclude_time_ranges=None):
        """Constructor.

        :param bool is_raw: Whether the filter works on raw or deserialized messages.
        :param list include_topics: If nonempty, the filter will only work on these topics.
        :param list exclude_topics: If nonempty, the filter will skip these topics (but pass them further).
        :param list include_types: If nonempty, the filter will only work on these message types.
        :param list exclude_types: If nonempty, the filter will skip these message types (but pass them further).
        :param rospy.Time min_stamp: If set, the filter will only work on messages after this timestamp.
        :param rospy.Time max_stamp: If set, the filter will only work on messages before this timestamp.
        :param include_time_ranges: Time ranges that specify which regions of the bag should be processed.
                                    List of pairs (start, end_or_duration) or a TimeRanges object.
        :type include_time_ranges: list or TimeRanges
        :param exclude_time_ranges: Time ranges that specify which regions of the bag should be skipped.
                                    List of pairs (start, end_or_duration) or a TimeRanges object.
        :type exclude_time_ranges: list or TimeRanges
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
        self._params = None
        """If ROS parameters are recorded for the bag, they should be passed here."""
        self._include_time_ranges = self._parse_time_ranges(include_time_ranges)
        """Time ranges that specify which regions of the bag should be processed by this filter. If empty,
        the filter should work for all time regions except the excluded ones."""
        self._exclude_time_ranges = self._parse_time_ranges(exclude_time_ranges)
        """Time ranges that specify which regions of the bag should be skipped (but passed further)."""

        self.__rospack = None

    @staticmethod
    def _parse_time_ranges(ranges):
        if ranges is None:
            return None
        return TimeRanges(TimeRange(start, end) for start, end in ranges)

    def set_bag(self, bag):
        """If this filter is working on a bag, it should be set here before the filter starts being used on the bag.

        :param rosbag.bag.Bag bag: The bag file open for reading.
        """
        self._bag = bag

        start_time = bag.get_start_time()
        if self._include_time_ranges is not None:
            self._include_time_ranges.set_base_time(start_time)
        if self._exclude_time_ranges is not None:
            self._exclude_time_ranges.set_base_time(start_time)

    def set_params(self, params):
        """Set the ROS parameters recorded for the currently open bag file.

        :param params: The ROS parameters.
        :type params: dict
        """
        self._params = params

    def _get_param(self, param, default=None):
        """Get parameter `param` from the parameters set by :meth:`set_params`.

        :param str param: The parameter to get.
        :param default: The default value returned in case the parameter is not found.
        :return: The found parameter value or the default.
        """
        return self.__get_param(self._params, param, default)

    def __get_param(self, params, param, default=None):
        if param.startswith("/"):
            param = param[1:]
        if params is None:
            return default
        if param in params:
            return params[param]
        if "/" not in param:
            return default
        key, rest = param.split("/", 1)
        return self.__get_param(params.get(key, None), rest, default)

    def _set_param(self, param, value):
        """Set parameter `param` to the parameters set by :meth:`set_params`.

        :param str param: Name of the parameter.
        :param value: The value to set.
        """
        self.__set_param(self._params, param, value)

    def __set_param(self, params, param, value):
        if param.startswith("/"):
            param = param[1:]
        if "/" in param:
            key, rest = param.split("/", 1)
            if key not in params:
                params[key] = {}
            self.__set_param(params[key], rest, value)
        else:
            params[param] = value

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

    def consider_message(self, topic, datatype, stamp, header, is_from_extra_time_range=False):
        """This function should be called before calling filter(). If it returns False, filter() should not be called
        and the original message should be used instead.

        :param str topic:
        :param str datatype:
        :param Time stamp:
        :param dict header:
        :param bool is_from_extra_time_range: If True, this message comes from extra time range.
        :return: Whether filter() should be called. If False, the message is passed to the next filter (not discarded).
        :rtype: bool
        """
        # Filters have to explicitly opt-in to reading messages from extra time ranges.
        if is_from_extra_time_range:
            return False
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
        if self._include_time_ranges and stamp not in self._include_time_ranges:
            return False
        if self._exclude_time_ranges and stamp in self._exclude_time_ranges:
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

    def extra_initial_messages(self):
        # type: () -> Iterable[Tuple[STRING_TYPE, Any, rospy.Time, Optional[Dict[STRING_TYPE, STRING_TYPE]]]]
        """Get extra messages that should be passed to the filter before the iteration over bag messages starts.

        This can be used e.g. by filters that are more generators the actual filters (i.e. they do not operate on
        existing messages, but instead create new ones based on other information).

        :note: :py:meth:`set_bag` should be called before calling this method.
        :note: Do not generate very large data. All initial messages will be stored in RAM at once.
        :return: A list or iterator of the message 4-tuples `(topic, message, stamp, connection_header)`.
        """
        return []

    def extra_final_messages(self):
        # type: () -> Iterable[Tuple[STRING_TYPE, Any, rospy.Time, Optional[Dict[STRING_TYPE, STRING_TYPE]]]]
        """Get extra messages that should be passed to the filter after the iteration over bag messages stops.

        This can be used e.g. by filters that are more generators the actual filters (i.e. they do not operate on
        existing messages, but instead create new ones based on other information).

        :note: :py:meth:`set_bag` should be called before calling this method.
        :note: Do not generate very large data. All final messages will be stored in RAM at once.
        :return: A list or iterator of the message 4-tuples `(topic, message, stamp, connection_header)`.
        """
        return []

    def extra_time_ranges(self, bags):
        """If this filter requires that certain time ranges are read from the bagfiles in any case (like the static
        TFs at the beginning), it should return the requested time range here. Such parts of the bag will be always
        passed to the filter regardless of the include/exclude time ranges of this filter and the start/end times
        of the whole bag.

        :param bags: The bags that are going to be processed.
        :type bags: rosbag.Bag or MultiBag
        :return: The time ranges of the input bags that should always be read. If None, no extra ranges are required.
        :rtype: TimeRanges or None
        """
        return None

    def __get_rospack(self):
        if self.__rospack is None:
            self.__rospack = rospkg.RosPack()
        return self.__rospack

    def resolve_file(self, filename):
        """Resolve `filename` relative to the bag set by :meth:`set_bag`.

        :note: This is ideally called from :meth:`on_filtering_start` or :meth:`filter` because earlier, the `_bag`
               member variable is not set.
        """
        matches = re.match(r'\$\(find ([^)]+)\)', filename)
        if matches is not None:
            package_path = self.__get_rospack().get_path(matches[1])
            filename = filename.replace('$(find %s)' % (matches[1],), package_path)

        if self._bag is None or os.path.isabs(filename) or len(self._bag.filename) == 0:
            return filename

        return os.path.join(os.path.dirname(self._bag.filename), filename)

    def on_filtering_start(self):
        """This function is called right before the first message is passed to filter().

        :note: Specifically, :meth:`set_params` and :meth:`set_bag` are already called at this stage.
        :note: :meth:`extra_initial_messages` will be called after calling this method.
        """
        pass

    def on_filtering_end(self):
        """This function is called right after the last message is processed by filter()."""
        pass

    def reset(self):
        """Reset the filter. This should be called e.g. before starting a new bag."""
        self._bag = None
        self._params = None

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

    def _default_str_params(self, include_topics=True, exclude_topics=True, include_types=True, exclude_types=True,
                            min_stamp=True, max_stamp=True, include_time_ranges=True, exclude_time_ranges=True):
        """Parameters to be printed when stringifying this instance. This is called by __str__().

        :return: The parameters to print.
        :rtype: str
        """
        parts = []
        if self.is_raw:
            parts.append('raw')
        if self._include_topics and include_topics:
            parts.append('include_topics=%s' % str(self._include_topics))
        if self._exclude_topics and exclude_topics:
            parts.append('exclude_topics=%s' % str(self._exclude_topics))
        if self._include_types and include_types:
            parts.append('include_types=%s' % str(self._include_types))
        if self._exclude_types and exclude_types:
            parts.append('exclude_types=%s' % str(self._exclude_types))
        if self._min_stamp and min_stamp:
            parts.append('min_stamp=%s' % to_str(self._min_stamp))
        if self._max_stamp and max_stamp:
            parts.append('max_stamp=%s' % to_str(self._max_stamp))
        if self._include_time_ranges and include_time_ranges:
            parts.append('include_time_ranges=%s' % str(self._include_time_ranges))
        if self._exclude_time_ranges and exclude_time_ranges:
            parts.append('exclude_time_ranges=%s' % str(self._exclude_time_ranges))
        return ",".join(parts)

    def _str_params(self):
        """Parameters to be printed when stringifying this instance. This is called by __str__().

        :return: The parameters to print.
        :rtype: str
        """
        return self._default_str_params()


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


class DeserializedMessageFilterWithTF(DeserializedMessageFilter):
    """Filter for deserialized messages that always correctly reads static TFs from the bag start."""

    def __init__(self, include_topics=None, include_types=None, tf_topics=("/tf",), tf_static_topics=("/tf_static",),
                 initial_bag_part_duration=genpy.Duration(2), *args, **kwargs):
        self._tf_static_topics = tf_static_topics
        self._initial_bag_part_duration = initial_bag_part_duration

        if include_topics is not None:
            for t in tf_topics + tf_static_topics:
                if t not in include_topics:
                    include_topics.append(t)
        if include_types is not None:
            if "tf2_msgs/TFMessage" not in include_types:
                include_types.append("tf2_msgs/TFMessage")
        super(DeserializedMessageFilterWithTF, self).__init__(
            include_topics=include_topics, include_types=include_types, *args, **kwargs)

    def consider_message(self, topic, datatype, stamp, header, is_from_extra_time_ranges=False):
        # /tf has standard rules for being considered, but tf_static needs to be always accepted
        if topic in self._tf_static_topics:
            return True
        return super(DeserializedMessageFilterWithTF, self).consider_message(
            topic, datatype, stamp, header, is_from_extra_time_ranges)

    def extra_time_ranges(self, bags):
        # Require the beginnings of all bag files where static TFs can be stored
        return TimeRanges([TimeRange(0, self._initial_bag_part_duration)])


class Passthrough(RawMessageFilter):
    """
    Just pass all messages through.
    """

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        return topic, datatype, data, md5sum, pytype, stamp, header


class NoMessageFilter(RawMessageFilter):
    """
    Ignore all messages. Good base for message generators or other helpers.
    """

    def consider_message(self, topic, datatype, stamp, header, is_from_extra_time_range=False):
        return False


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

    def consider_message(self, topic, datatype, stamp, header, is_from_extra_time_range=False):
        return True  # actual considering is done in the filter() loop

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, is_from_extra_time_range=False):
        msg = None
        last_was_raw = True
        additional_msgs = []
        for f in self.filters:
            if not f.consider_message(topic, datatype, stamp, header, is_from_extra_time_range):
                continue
            if f.is_raw:
                if not last_was_raw:
                    datatype, data, md5sum, pytype = msg_to_raw(msg)
                if isinstance(f, FilterChain):
                    ret = f(topic, datatype, data, md5sum, pytype, stamp, header, is_from_extra_time_range)
                else:
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

    def set_params(self, params):
        for f in self.filters:
            f.set_params(params)
        super(FilterChain, self).set_params(params)

    def extra_time_ranges(self, bags):
        time_ranges = None
        for f in self.filters:
            extra_ranges = f.extra_time_ranges(bags)
            if extra_ranges is not None:
                if time_ranges is None:
                    time_ranges = TimeRanges([])
                time_ranges.append(extra_ranges.ranges)
        return time_ranges

    def extra_initial_messages(self):
        for f in self.filters:
            for m in f.extra_initial_messages():
                yield m

    def extra_final_messages(self):
        for f in self.filters:
            for m in f.extra_final_messages():
                yield m

    def on_filtering_start(self):
        for f in self.filters:
            f.on_filtering_start()
        super(FilterChain, self).on_filtering_start()

    def on_filtering_end(self):
        for f in self.filters:
            f.on_filtering_end()
        super(FilterChain, self).on_filtering_end()

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


def fix_connection_header(header, topic, datatype, md5sum, pytype):
    header["topic"] = topic
    header["message_definition"] = pytype._full_text
    header["md5sum"] = md5sum
    header["type"] = datatype
    return header


def filter_message(topic, msg, stamp, connection_header, filter, raw_output=True, is_from_extra_time_range=False):
    """Apply the given filter to a message.

    :param str topic: The message topic.
    :param msg: The message (either a deserialized message or a raw message as 4-tuple).
    :type msg: genpy.Message or tuple
    :param rospy.Time stamp: Receive timestamp of the message.
    :param dict connection_header: Connection header.
    :param MessageFilter filter: The filter to apply.
    :param bool raw_output: Whether to output a raw message or a deserialized one.
    :param bool is_from_extra_time_range: If True, this message comes from extra time range.
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
        if filter.consider_message(topic, datatype, stamp, connection_header, is_from_extra_time_range):
            if isinstance(filter, FilterChain):
                ret = filter(topic, datatype, data, md5sum, pytype, stamp, connection_header, is_from_extra_time_range)
            else:
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
        if filter.consider_message(topic, msg.__class__._type, stamp, connection_header, is_from_extra_time_range):
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
    connection_header = fix_connection_header(connection_header, topic, datatype, md5sum, pytype)

    ret = topic, out_msg, stamp, connection_header

    if len(additional_msgs) == 0:
        return ret
    return [ret] + additional_msgs


__all__ = [
    DeserializedMessageFilter.__name__,
    DeserializedMessageFilterWithTF.__name__,
    FilterChain.__name__,
    MessageFilter.__name__,
    Passthrough.__name__,
    RawMessageFilter.__name__,
    filter_message.__name__,
    get_filters.__name__,
]
