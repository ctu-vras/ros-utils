# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Implementations of common message filters."""

from __future__ import absolute_import, division, print_function

import copy
import matplotlib.cm as cmap
import numpy as np
import sys
from typing import Dict

import rospy
from cras.image_encodings import isColor, isMono, isBayer, isDepth, bitDepth, numChannels, MONO8, BGR8,\
    TYPE_16UC1, TYPE_32FC1, YUV422
import cv2  # Workaround for https://github.com/opencv/opencv/issues/14884 on Jetsons.
from cv_bridge import CvBridge, CvBridgeError
from image_transport_codecs import decode, encode
from image_transport_codecs.compressed_depth_codec import has_rvl
from image_transport_codecs.parse_compressed_format import guess_any_compressed_image_transport_format
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from .message_filter import DeserializedMessageFilter, RawMessageFilter, TopicSet


def dict_to_str(d, sep='='):
    return ', '.join('%s%s%s' % (k, sep, v) for k, v in d.items())


class SetFields(DeserializedMessageFilter):
    """Change values of some fields of a message (pass the fields to change as kwargs)."""

    def __init__(self, include_topics=None, exclude_topics=None, include_types=None, exclude_types=None,
                 min_stamp=None, max_stamp=None, **kwargs):
        """
        :param list include_topics: If nonempty, the filter will only work on these topics.
        :param list exclude_topics: If nonempty, the filter will skip these topics (but pass them further).
        :param list include_types: If nonempty, the filter will only work on these message types.
        :param list exclude_types: If nonempty, the filter will skip these message types (but pass them further).
        :param rospy.Time min_stamp: If set, the filter will only work on messages after this timestamp.
        :param rospy.Time max_stamp: If set, the filter will only work on messages before this timestamp.
        :param dict kwargs: The fields to set. Keys are field name, values are the new values to set.
        """
        super(SetFields, self).__init__(
            include_topics, exclude_topics, include_types, exclude_types, min_stamp, max_stamp)
        self.field_values = kwargs

    def filter(self, topic, msg, stamp, header):
        for k, v in self.field_values.items():
            if k not in msg.__slots__:
                continue
            setattr(msg, k, v)
        return topic, msg, stamp, header

    def _str_params(self):
        parts = []
        params = dict_to_str(self.field_values)
        if len(params) > 0:
            parts.append(params)
        parent_params = super(SetFields, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class FixHeader(DeserializedMessageFilter):
    """Change the header of a message."""

    def __init__(self, frame_id="", frame_id_prefix="", frame_id_suffix="", stamp_from_receive_time=False,
                 stamp_offset=0.0, *args, **kwargs):
        """
        :param str frame_id: If nonempty, frame_id will be set to this value.
        :param str frame_id_prefix: If nonempty, header will be prefixed with this value.
        :param str frame_id_suffix: If nonempty, header will be suffixed with this value.
        :param bool stamp_from_receive_time: If true, set stamp from the connection header receive time.
        :param stamp_offset: If nonzero, offset the stamp by this duration (seconds).
        :type stamp_offset: float or rospy.Duration
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(FixHeader, self).__init__(*args, **kwargs)
        self.frame_id = frame_id
        self.frame_id_prefix = frame_id_prefix
        self.frame_id_suffix = frame_id_suffix
        self.stamp_from_receive_time = stamp_from_receive_time
        self.stamp_offset = rospy.Duration(stamp_offset)

    def filter(self, topic, msg, stamp, header):
        if len(msg.__slots__) == 0 or msg.__slots__[0] != 'header':
            return topic, msg, stamp, header

        if len(self.frame_id) > 0:
            msg.header.frame_id = self.frame_id
        else:
            if len(self.frame_id_prefix) > 0:
                msg.header.frame_id = self.frame_id_prefix + msg.header.frame_id
            if len(self.frame_id_suffix) > 0:
                msg.header.frame_id += self.frame_id_suffix

        if self.stamp_from_receive_time:
            msg.header.stamp = stamp
        msg.header.stamp += self.stamp_offset

        return topic, msg, stamp, header

    def _str_params(self):
        parts = []
        if len(self.frame_id) > 0:
            parts.append('frame_id=' + self.frame_id)
        if len(self.frame_id_prefix) > 0:
            parts.append('frame_id_prefix=' + self.frame_id_prefix)
        if len(self.frame_id_suffix) > 0:
            parts.append('frame_id_suffix=' + self.frame_id_suffix)
        if self.stamp_from_receive_time:
            parts.append('stamp_from_receive_time')
        if self.stamp_offset != rospy.Duration(0, 0):
            parts.append('stamp_offset=' + self.stamp_offset)
        parent_params = super(FixHeader, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class Deduplicate(RawMessageFilter):
    """Discard all messages except each first changed."""

    def __init__(self, ignore_seq=False, ignore_stamp=False, per_frame_id=False, max_ignored_duration=None,
                 *args, **kwargs):
        """
        :param bool ignore_seq: If True, differing header.seq will not make a difference.
        :param bool ignore_stamp: If True, differing header.stamp will not make a difference.
        :param bool per_frame_id: If True, messages will be clustered by header.frame_id and comparisons will only be
                                  made between messages with equal frame_id.
        :param float max_ignored_duration: If set, a duplicate will pass if its stamp is further from the last passed
                                           message than the given duration (in seconds).
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(Deduplicate, self).__init__(*args, **kwargs)
        self._ignore_seq = ignore_seq
        self._ignore_stamp = ignore_stamp
        self._per_frame_id = per_frame_id
        self._max_ignored_duration = rospy.Duration(max_ignored_duration)
        self._last_msgs = {}

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        has_header = pytype.__slots__[0] == 'header'
        key = topic
        if has_header and self._per_frame_id:
            key = "%s@%s" % (topic, Header().deserialize(data).frame_id)

        if key not in self._last_msgs:
            self._last_msgs[key] = data, stamp
            return topic, datatype, data, md5sum, pytype, stamp, header

        last_msg, last_msg_stamp = self._last_msgs[key]

        seq_ok = True
        stamp_ok = True
        stamp_diff_ok = True
        compare_idx = 0

        if has_header:
            seq_ok = self._ignore_seq or data[0:4] == last_msg[0:4]
            stamp_ok = self._ignore_stamp or data[4:12] == last_msg[4:12]
            compare_idx = 12
        if self._max_ignored_duration is not None:
            stamp_diff_ok = stamp - last_msg_stamp < self._max_ignored_duration

        self._last_msgs[key] = data, stamp

        if seq_ok and stamp_ok and stamp_diff_ok and data[compare_idx:] == last_msg[compare_idx:]:
            return None

        return topic, datatype, data, md5sum, pytype, stamp, header

    def _str_params(self):
        parts = []
        parts.append('ignore_seq=%r' % (self._ignore_seq,))
        parts.append('ignore_stamp=%r' % (self._ignore_stamp,))
        parts.append('per_frame_id=%r' % (self._per_frame_id,))
        parts.append('max_ignored_duration=%r' % (self._max_ignored_duration,))
        parent_params = super(Deduplicate, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class Remap(RawMessageFilter):
    """Remap topics."""

    def __init__(self, **kwargs):
        """
        :param dict kwargs: The mapping to use. Keys are topics to be remapped, values are their new names.
        """
        super(Remap, self).__init__(include_topics=kwargs.keys())
        self.remap = kwargs

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        return self.remap.get(topic, topic), datatype, data, md5sum, pytype, stamp, header

    def _str_params(self):
        return dict_to_str(self.remap, '=>')

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument(
            '--remap', nargs='+', metavar="FROM TO",
            help="Remap topics. This argument should be an even-sized list of pairs [FROM TO].")

    @staticmethod
    def process_cli_args(filters, args):
        if args.remap:
            topics_from = args.remap[0::2]
            topics_to = args.remap[1::2]
            filters.append(Remap(**dict(zip(topics_from, topics_to))))


class Throttle(RawMessageFilter):
    """Throttle messages on topics."""

    def __init__(self, **kwargs):
        """
        :param kwargs: Keys are topics to be throttled, values are their maximum frequencies.
        :type kwargs: Dict[str, float]
        """
        super(Throttle, self).__init__(include_topics=kwargs.keys())
        self.hz = kwargs
        self.prev_t = {}

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        t = stamp.to_sec()
        if topic in self.prev_t:
            period = t - self.prev_t[topic]
            if period <= 0.:
                return None
            hz = 1. / period
            if hz > self.hz[topic]:
                return None
        self.prev_t[topic] = t
        return topic, datatype, data, md5sum, pytype, stamp, header

    def reset(self):
        self.prev_t = {}
        super(Throttle, self).reset()

    def _str_params(self):
        return dict_to_str(self.hz, '@')

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument(
            '--throttle', '--hz', nargs='+', metavar="TOPIC RATE",
            help="Throttle messages. This argument should be an even-sized list of pairs [TOPIC RATE].")

    @staticmethod
    def yaml_config_args():
        return 'throttle',

    @staticmethod
    def process_cli_args(filters, args):
        if args.throttle:
            topics = args.throttle[0::2]
            hz = [float(hz) for hz in args.throttle[1::2]]
            filters.append(Throttle(**dict(zip(topics, hz))))


class Topics(RawMessageFilter):
    """Select topics that will be retained or removed. This works as a global filter."""

    def __init__(self, include_topics=None, exclude_topics=None):
        """
        :param list include_topics: If nonempty, all topics not on this list will be dropped.
        :param list exclude_topics: If nonempty, all topics on this list will be dropped.
        """
        super(Topics, self).__init__()
        # do not use _include_topics and _exclude_topics here as they would not allow the filter to reject the messages
        self.include = TopicSet(include_topics)
        self.exclude = TopicSet(exclude_topics)

    def topic_filter(self, topic):
        if self.include and topic not in self.include:
            return False
        if self.exclude and topic in self.exclude:
            return False
        return True

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        # The filtering is done in topic_filter()
        return topic, datatype, data, md5sum, pytype, stamp, header

    def _str_params(self):
        params = []
        if self.include:
            params.append('include_topics=' + str(self.include))
        if self.exclude:
            params.append('exclude_topics=' + str(self.exclude))
        return ", ".join(params)

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument('-i', '--include-topics', nargs='+', help="Retain only these topics")
        parser.add_argument('-e', '--exclude-topics', nargs='+', help="Remove these topics")

    @staticmethod
    def yaml_config_args():
        return 'include_topics', 'exclude_topics'

    @staticmethod
    def process_cli_args(filters, args):
        if args.include_topics or args.exclude_topics:
            filters.append(Topics(include_topics=args.include_topics, exclude_topics=args.exclude_topics))


class TopicTypes(RawMessageFilter):
    """Select topic types that will be retained or removed. This works as a global filter."""

    def __init__(self, include_types=None, exclude_types=None):
        """
        :param list include_types: If nonempty, messages of types not on this list will be dropped.
        :param list exclude_types: If nonempty, messages of types on this list will be dropped.
        """
        super(TopicTypes, self).__init__()
        # do not use _include_types and _exclude_types here as they would not allow the filter to reject the messages
        self.include = TopicSet(include_types)
        self.exclude = TopicSet(exclude_types)

    def connection_filter(self, topic, datatype, md5sum, msg_def, header):
        if self.include and datatype not in self.include:
            return False
        if self.exclude and datatype in self.exclude:
            return False
        return True

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        # The filtering is done in connection_filter()
        return topic, datatype, data, md5sum, pytype, stamp, header

    def _str_params(self):
        params = []
        if self.include:
            params.append('include_types=' + str(self.include))
        if self.exclude:
            params.append('exclude_types=' + str(self.exclude))
        return ", ".join(params)

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument('--include-types', nargs='+', help="Retain only messages of these types")
        parser.add_argument('--exclude-types', nargs='+', help="Remove messages of these types")

    @staticmethod
    def yaml_config_args():
        return 'include_types', 'exclude_types'

    @staticmethod
    def process_cli_args(filters, args):
        if args.include_types or args.exclude_types:
            filters.append(TopicTypes(include_types=args.include_types, exclude_types=args.exclude_types))


class Drop(RawMessageFilter):
    """Drop matching messages. The difference between Topics and Drop is that Drop acts locally, i.e. can be used in the
     middle of a filter chain."""

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        return None


class Transforms(DeserializedMessageFilter):
    """Filter or change transforms."""

    def __init__(self, include_parents=(), exclude_parents=(), include_children=(), exclude_children=(),
                 change=None, include_topics=None, exclude_topics=None, include_types=None, exclude_types=None,
                 *args, **kwargs):
        """
        :param list include_parents: If nonempty, only TFs with one of the listed frames as parent will be retained.
        :param list exclude_parents: If nonempty, TFs with one of the listed frames as parent will be dropped.
        :param list include_children: If nonempty, only TFs with one of the listed frames as child will be retained.
        :param list exclude_children: If nonempty, TFs with one of the listed frames as child will be dropped.
        :param dict change: A multilevel dictionary. First key is parent frame. Second key is child frame. Values are
                            dicts with optional keys 'translation', 'rotation', 'frame_id', 'child_frame_id'.
                            'translation' is a dict with optional keys 'x', 'y', 'z'.
                            'rotation' is a dict with optional keys 'x', 'y', 'z', 'ẅ́'.
                            If any of these optional keys is defined, it overwrites the values in the TF message.
        :param list include_topics: The topics on which this filter operates. Leave empty for the default set of topics.
        :param list exclude_topics: Do not operate on these topics.
        :param list include_types: Ignored.
        :param list exclude_types: Ignored.
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(Transforms, self).__init__(
            include_topics=include_topics if include_topics is not None else ['/tf', '/tf_static'],
            exclude_topics=exclude_topics, include_types=['tf2_msgs/TFMessage'], *args, **kwargs)
        self.include_parents = TopicSet(include_parents)
        self.exclude_parents = TopicSet(exclude_parents)
        self.include_children = TopicSet(include_children)
        self.exclude_children = TopicSet(exclude_children)
        self.change = change if change is not None else {}
        self.changed_parents = TopicSet(self.change.keys())

    def filter(self, topic, msg, stamp, header):
        if self.include_parents:
            msg.transforms = [tf for tf in msg.transforms if tf.header.frame_id in self.include_parents]
        if self.exclude_parents:
            msg.transforms = [tf for tf in msg.transforms if tf.header.frame_id not in self.exclude_parents]
        if self.include_children:
            msg.transforms = [tf for tf in msg.transforms if tf.child_frame_id in self.include_children]
        if self.exclude_children:
            msg.transforms = [tf for tf in msg.transforms if tf.child_frame_id not in self.exclude_children]
        if len(self.change) > 0:
            for transform in msg.transforms:
                if transform.header.frame_id in self.changed_parents:
                    if transform.child_frame_id in self.change[transform.header.frame_id]:
                        changes = self.change[transform.header.frame_id][transform.child_frame_id]
                        if 'translation' in changes:
                            transform.translation.x = float(changes['translation'].get('x', transform.translation.x))
                            transform.translation.y = float(changes['translation'].get('y', transform.translation.y))
                            transform.translation.z = float(changes['translation'].get('z', transform.translation.z))
                        if 'rotation' in changes:
                            transform.rotation.x = float(changes['rotation'].get('x', transform.rotation.x))
                            transform.rotation.y = float(changes['rotation'].get('y', transform.rotation.y))
                            transform.rotation.z = float(changes['rotation'].get('z', transform.rotation.z))
                            transform.rotation.w = float(changes['rotation'].get('w', transform.rotation.w))
                        if 'frame_id' in changes:
                            transform.header.frame_id = changes['frame_id']
                        if 'child_frame_id' in changes:
                            transform.child_frame_id = changes['child_frame_id']
        if not msg.transforms:
            return None
        return topic, msg, stamp, header

    def _str_params(self):
        parts = []
        if self.include_parents:
            parts.append('include_parents=' + str(self.include_parents))
        if self.exclude_parents:
            parts.append('exclude_parents=' + str(self.exclude_parents))
        if self.include_children:
            parts.append('include_children=' + str(self.include_children))
        if self.exclude_children:
            parts.append('exclude_children=' + str(self.exclude_children))
        if len(self.change) > 0:
            parts.append('change=%r' % (self.change,))
        parent_params = super(Transforms, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument('--include-tf-parents', nargs='+', help="Retain only TFs with these frames as parents")
        parser.add_argument('--exclude-tf-parents', nargs='+', help="Remove TFs with these frames as parents")
        parser.add_argument('--include-tf-children', nargs='+', help="Retain only TFs with these frames as children")
        parser.add_argument('--exclude-tf-children', nargs='+', help="Remove TFs with these frames as children")

    @staticmethod
    def yaml_config_args():
        return 'include_tf_parents', 'exclude_tf_parents', 'include_tf_children', 'exclude_tf_children'

    @staticmethod
    def process_cli_args(filters, args):
        if args.include_tf_parents or args.exclude_tf_parents or args.include_tf_children or args.exclude_tf_children:
            filters.append(Transforms(
                include_parents=args.include_tf_parents, exclude_parents=args.exclude_tf_parents,
                include_children=args.include_tf_children, exclude_children=args.exclude_tf_children))


class MergeInitialStaticTf(DeserializedMessageFilter):
    def __init__(self, delay=5.0):
        super(MergeInitialStaticTf, self).__init__(include_topics=["/tf_static"], include_types=["tf2_msgs/TFMessage"])
        self.delay = rospy.Duration(delay)
        self.end_time = None
        self.merged_message_published = False
        self.merged_transforms = {}

    def set_bag(self, bag):
        super(MergeInitialStaticTf, self).set_bag(bag)
        start_time = rospy.Time(bag.get_start_time())
        self.end_time = start_time + self.delay
        for topic, msg, stamp in bag.read_messages(
                topics=['/tf_static'], start_time=start_time, end_time=self.end_time):
            for tf in msg.transforms:
                self.merged_transforms[tf.child_frame_id] = tf

    def consider_message(self, topic, datatype, stamp, header):
        if self.end_time is None or self.end_time <= stamp:
            return False
        return super(MergeInitialStaticTf, self).consider_message(topic, datatype, stamp, header)

    def filter(self, topic, msg, stamp, header):
        if self.merged_message_published:
            return None
        self.merged_message_published = True
        return topic, TFMessage(list(self.merged_transforms.values())), stamp, header

    def reset(self):
        self.merged_message_published = False
        self.merged_transforms.clear()
        self.end_time = None
        super(MergeInitialStaticTf, self).reset()

    def _str_params(self):
        parts = ['delay=%f' % (self.delay.to_sec(),)]
        parent_params = super(MergeInitialStaticTf, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument('--merge-initial-static-tf', nargs='?', required=False, type=float, metavar="DURATION",
                            help="Merge a few initial static TFs into one. DURATION specifies the duration of the "
                                 "initial bag section to be considered for the merging. DURATION defaults to 5 secs.")

    @staticmethod
    def yaml_config_args():
        return ['merge_initial_static_tf']

    @staticmethod
    def process_cli_args(filters, args):
        if hasattr(args, 'merge_initial_static_tf') and args.merge_initial_static_tf is not None:
            if isinstance(args.merge_initial_static_tf, bool):
                if args.merge_initial_static_tf:
                    filters.append(MergeInitialStaticTf())
            else:
                filters.append(MergeInitialStaticTf(args.merge_initial_static_tf))


class FixSpotCams(RawMessageFilter):
    """Fix a problem with Spot robot cameras that publish a bit weird message header."""

    def __init__(self, *args, **kwargs):
        """
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(FixSpotCams, self).__init__(*args, **kwargs)

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        header["message_definition"] = CompressedImage._full_text
        header["md5sum"] = md5sum = CompressedImage._md5sum
        header["type"] = datatype = CompressedImage._type
        pytype = CompressedImage

        return topic, datatype, data, md5sum, pytype, stamp, header


class MaxMessageSize(RawMessageFilter):
    """Drop messages larger than the specified message size."""

    def __init__(self, size_limit, *args, **kwargs):
        """
        :param int size_limit: Maximum message size [B].
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(MaxMessageSize, self).__init__(*args, **kwargs)
        self.size_limit = size_limit

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        if len(data) > self.size_limit:
            return None

        return topic, datatype, data, md5sum, pytype, stamp, header

    def _str_params(self):
        params = ["size=%d B" % self.size_limit]
        parent_params = super(MaxMessageSize, self)._str_params()
        if len(parent_params) > 0:
            params.append(parent_params)
        return ",".join(params)

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument('--max-message-size', type=int, help='Remove all messages larger than this size [B]')

    @staticmethod
    def yaml_config_args():
        return 'max_message_size',

    @staticmethod
    def process_cli_args(filters, args):
        if args.max_message_size:
            filters.append(MaxMessageSize(size_limit=args.max_message_size))


class MakeLatched(RawMessageFilter):
    """Make topics latched."""

    def __init__(self, *args, **kwargs):
        """
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(MakeLatched, self).__init__(*args, **kwargs)

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header):
        header["latching"] = "1"
        return topic, datatype, data, md5sum, pytype, stamp, header


class CompressImages(DeserializedMessageFilter):
    """Compress an Image topic to CompressedImage."""

    def __init__(self, include_types=None, only_color=False, only_depth=False, transport=None, transport_params=None,
                 transport_mapping=None, format_mapping=None, *args, **kwargs):
        """
        :param list include_types: Types of messages to work on. The default is sensor_msgs/Image.
        :param bool only_color: If true, only color images will be processed (i.e. 3 or 4 channels, or 1 8-bit channel).
        :param bool only_depth: If true, only depth images will be processed (i.e. 1 16-bit channel).
        :param str transport: What image_transport to used. If not provided, 'compressedDepth' will be used for depth
                              images and 'compressed' for the rest.
        :param dict transport_params: Parameters of image transport(s). Keys are transport names (e.g. 'compressed'),
                                      values are the publisher dynamic reconfigure parameters.
        :param dict transport_mapping: Maps the message's 'encoding' field values to image transports. This overrides
                                       the default transport set by 'transport' arg or the autodetected one.
        :param dict format_mapping: Maps the message's 'encoding' field values encoder 'format's (i.e. 'jpg', 'png',
                                    'rvl' etc.).
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(CompressImages, self).__init__(
            include_types=include_types if include_types is not None else ['sensor_msgs/Image'], *args, **kwargs)
        self._cv = CvBridge()

        self.only_color = only_color
        self.only_depth = only_depth

        self.transport = transport
        self.transport_mapping = transport_mapping if transport_mapping is not None else {}
        self.transport_params = transport_params if transport_params is not None else {}
        # map of the raw images' encoding parameter to output image format
        self.format_mapping = format_mapping if format_mapping is not None else {}

    def filter(self, topic, msg, stamp, header):
        enc = msg.encoding
        is_color = isColor(enc) or isMono(enc) or isBayer(enc) or enc == YUV422
        is_depth = isDepth(enc)

        if (self.only_color and not is_color) or (self.only_depth and not is_depth):
            return topic, msg, stamp, header

        transport = ("compressedDepth" if is_depth else "compressed") if self.transport is None else self.transport
        transport = self.transport_mapping.get(enc, transport)

        compressed_msg, compressed_topic, err = self.get_image_for_transport(msg, topic, transport)
        # If encoding using compressedDepth fails, try with compressed
        if compressed_msg is None and transport == "compressedDepth":
            transport = "compressed"
            compressed_msg, compressed_topic, err = self.get_image_for_transport(msg, topic, transport)

        if compressed_msg is None:
            print('Error converting image: ' + str(err), file=sys.stderr)
            return topic, msg, stamp, header

        return compressed_topic, compressed_msg, stamp, header

    def get_image_for_transport(self, msg, topic, transport):
        compressed_topic = rospy.names.ns_join(topic, transport)

        config = self.transport_params.get(transport, {})
        if msg.encoding in self.format_mapping:
            config = copy.deepcopy(config)
            config["format"] = self.format_mapping[msg.encoding]
        # 16-bit images cannot be compressed to JPEG
        if transport == "compressed" and bitDepth(msg.encoding) > 8:
            config["format"] = "png"
        # Melodic doesn't have RVL
        if transport == "compressedDepth" and not has_rvl():
            config["format"] = "png"

        compressed_msg, err = encode(msg, compressed_topic, config)

        return compressed_msg, compressed_topic, err

    def _str_params(self):
        parts = []
        if self.only_color:
            parts.append('only_color')
        if self.only_depth:
            parts.append('only_depth')
        if self.transport:
            parts.append('transport=' + self.transport)
        if len(self.transport_params) > 0:
            parts.append('transport_params=%r' % (self.transport_params,))
        if len(self.transport_mapping) > 0:
            parts.append('transport_mapping=%r' % (self.transport_mapping,))
        if len(self.format_mapping) > 0:
            parts.append('format_mapping=%r' % (self.format_mapping,))
        parent_params = super(CompressImages, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class DecompressImages(DeserializedMessageFilter):
    """Decompress images on a CompressedImage topic into Image messages."""

    def __init__(self, include_types=None, desired_encodings=None, transport=None, transport_params=None,
                 *args, **kwargs):
        """
        :param list include_types: Types of messages to work on. The default is sensor_msgs/Image.
        :param dict desired_encodings: Maps topic names to target 'encoding' values of the decoded Image. By default,
                                       'passthrough' encoding is used which just passes the encoding in which the
                                       compressed image was stored.
        :param str transport: If nonempty, overrides the autodetected image_transport.
        :param dict transport_params: Parameters of image transport(s). Keys are transport names (e.g. 'compressed'),
                                      values are the subscriber dynamic reconfigure parameters.
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(DecompressImages, self).__init__(
            include_types=include_types if include_types is not None else ['sensor_msgs/CompressedImage'],
            *args, **kwargs)

        self._cv = CvBridge()

        # map from topic to desired encoding of the raw images (one of the strings in sensor_msgs/image_encodings.h)
        self.desired_encodings = desired_encodings if desired_encodings is not None else {}
        self.transport = transport
        self.transport_params = transport_params if transport_params is not None else {}

    def filter(self, topic, msg, stamp, header):
        transport = self.transport
        raw_topic = topic
        if transport is None and "/" in topic:
            raw_topic, transport = topic.rsplit("/", 1)

        params = self.transport_params.get(transport, {}) if transport is not None else {}
        raw_img, err = decode(msg, topic, params)
        if raw_img is None:
            print('Error converting image: ' + str(err), file=sys.stderr)
            return topic, msg, stamp, header

        desired_encoding = self.desired_encodings.get(topic, 'passthrough')
        if desired_encoding == 'passthrough':
            return raw_topic, raw_img, stamp, header

        compressed_fmt, compressed_depth_fmt, err = guess_any_compressed_image_transport_format(msg)
        if compressed_fmt is None and compressed_depth_fmt is None:
            print('Error converting image to desired encoding: ' + str(err), file=sys.stderr)
            return raw_topic, raw_img, stamp, header

        raw_encoding = compressed_fmt.rawEncoding if compressed_fmt is not None else compressed_depth_fmt.rawEncoding
        if desired_encoding == raw_encoding:
            return raw_topic, raw_img, stamp, header

        try:
            cv_img = self._cv.imgmsg_to_cv2(raw_img, desired_encoding)
            return raw_topic, self._cv.cv2_to_imgmsg(cv_img, desired_encoding, raw_img.header), stamp, header
        except CvBridgeError as e:
            print('Error converting image to desired encoding: ' + str(e), file=sys.stderr)
            return raw_topic, raw_img, stamp, header

    def _str_params(self):
        parts = []
        if len(self.desired_encodings):
            parts.append('desired_encodings=%r' % (self.desired_encodings,))
        if self.transport:
            parts.append('transport=' + self.transport)
        if len(self.transport_params) > 0:
            parts.append('transport_config=%r' % (self.transport_params,))
        parent_params = super(DecompressImages, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument('--decompress-images', action='store_true', help='Decompress all images')

    @staticmethod
    def yaml_config_args():
        return 'decompress_images',

    @staticmethod
    def process_cli_args(filters, args):
        if hasattr(args, 'decompress_images') and args.decompress_images:
            filters.append(DecompressImages())


class DepthImagePreview(DeserializedMessageFilter):
    """Convert 16- or 32-bit depth images to 8-bit mono or color images."""

    def __init__(self, include_types=None, new_topic_suffix="_preview", keep_orig_image=True, normalize=False,
                 num_normalization_samples=1, adaptive_normalization=True, fixed_normalization_bounds=None,
                 colormap=None, *args, **kwargs):
        """
        :param list include_types: Types of messages to work on. The default is sensor_msgs/Image.
        :param str new_new_topic_suffix: Suffix to add to the new image topic.
        :param bool keep_orig_image: Whether the original image topic should be kept or not.
        :param bool normalize: Whether to normalize the images.
        :param float num_normalization_samples: If greater than 1, this filter samples N images from the bag file to
                                                compute a global normalization factor. If lower than 1, it specifies a
                                                percentage of images to take as samples. If equal to 1, each image is
                                                normalized independently and random access to the bag file is not
                                                needed.
        :param bool adaptive_normalization: If num_normalization_samples is 1 (independent normalization), collect
                                            info about normalization range from the so-far seen samples. This means
                                            the normalization range is changing, but it is only growing, never
                                            decreasing.
        :param tuple fixed_normalization_bounds: If not None, normalization is not estimated and is not adapted, but
                                                 these bounds are used all the time. For 16UC1 images, these bounds
                                                 are multiplied by 1000 to comply with the Kinect standards (conversion
                                                 to millimeters). The tuple should contain exactly 2 float elements
                                                 meaning min value and max value.
        :param str colormap: If None, the conversion will be into 8-bit grayscale. If not None, the passed string should
                             name an installed matplotlib colormap (e.g. 'jet') to use and the resulting images will be
                             3-channel BGR8 images.
        :param args: Standard include/exclude topics/types and min/max stamp args.
        :param kwargs: Standard include/exclude topics/types and min/max stamp kwargs.
        """
        super(DepthImagePreview, self).__init__(
            include_types=include_types if include_types is not None else ['sensor_msgs/Image'],
            *args, **kwargs)

        self._cv = CvBridge()

        self.new_topic_suffix = new_topic_suffix
        self.keep_orig_image = keep_orig_image
        self.normalize = normalize
        self.num_normalization_samples = num_normalization_samples
        self.adaptive_normalization = adaptive_normalization
        self.fixed_normalization_bounds = fixed_normalization_bounds
        if num_normalization_samples != 1 or fixed_normalization_bounds is not None:
            self.adaptive_normalization = False
        self.colormap = None
        if colormap is not None:
            try:
                self.colormap = cmap.get_cmap(colormap)
            except KeyError:
                self.colormap = cmap.get_cmap(None)
                print('Colormap {} not found, using default colormap.'.format(colormap), file=sys.stderr)
        self.min_values = {}
        self.max_values = {}

    def set_bag(self, bag):
        super(DepthImagePreview, self).set_bag(bag)

        if self.num_normalization_samples == 1 or self.fixed_normalization_bounds is not None:
            return

        def connection_filter(topic, datatype, md5sum, msg_def, header):
            return datatype == "sensor_msgs/Image"

        conns = bag._get_connections(self._include_topics, connection_filter)
        image_topics = set([c.topic for c in conns])
        depth_image_topics = []

        for topic in image_topics:
            for _, msg, stamp in bag.read_messages([topic]):
                if msg.encoding == TYPE_16UC1 or msg.encoding == TYPE_32FC1:
                    depth_image_topics.append(topic)
                break

        _, topic_info = bag.get_type_and_topic_info(depth_image_topics)
        for topic in depth_image_topics:
            info = topic_info[topic]
            num_messages = info.message_count
            num_samples = int(self.num_normalization_samples) if self.num_normalization_samples > 1 else \
                int(self.num_normalization_samples * num_messages)
            num_samples = min(num_samples, num_messages)
            connections = bag._get_connections([topic], self.connection_filter)
            entries = list(bag._get_entries(connections, self._min_stamp, self._max_stamp))
            if len(entries) < num_samples:
                entries = list(bag._get_entries(connections))
            idx = np.round(np.linspace(0, len(entries) - 1, num_samples)).astype(int)
            for i in idx:
                entry = entries[i]
                _, msg, _ = bag._read_message((entry.chunk_pos, entry.offset))
                msg_data = self._cv.imgmsg_to_cv2(msg)
                if msg.encoding == TYPE_16UC1:
                    msg_data = msg_data[msg_data != 0]
                    msg_data = msg_data[msg_data != 65535]
                else:
                    msg_data = msg_data[np.isfinite(msg_data)]
                if topic not in self.min_values:
                    self.min_values[topic] = np.min(msg_data)
                    self.max_values[topic] = np.max(msg_data)
                else:
                    self.min_values[topic] = min(self.min_values[topic], np.min(msg_data))
                    self.max_values[topic] = max(self.max_values[topic], np.max(msg_data))

    def filter(self, topic, msg, stamp, header):
        if msg.encoding != TYPE_16UC1 and msg.encoding != TYPE_32FC1:
            # If we encounter a non-depth topic, exclude it from this filter. So only the first message from non-depth
            # topics will be needlessly deserialized because of this filter.
            self._exclude_topics = TopicSet(list(self._exclude_topics) + [topic])
            return topic, msg, stamp, header

        in_bit_depth = 16 if msg.encoding == TYPE_16UC1 else 32
        new_bit_depth = 8

        try:
            img = self._cv.imgmsg_to_cv2(msg)
            orig_img = img

            if self.normalize or msg.encoding == TYPE_32FC1:
                if msg.encoding == TYPE_16UC1:
                    img_valid = img[(img != 0) & (img != 65535)]
                else:
                    img_valid = img[np.isfinite(img)]

                if self.fixed_normalization_bounds is not None:
                    if topic not in self.min_values:
                        min_bound, max_bound = self.fixed_normalization_bounds
                        if msg.encoding == TYPE_16UC1:
                            min_bound = int(min_bound * 1000)
                            max_bound = int(max_bound * 1000)
                        self.min_values[topic] = min_bound
                        self.max_values[topic] = max_bound

                elif self.adaptive_normalization:
                    im_min = np.min(img_valid)
                    im_max = np.max(img_valid)
                    if topic not in self.min_values:
                        self.min_values[topic] = im_min
                        self.max_values[topic] = im_max
                    else:
                        self.min_values[topic] = min(self.min_values[topic], im_min)
                        self.max_values[topic] = max(self.max_values[topic], im_max)

                im_min = self.min_values[topic] if topic in self.min_values else np.min(img_valid)
                im_max = self.max_values[topic] if topic in self.max_values else np.max(img_valid)
                im_range = max(im_max - im_min, 1)
                img = (img - im_min) / float(im_range)
                img = np.clip(img, 0.0, 1.0)

                if msg.encoding == TYPE_16UC1:
                    img[orig_img == 0] = 0.0
                    img[orig_img == 65535] = 1.0
                else:
                    img[np.isinf(orig_img)] = ~np.signbit(orig_img[np.isinf(orig_img)])
                    img[np.isnan(orig_img)] = 0.0
            else:
                img = img / (np.power(2.0, in_bit_depth) - 1)

            if self.colormap is None:
                img = img * (np.power(2.0, new_bit_depth) - 1)
                img = img.astype(np.uint8)
                desired_encoding = MONO8
            else:
                img = self.colormap(img, bytes=True)
                if self.colormap.is_gray():
                    img = img[:, :, 0]
                    desired_encoding = MONO8
                else:
                    img = img[:, :, (2, 1, 0)]
                    desired_encoding = BGR8

            new_topic = topic + self.new_topic_suffix
            new_msg = self._cv.cv2_to_imgmsg(img, desired_encoding, msg.header)
            new_image = new_topic, new_msg, stamp, header
            if not self.keep_orig_image:
                return new_image
            return [(topic, msg, stamp, header), new_image]
        except CvBridgeError as e:
            print('Error converting image to desired encoding: ' + str(e), file=sys.stderr)
            return topic, msg, stamp, header

    def reset(self):
        self.min_values.clear()
        self.max_values.clear()
        super(DepthImagePreview, self).reset()

    def _str_params(self):
        parts = []
        parts.append('new_topic_suffix=%s' % (self.new_topic_suffix,))
        parts.append('keep_orig_image=%r' % (self.keep_orig_image,))
        parts.append('normalize=%r' % (self.normalize,))
        if self.adaptive_normalization:
            parts.append('adaptive_normalization')
        if self.num_normalization_samples != 1:
            parts.append('num_normalization_samples=%r' % (self.num_normalization_samples,))
        if self.fixed_normalization_bounds is not None:
            parts.append('fixed_normalization_bounds=%r' % (self.fixed_normalization_bounds,))
        if self.colormap is not None:
            parts.append('colormap=%r' % (self.colormap,))
        parent_params = super(DepthImagePreview, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)
