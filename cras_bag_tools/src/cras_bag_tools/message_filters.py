# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Implementations of common message filters."""

from __future__ import absolute_import, division, print_function

import copy
import math
import os.path

import matplotlib.cm as cmap
import numpy as np
import sys
import yaml
from collections import deque
from enum import Enum
from typing import Dict, List, Optional, Sequence, Set, Tuple, Union
from lxml import etree
from numpy.linalg import inv
from typing import Any, Dict

import rospy
from angles import normalize_angle, normalize_angle_positive
from camera_calibration_parsers import readCalibration
from cras.distortion_models import PLUMB_BOB, RATIONAL_POLYNOMIAL, EQUIDISTANT
from cras.geometry_utils import quat_msg_from_rpy
from cras.image_encodings import isColor, isMono, isBayer, isDepth, bitDepth, numChannels, MONO8, RGB8, BGR8,\
    TYPE_16UC1, TYPE_32FC1, YUV422
from cras.log_utils import rosconsole_notifyLoggerLevelsChanged, rosconsole_set_logger_level, RosconsoleLevel
from cras.string_utils import to_str, STRING_TYPE
import cv2  # Workaround for https://github.com/opencv/opencv/issues/14884 on Jetsons.
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.encoding import decode_config
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Twist, TwistStamped, Vector3
from image_transport_codecs import decode, encode
from image_transport_codecs.compressed_depth_codec import has_rvl
from image_transport_codecs.parse_compressed_format import guess_any_compressed_image_transport_format
from kdl_parser_py.urdf import treeFromUrdfModel
from nav_msgs.msg import Odometry
from ros_numpy import msgify, numpify
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from tf2_py import BufferCore
from urdf_parser_py import urdf, xml_reflection
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from .message_filter import DeserializedMessageFilter, DeserializedMessageFilterWithTF, MessageTags, NoMessageFilter, \
    RawMessageFilter, TopicSet, tags_for_generated_msg


def urdf_error(message):
    if "selfCollide" in message:
        return
    print(message, file=sys.stderr)


xml_reflection.core.on_error = urdf_error


def dict_to_str(d, sep='='):
    return '{' + ', '.join('%s%s%s' % (
        k, sep, str(v) if not isinstance(v, dict) else dict_to_str(v, sep)) for k, v in d.items()) + "}"


def create_connection_header(topic, msg_type, latch=False):
    header = {
        "callerid": "/bag_filter",
        "topic": topic,
        "message_definition": msg_type._full_text,  # noqa
        "type": msg_type._type,  # noqa
        "md5sum": msg_type._md5sum,  # noqa
    }
    if latch:
        header["latching"] = "1"
    return header


class SetFields(DeserializedMessageFilter):
    """Change values of some fields of a message (pass the fields to change as kwargs)."""

    def __init__(self, include_topics=None, exclude_topics=None, include_types=None, exclude_types=None,
                 min_stamp=None, max_stamp=None, include_time_ranges=None, exclude_time_ranges=None,
                 include_tags=None, exclude_tags=None, **kwargs):
        """
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
        :param list include_tags: If nonempty, the filter will only work on messages with these tags.
        :param list exclude_tags: If nonempty, the filter will skip messages with these tags.
        :param dict kwargs: The fields to set. Keys are field name, values are the new values to set.
        """
        super(SetFields, self).__init__(
            include_topics, exclude_topics, include_types, exclude_types, min_stamp, max_stamp,
            include_time_ranges, exclude_time_ranges, include_tags, exclude_tags)
        self.field_values = kwargs

    def filter(self, topic, msg, stamp, header, tags):
        for k, v in self.field_values.items():
            if k not in msg.__slots__:
                continue
            setattr(msg, k, v)
            tags.add(MessageTags.CHANGED)
        return topic, msg, stamp, header, tags

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
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(FixHeader, self).__init__(*args, **kwargs)
        self.frame_id = frame_id
        self.frame_id_prefix = frame_id_prefix
        self.frame_id_suffix = frame_id_suffix
        self.stamp_from_receive_time = stamp_from_receive_time
        self.stamp_offset = rospy.Duration(stamp_offset)

    def filter(self, topic, msg, stamp, header, tags):
        if len(msg.__slots__) > 0 and msg.__slots__[0] == 'header':
            self.fix_header(msg.header, stamp)
            tags.add(MessageTags.CHANGED)
        # Support for TF, Path and similar array-only messages
        elif len(msg.__slots__) == 1 and msg._get_types()[0].endswith("[]"):
            for m in getattr(msg, msg.__slots__[0]):
                if len(m.__slots__) > 0 and m.__slots__[0] == 'header':
                    self.fix_header(m.header, stamp)
                    tags.add(MessageTags.CHANGED)

        return topic, msg, stamp, header, tags

    def fix_header(self, header, stamp):
        if len(self.frame_id) > 0:
            header.frame_id = self.frame_id
        else:
            if len(self.frame_id_prefix) > 0:
                header.frame_id = self.frame_id_prefix + header.frame_id
            if len(self.frame_id_suffix) > 0:
                header.frame_id += self.frame_id_suffix

        if self.stamp_from_receive_time:
            header.stamp = stamp
        header.stamp += self.stamp_offset

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
            parts.append('stamp_offset=' + str(self.stamp_offset.to_sec()))
        parent_params = super(FixHeader, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class Deduplicate(RawMessageFilter):
    """Discard all messages except each first changed."""

    def __init__(self, ignore_seq=False, ignore_stamp=False, ignore_stamp_difference=None, per_frame_id=False,
                 max_ignored_duration=None, *args, **kwargs):
        """
        :param bool ignore_seq: If True, differing header.seq will not make a difference.
        :param bool ignore_stamp: If True, differing header.stamp will not make a difference.
        :param float ignore_stamp_difference: If set, stamps differing by up to this duration are considered equal.
        :param bool per_frame_id: If True, messages will be clustered by header.frame_id and comparisons will only be
                                  made between messages with equal frame_id.
        :param float max_ignored_duration: If set, a duplicate will pass if its stamp is further from the last passed
                                           message than the given duration (in seconds).
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(Deduplicate, self).__init__(*args, **kwargs)
        self._ignore_seq = ignore_seq
        self._ignore_stamp = ignore_stamp
        self._ignore_stamp_difference = \
            rospy.Duration(ignore_stamp_difference) if ignore_stamp_difference is not None else None
        self._per_frame_id = per_frame_id
        self._max_ignored_duration = rospy.Duration(max_ignored_duration) if max_ignored_duration is not None else None
        self._last_msgs = {}

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        has_header = pytype.__slots__[0] == 'header'
        key = topic
        msg_header = None
        if has_header and self._per_frame_id:
            msg_header = Header().deserialize(data)
            key = "%s@%s" % (topic, msg_header.frame_id)

        if key not in self._last_msgs:
            self._last_msgs[key] = data, stamp
            return topic, datatype, data, md5sum, pytype, stamp, header, tags

        last_msg, last_msg_stamp = self._last_msgs[key]

        seq_ok = True
        stamp_ok = True
        stamp_diff_ok = True
        compare_idx = 0

        if has_header:
            seq_ok = self._ignore_seq or data[0:4] == last_msg[0:4]
            stamp_ok = self._ignore_stamp or data[4:12] == last_msg[4:12]
            if not self._ignore_stamp and self._ignore_stamp_difference is not None and not stamp_ok:
                if msg_header is None:
                    msg_header = Header().deserialize(data)
                last_msg_header = Header().deserialize(last_msg)
                diff = abs(msg_header.stamp - last_msg_header.stamp)
                if diff < self._ignore_stamp_difference:
                    stamp_ok = True

            compare_idx = 12
        if self._max_ignored_duration is not None:
            stamp_diff_ok = stamp - last_msg_stamp < self._max_ignored_duration

        if seq_ok and stamp_ok and stamp_diff_ok and data[compare_idx:] == last_msg[compare_idx:]:
            return None

        self._last_msgs[key] = data, stamp

        return topic, datatype, data, md5sum, pytype, stamp, header, tags

    def _str_params(self):
        parts = []
        parts.append('ignore_seq=%r' % (self._ignore_seq,))
        parts.append('ignore_stamp=%r' % (self._ignore_stamp,))
        if self._ignore_stamp_difference is not None:
            parts.append('ignore_stamp_difference=%s' % (to_str(self._ignore_stamp_difference),))
        parts.append('per_frame_id=%r' % (self._per_frame_id,))
        parts.append('max_ignored_duration=%r' % (self._max_ignored_duration,))
        parent_params = super(Deduplicate, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class DeduplicateJointStates(DeserializedMessageFilter):
    """Discard all messages except each first changed."""

    def __init__(self, max_ignored_duration=None, include_topics=None, ignore_stamp=False, ignore_stamp_difference=None,
                 *args, **kwargs):
        """
        :param float max_ignored_duration: If set, a duplicate will pass if its stamp is further from the last passed
                                           joint state than the given duration (in seconds).
        :param list include_topics: Topics to work on (defaults to 'joint_states').
        :param bool ignore_stamp: If True, differing header.stamp will not make a difference.
        :param float ignore_stamp_difference: If set, stamps differing by up to this duration are considered equal.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(DeduplicateJointStates, self).__init__(
            include_topics=['joint_states'] if include_topics is None else include_topics,
            include_types=[JointState._type], *args, **kwargs)  # noqa
        self._ignore_stamp = ignore_stamp
        self._ignore_stamp_difference = \
            rospy.Duration(ignore_stamp_difference) if ignore_stamp_difference is not None else None
        self._max_ignored_duration = rospy.Duration(max_ignored_duration) if max_ignored_duration is not None else None

        self._last_states = {}

    def filter(self, topic, msg, stamp, header, tags):
        if len(msg.name) == 0:
            return topic, msg, stamp, header, tags

        for i in reversed(range(len(msg.name))):
            name = msg.name[i]

            if name not in self._last_states:
                self._last_states[name] = self._get_state(msg, i, stamp)
                continue

            last_state = self._last_states[name]

            stamp_ok = msg.header.stamp == last_state[0]
            if not stamp_ok and not self._ignore_stamp and self._ignore_stamp_difference is not None:
                diff = abs(msg.header.stamp - last_state[0])
                if diff < self._ignore_stamp_difference:
                    stamp_ok = True

            stamp_diff_ok = True
            if self._max_ignored_duration is not None:
                stamp_diff_ok = stamp - last_state[5] < self._max_ignored_duration

            state = self._get_state(msg, i, stamp)
            if stamp_ok and stamp_diff_ok:
                if np.allclose(state[2:5], last_state[2:5], equal_nan=True):
                    if isinstance(msg.name, tuple):
                        msg.name = list(msg.name)
                    del msg.name[i]
                    if len(msg.position) > i:
                        if isinstance(msg.position, tuple):
                            msg.position = list(msg.position)
                        del msg.position[i]
                    if len(msg.velocity) > i:
                        if isinstance(msg.velocity, tuple):
                            msg.velocity = list(msg.velocity)
                        del msg.velocity[i]
                    if len(msg.effort) > i:
                        if isinstance(msg.effort, tuple):
                            msg.effort = list(msg.effort)
                        del msg.effort[i]
            else:
                self._last_states[name] = state

        if len(msg.name) == 0:
            return None

        return topic, msg, stamp, header, tags

    def _get_state(self, msg, i, stamp):
        return (
            msg.header.stamp,
            msg.name[i],
            msg.position[i] if len(msg.position) > i else None,
            msg.velocity[i] if len(msg.velocity) > i else None,
            msg.effort[i] if len(msg.effort) > i else None,
            stamp,
        )

    def _str_params(self):
        parts = []
        parts.append('ignore_stamp=%r' % (self._ignore_stamp,))
        if self._ignore_stamp_difference is not None:
            parts.append('ignore_stamp_difference=%s' % (to_str(self._ignore_stamp_difference),))
        parts.append('max_ignored_duration=%r' % (self._max_ignored_duration,))
        parent_params = super(DeduplicateJointStates, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class DeduplicateTF(DeserializedMessageFilter):
    """Discard all messages except each first changed."""

    def __init__(self, max_ignored_duration=None, include_topics=None, *args, **kwargs):
        """
        :param float max_ignored_duration: If set, a duplicate will pass if its stamp is further from the last passed
                                           message than the given duration (in seconds).
        :param list include_topics: Topics to work on (defaults to standard TF topics).
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(DeduplicateTF, self).__init__(
            include_topics=['tf', 'tf_static'] if include_topics is None else include_topics,
            include_types=[TFMessage._type], *args, **kwargs)  # noqa
        self._max_ignored_duration = rospy.Duration(max_ignored_duration) if max_ignored_duration is not None else None
        self._last_msgs = {}

    def filter(self, topic, msg, stamp, header, tags):
        tfs = msg.transforms
        if len(tfs) == 0:
            return topic, msg, stamp, header, tags

        for i in reversed(range(len(tfs))):
            key = "%s@%s@%s" % (topic, tfs[i].header.frame_id, tfs[i].child_frame_id)

            if key not in self._last_msgs:
                self._last_msgs[key] = tfs[i], stamp, copy.deepcopy(tags)
                continue

            last_msg, last_msg_stamp, last_tags = self._last_msgs[key]

            stamp_ok = tfs[i].header.stamp == last_msg.header.stamp

            stamp_diff_ok = True
            if self._max_ignored_duration is not None:
                stamp_diff_ok = stamp - last_msg_stamp < self._max_ignored_duration

            if stamp_ok and stamp_diff_ok:
                del tfs[i]
            else:
                self._last_msgs[key] = tfs[i], stamp, copy.deepcopy(tags)

        if len(tfs) == 0:
            return None

        return topic, msg, stamp, header, tags

    def _str_params(self):
        parts = []
        parts.append('max_ignored_duration=%r' % (self._max_ignored_duration,))
        parent_params = super(DeduplicateTF, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class Remap(RawMessageFilter):
    """Remap topics."""

    def __init__(self, min_stamp=None, max_stamp=None, include_time_ranges=None, exclude_time_ranges=None,
                 include_tags=None, exclude_tags=None, **kwargs):
        """
        :param dict kwargs: The mapping to use. Keys are topics to be remapped, values are their new names.
        """
        super(Remap, self).__init__(kwargs.keys(), None, None, None, min_stamp, max_stamp,
                                    include_time_ranges, exclude_time_ranges, include_tags, exclude_tags)
        self.remap = kwargs

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        return self.remap.get(topic, topic), datatype, data, md5sum, pytype, stamp, header, tags

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


class Copy(RawMessageFilter):
    """Copy topics to other topics."""

    def __init__(self, min_stamp=None, max_stamp=None, include_time_ranges=None, exclude_time_ranges=None,
                 include_tags=None, exclude_tags=None, add_tags=None, **kwargs):
        """
        :param set add_tags: If set, these tags will be added to the copies of messages.
        :param dict kwargs: The mapping to use. Keys are topics to be copied, values are their new names.
        """
        super(Copy, self).__init__(kwargs.keys(), None, None, None, min_stamp, max_stamp,
                                   include_time_ranges, exclude_time_ranges, include_tags, exclude_tags)
        self._add_tags = add_tags
        self.copy = kwargs

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        copy_tags = tags_for_generated_msg(tags)
        if self._add_tags:
            copy_tags = copy_tags.union(self._add_tags)
        return [
            (topic, datatype, data, md5sum, pytype, stamp, header, tags),
            (self.copy.get(topic, topic), datatype, data, md5sum, pytype, stamp, header, copy_tags),
        ]

    def _str_params(self):
        return dict_to_str(self.copy, '=>')

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument(
            '--copy', nargs='+', metavar="FROM TO",
            help="Copy topics. This argument should be an even-sized list of pairs [FROM TO].")

    @staticmethod
    def process_cli_args(filters, args):
        if args.remap:
            topics_from = args.remap[0::2]
            topics_to = args.remap[1::2]
            filters.append(Copy(**dict(zip(topics_from, topics_to))))


class Throttle(RawMessageFilter):
    """Throttle messages on topics."""

    def __init__(self, min_stamp=None, max_stamp=None, include_time_ranges=None, exclude_time_ranges=None,
                 include_tags=None, exclude_tags=None, **kwargs):
        """
        :param kwargs: Keys are topics to be throttled, values are their maximum frequencies.
        :type kwargs: Dict[str, float]
        """
        super(Throttle, self).__init__(kwargs.keys(), None, None, None, min_stamp, max_stamp,
                                       include_time_ranges, exclude_time_ranges, include_tags, exclude_tags)
        self.hz = kwargs
        self.prev_t = {}

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        t = stamp.to_sec()
        if topic in self.prev_t:
            period = t - self.prev_t[topic]
            if period <= 0.:
                return None
            hz = 1. / period
            if hz > self.hz[topic]:
                return None
        self.prev_t[topic] = t
        return topic, datatype, data, md5sum, pytype, stamp, header, tags

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

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        # The filtering is done in topic_filter()
        return topic, datatype, data, md5sum, pytype, stamp, header, tags

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

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        # The filtering is done in connection_filter()
        return topic, datatype, data, md5sum, pytype, stamp, header, tags

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

    def consider_message(self, topic, datatype, stamp, header, tags):
        # Drop also messages from extra time ranges
        return super(Drop, self).consider_message(topic, datatype, stamp, header, tags)

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
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
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
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

    def filter(self, topic, msg, stamp, header, tags):
        num_tfs = len(msg.transforms)
        if self.include_parents:
            msg.transforms = [tf for tf in msg.transforms if tf.header.frame_id in self.include_parents]
            if len(msg.transforms) < num_tfs:
                tags.add(MessageTags.CHANGED)
                num_tfs = len(msg.transforms)
        if self.exclude_parents:
            msg.transforms = [tf for tf in msg.transforms if tf.header.frame_id not in self.exclude_parents]
            if len(msg.transforms) < num_tfs:
                tags.add(MessageTags.CHANGED)
                num_tfs = len(msg.transforms)
        if self.include_children:
            msg.transforms = [tf for tf in msg.transforms if tf.child_frame_id in self.include_children]
            if len(msg.transforms) < num_tfs:
                tags.add(MessageTags.CHANGED)
                num_tfs = len(msg.transforms)
        if self.exclude_children:
            msg.transforms = [tf for tf in msg.transforms if tf.child_frame_id not in self.exclude_children]
            if len(msg.transforms) < num_tfs:
                tags.add(MessageTags.CHANGED)
                num_tfs = len(msg.transforms)
        if len(self.change) > 0:
            for transform in msg.transforms:
                if transform.header.frame_id in self.changed_parents:
                    if transform.child_frame_id in self.change[transform.header.frame_id]:
                        changes = self.change[transform.header.frame_id][transform.child_frame_id]
                        if 'translation' in changes:
                            transform.translation.x = float(changes['translation'].get('x', transform.translation.x))
                            transform.translation.y = float(changes['translation'].get('y', transform.translation.y))
                            transform.translation.z = float(changes['translation'].get('z', transform.translation.z))
                            tags.add(MessageTags.CHANGED)
                        if 'rotation' in changes:
                            transform.rotation.x = float(changes['rotation'].get('x', transform.rotation.x))
                            transform.rotation.y = float(changes['rotation'].get('y', transform.rotation.y))
                            transform.rotation.z = float(changes['rotation'].get('z', transform.rotation.z))
                            transform.rotation.w = float(changes['rotation'].get('w', transform.rotation.w))
                            tags.add(MessageTags.CHANGED)
                        if 'frame_id' in changes:
                            transform.header.frame_id = changes['frame_id']
                            tags.add(MessageTags.CHANGED)
                        if 'child_frame_id' in changes:
                            transform.child_frame_id = changes['child_frame_id']
                            tags.add(MessageTags.CHANGED)
        if not msg.transforms:
            return None
        return topic, msg, stamp, header, tags

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
            parts.append('change=' + dict_to_str(self.change))
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
    """Merge all /tf_static messages from the beginning of bag files into a single message."""

    def __init__(self, delay=5.0, add_tags=None):
        super(MergeInitialStaticTf, self).__init__(include_topics=["/tf_static"], include_types=["tf2_msgs/TFMessage"])
        self.delay = rospy.Duration(delay)
        self.end_time = None
        self.merged_message_published = False
        self.merged_transforms = {}
        self.add_tags = add_tags

    def set_bag(self, bag):
        super(MergeInitialStaticTf, self).set_bag(bag)
        start_time = rospy.Time(bag.get_start_time())
        self.end_time = start_time + self.delay
        for topic, msg, stamp in bag.read_messages(
                topics=['/tf_static'], start_time=start_time, end_time=self.end_time):
            for tf in msg.transforms:
                self.merged_transforms[tf.child_frame_id] = tf

    def consider_message(self, topic, datatype, stamp, header, tags):
        if self.end_time is None or self.end_time <= stamp:
            return False
        return super(MergeInitialStaticTf, self).consider_message(topic, datatype, stamp, header, tags)

    def filter(self, topic, msg, stamp, header, tags):
        if self.merged_message_published:
            return None
        self.merged_message_published = True
        merged_tags = tags_for_generated_msg(tags)
        if self.add_tags:
            merged_tags = merged_tags.union(self.add_tags)
        return topic, TFMessage(list(self.merged_transforms.values())), stamp, header, merged_tags

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


def set_transform_from_KDL_frame(transform, frame):
    translation = frame.p
    rotation = frame.M.GetQuaternion()
    transform.translation.x = translation.x()
    transform.translation.y = translation.y()
    transform.translation.z = translation.z()
    transform.rotation.x = rotation[0]
    transform.rotation.y = rotation[1]
    transform.rotation.z = rotation[2]
    transform.rotation.w = rotation[3]


class RecomputeTFFromJointStates(DeserializedMessageFilter):
    """Recompute some TFs from the URDF model and joint states."""

    def __init__(self, include_topics=None, description_param=None, description_file=None, joint_state_cache_size=100,
                 include_parents=(), exclude_parents=(), include_children=(), exclude_children=(), include_joints=(),
                 exclude_joints=(), publish_new_tfs=False, add_tags=None, *args, **kwargs):
        """
        :param list include_topics: Topics to handle. It should contain both the JointState and TF topics. If no TF
                                    topic is given, /tf and /tf_static are added automatically.
        :param str description_param: Name of the ROS parameter that hold the URDF model.
        :param str description_file: Path to a file with the URDF model (has precedence over description_param).
        :param joint_state_cache_size: Number of JointState messages that will be cached to be searchable when a TF
                                       message comes and needs to figure out the JointState message it was created
                                       from.
        :param list include_parents: If nonempty, only TFs with one of the listed frames as parent will be processed.
        :param list exclude_parents: If nonempty, TFs with one of the listed frames as parent will not be processed.
        :param list include_children: If nonempty, only TFs with one of the listed frames as child will be processed.
        :param list exclude_children: If nonempty, TFs with one of the listed frames as child will not be processed.
        :param list include_joints: If nonempty, only joints with the listed named will be processed.
        :param list exclude_joints: If nonempty, joints with one of the listed names will not be processed.
        :param list publish_new_tfs: If true, the mode of operation is changed so that every joint_states message
                                     will create a new TF message for the contained joints (i.e. new messages are added
                                     to the bag).
        :param set add_tags: Add these tags to all modified TF messages.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        if include_topics is None:
            include_topics = ("/joint_states",)
        has_tf = False
        for topic in include_topics:
            if "tf" in topic:
                has_tf = True
                break
        if not has_tf:
            include_topics = list(include_topics)
            include_topics.append("/tf")
            include_topics.append("/tf_static")

        super(RecomputeTFFromJointStates, self).__init__(
            include_topics=include_topics, include_types=(TFMessage._type, JointState._type), *args, **kwargs)  # noqa

        self._urdf_model = None
        self._kdl_model = None
        self._mimic_joints = {}
        self._segments = {}
        self._segments_fixed = {}
        self._child_to_joint_map = {}
        self._joint_to_child_map = {}
        self._joint_state_cache = deque(maxlen=joint_state_cache_size)
        self._add_tags = add_tags

        self.include_parents = TopicSet(include_parents)
        self.exclude_parents = TopicSet(exclude_parents)
        self.include_children = TopicSet(include_children)
        self.exclude_children = TopicSet(exclude_children)
        self.include_joints = TopicSet(include_joints)
        self.exclude_joints = TopicSet(exclude_joints)
        self.publish_new_tfs = publish_new_tfs

        self.description_param = description_param

        self._model_from_file = None
        if description_file is not None:
            if os.path.exists(description_file):
                try:
                    with open(description_file, "r") as f:
                        self._model_from_file = f.read()
                    print("Loaded URDF model from file " + description_file)
                except Exception as e:
                    print('Could not read URDF model from file %s: %s' % (description_file, str(e)), file=sys.stderr)
            else:
                print('URDF model file "%s" does not exist.' % (description_file,), file=sys.stderr)

    def set_bag(self, bag):
        super(RecomputeTFFromJointStates, self).set_bag(bag)

        model = self._model_from_file
        if model is None:
            if self.description_param is None:
                self.description_param = "robot_description"
            model = self._get_param(self.description_param)
            if model is not None:
                print("Read URDF model from ROS parameter " + self.description_param)

        if model is None:
            raise RuntimeError("RecomputeTFFromJointStates requires robot URDF either as a file or ROS parameter.")

        self._urdf_model = urdf.URDF.from_xml_string(model)
        ok, self._kdl_model = treeFromUrdfModel(self._urdf_model, quiet=True)
        if not ok:
            self._kdl_model = None
            print('Could not parse URDF model into KDL model.', file=sys.stderr)

        self._mimic_joints = {}
        for joint_name, joint in self._urdf_model.joint_map.items():
            if joint.mimic is not None:
                self._mimic_joints[joint_name] = joint.mimic

        self._segments = {}
        self._segments_fixed = {}
        self._child_to_joint_map = {}
        self._joint_to_child_map = {}
        for child, (joint_name, parent) in self._urdf_model.parent_map.items():
            if self.include_joints and joint_name not in self.include_joints:
                continue
            if self.exclude_joints and joint_name in self.exclude_joints:
                continue
            segment = self._kdl_model.getChain(parent, child).getSegment(0)
            joint = segment.getJoint()
            self._child_to_joint_map[child] = joint_name
            self._joint_to_child_map[joint_name] = child
            if joint.getTypeName() == "None":
                self._segments_fixed[child] = segment
            else:
                self._segments[child] = segment

    def filter(self, topic, msg, stamp, header, tags):
        if self._kdl_model is None:
            return topic, msg, stamp, header, tags

        if msg._type == TFMessage._type:  # noqa
            for transform in msg.transforms:
                if self.include_parents and transform.header.frame_id not in self.include_parents:
                    continue
                if self.exclude_parents and transform.header.frame_id in self.exclude_parents:
                    continue
                if self.include_children and transform.child_frame_id not in self.include_children:
                    continue
                if self.exclude_children and transform.child_frame_id in self.exclude_children:
                    continue

                is_dynamic = transform.child_frame_id in self._segments
                is_static = transform.child_frame_id in self._segments_fixed
                if not is_static and not is_dynamic:
                    continue  # not a TF from the robot model

                if is_static:
                    success = self._recompute_static_transform(transform)
                else:
                    success = self._recompute_dynamic_transform(transform)
                if not success:
                    print("Could not recompute transform %s -> %s at time %s." % (
                        transform.header.frame_id, transform.child_frame_id, to_str(stamp)), file=sys.stderr)
                else:
                    tags.add(MessageTags.CHANGED)
                    if self._add_tags:
                        tags = tags.union(self._add_tags)
            return topic, msg, stamp, header, tags

        else:  # the message is a joint state
            self._joint_state_cache.append(copy.deepcopy(msg))
            result = [(topic, msg, stamp, header, tags)]

            if self.publish_new_tfs:
                tf_msg = TFMessage()
                tf_static_msg = TFMessage()
                for i in range(len(msg.name)):
                    joint_name = msg.name[i]
                    if len(msg.position) <= i or not math.isfinite(msg.position[i]):
                        continue
                    if joint_name not in self._joint_to_child_map:
                        continue
                    child_name = self._joint_to_child_map[joint_name]
                    _, parent_name = self._urdf_model.parent_map[child_name]
                    t = TransformStamped()
                    t.header.frame_id = parent_name
                    t.header.stamp = msg.header.stamp
                    t.child_frame_id = child_name
                    if child_name in self._segments:
                        tf_msg.transforms.append(t)
                    else:
                        tf_static_msg.transforms.append(t)
                    # Append just zero-initialized TF; it will be passed to this filter later and it will get recomputed

                tf_tags = tags_for_generated_msg(tags)
                if self._add_tags:
                    tf_tags = tf_tags.union(self._add_tags)
                if len(tf_msg.transforms) > 0:
                    connection_header = create_connection_header("/tf", TFMessage, False)
                    result.append(("/tf", tf_msg, stamp, connection_header, tf_tags))
                if len(tf_static_msg.transforms) > 0:
                    connection_header = create_connection_header("/tf_static", TFMessage, True)
                    result.append(("/tf_static", tf_msg, stamp, connection_header, tf_tags))

            return result

    def _recompute_static_transform(self, transform):
        if transform.child_frame_id not in self._segments_fixed:
            return False
        segment = self._segments_fixed[transform.child_frame_id]
        frame = segment.pose(0.0)
        set_transform_from_KDL_frame(transform.transform, frame)
        return True

    def _recompute_dynamic_transform(self, transform):
        if transform.child_frame_id not in self._child_to_joint_map or transform.child_frame_id not in self._segments:
            return False
        if transform.child_frame_id not in self._segments:
            return False

        stamp = transform.header.stamp
        joint_name = self._child_to_joint_map[transform.child_frame_id]
        mimic = None
        if joint_name in self._mimic_joints:
            mimic = self._mimic_joints[joint_name]
            joint_name = mimic.joint
        for msg in reversed(self._joint_state_cache):
            if msg.header.stamp != stamp:
                continue
            if len(msg.position) == 0:
                continue
            for i in range(len(msg.name)):
                if len(msg.position) < i - 1:
                    break
                if msg.name[i] == joint_name:
                    pos = msg.position[i]
                    if mimic is not None:
                        pos = pos * float(mimic.multiplier) + float(mimic.offset)
                    segment = self._segments[transform.child_frame_id]
                    frame = segment.pose(pos)
                    set_transform_from_KDL_frame(transform.transform, frame)
                    return True
        return False

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
        if self.include_joints:
            parts.append('include_joints=' + str(self.include_joints))
        if self.exclude_joints:
            parts.append('exclude_joints=' + str(self.exclude_joints))
        if self.publish_new_tfs:
            parts.append("publish_new_tfs")
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class FixJointStates(DeserializedMessageFilter):
    """Adjust some joint states. If TFs are computed from them, use RecomputeTFFromJointStates to recompute TF."""

    class _JointStateType(Enum):
        POSITION = 1
        VELOCITY = 2
        EFFORT = 3
        INTEGRATE_POSITION = 4

    class _OperationType(Enum):
        VALUE = 1
        MULTIPLIER = 2
        OFFSET = 3
        NORMALIZE_ANGLE = 4
        NORMALIZE_ANGLE_POSITIVE = 5

    def __init__(self, changes=None, add_tags=None, *args, **kwargs):
        # type: (Optional[Dict[STRING_TYPE, Dict[STRING_TYPE, Dict[STRING_TYPE, Any]]]], Optional[Set[STRING_TYPE]], Any, Any) -> None  # noqa
        """
        :param changes: Dict specifying what should be changed. Keys are joint names. Values are dicts with possible
                        keys "position", "velocity", "effort", "integrate_position". Values of the first 3 dicts can be
                        "value" (set absolute value), "offset" (add offset), "multiplier" (multiply value),
                        "normalize_angle" (normalizes value to (-pi, pi)), "normalize_angle_positive" (normalizes
                        value to (0, 2*pi)).
                        If "value" is specified, then "offset" and "multiplier" are ignored. Otherwise, "multiplier" is
                        applied first and "offset" is added to the result.
                        "integrate_position" dict can contain keys "min_dt". "min_dt" specifies the minimum time delta
                        between consecutive joint states to consider them an update. It defaults to 0.01 s.
                        The first position when integrating position is the normal value contained in the message (it
                        can be influenced by the specified "position" change (e.g. set to 0)).
        :param add_tags: Tags to be added to modified joint states messages.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(FixJointStates, self).__init__(include_types=(JointState._type,), *args, **kwargs)  # noqa

        JointStateType = FixJointStates._JointStateType
        OperationType = FixJointStates._OperationType

        def convert_operations(operations):  # type: (Dict[STRING_TYPE, float]) -> Dict[OperationType, float]
            result = {}
            if "value" in operations:
                result[OperationType.VALUE] = float(operations["value"])
            if "multiplier" in operations:
                result[OperationType.MULTIPLIER] = float(operations["multiplier"])
            if "offset" in operations:
                result[OperationType.OFFSET] = float(operations["offset"])
            if "normalize_angle" in operations:
                result[OperationType.NORMALIZE_ANGLE] = bool(operations["normalize_angle"])
            if "normalize_angle_positive" in operations:
                result[OperationType.NORMALIZE_ANGLE_POSITIVE] = bool(operations["normalize_angle_positive"])
            return result

        self._changes = {}
        if changes is not None:
            for joint_name, change in changes.items():
                self._changes[joint_name] = {}
                if "position" in change:
                    self._changes[joint_name][JointStateType.POSITION] = convert_operations(change["position"])
                if "velocity" in change:
                    self._changes[joint_name][JointStateType.VELOCITY] = convert_operations(change["velocity"])
                if "effort" in change:
                    self._changes[joint_name][JointStateType.EFFORT] = convert_operations(change["effort"])
                if "integrate_position" in change:
                    self._changes[joint_name][JointStateType.INTEGRATE_POSITION] = change["integrate_position"]
        self._changed_joint_names = TopicSet(self._changes.keys())  # for efficient filtering

        self._add_tags = add_tags

        self._integrated_positions = {}
        self._last_velocities = {}
        self._last_integration_stamps = {}

    def filter(self, topic, msg, stamp, header, tags):
        JointStateType = FixJointStates._JointStateType

        changed_tags = copy.deepcopy(tags)
        changed_tags.add(MessageTags.CHANGED)
        if self._add_tags:
            changed_tags = changed_tags.union(self._add_tags)

        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self._changed_joint_names:
                continue
            changes = self._changes[name]

            if len(msg.position) > i and JointStateType.POSITION in changes:
                if isinstance(msg.position, tuple):
                    msg.position = list(msg.position)
                msg.position[i] = self._apply_changes(msg.position[i], changes[JointStateType.POSITION])
                tags = changed_tags

            if len(msg.velocity) > i and JointStateType.VELOCITY in changes:
                if isinstance(msg.velocity, tuple):
                    msg.velocity = list(msg.velocity)
                msg.velocity[i] = self._apply_changes(msg.velocity[i], changes[JointStateType.VELOCITY])
                tags = changed_tags

            if len(msg.effort) > i and JointStateType.EFFORT in changes:
                if isinstance(msg.effort, tuple):
                    msg.effort = list(msg.effort)
                msg.effort[i] = self._apply_changes(msg.effort[i], changes[JointStateType.EFFORT])
                tags = changed_tags

            if len(msg.position) > i and len(msg.velocity) > i and JointStateType.INTEGRATE_POSITION in changes:
                if name not in self._last_integration_stamps:
                    self._last_integration_stamps[name] = msg.header.stamp
                    self._integrated_positions[name] = msg.position[i]
                    self._last_velocities[name] = msg.velocity[i]
                else:
                    min_dt = changes[JointStateType.INTEGRATE_POSITION].get("min_dt", 0.01)
                    dt = (msg.header.stamp - self._last_integration_stamps[name]).to_sec()
                    if dt >= min_dt:
                        vel = (msg.velocity[i] + self._last_velocities[name]) / 2.0
                        distance = vel * dt
                        self._integrated_positions[name] += distance
                        if changes[JointStateType.INTEGRATE_POSITION].get("normalize_angle", False):
                            self._integrated_positions[name] = normalize_angle(self._integrated_positions[name])
                        if changes[JointStateType.INTEGRATE_POSITION].get("normalize_angle_positive", False):
                            self._integrated_positions[name] = normalize_angle_positive(
                                self._integrated_positions[name])
                        self._last_velocities[name] = msg.velocity[i]
                        self._last_integration_stamps[name] = msg.header.stamp
                    msg.position[i] = self._integrated_positions[name]
                    tags = changed_tags

        return topic, msg, stamp, header, tags

    def _apply_changes(self, value, changes):
        if len(changes) == 0:
            return value

        OperationType = FixJointStates._OperationType

        result = value
        if OperationType.VALUE in changes:
            result = changes[OperationType.VALUE]
        else:
            result = result * changes.get(OperationType.MULTIPLIER, 1.0) + changes.get(OperationType.OFFSET, 0.0)

        if changes.get(OperationType.NORMALIZE_ANGLE, False):
            result = normalize_angle(result)
        if changes.get(OperationType.NORMALIZE_ANGLE_POSITIVE, False):
            result = normalize_angle_positive(result)

        return result

    def reset(self):
        self._integrated_positions = {}
        self._last_velocities = {}
        self._last_integration_stamps = {}
        super(FixJointStates, self).reset()

    def _str_params(self):
        parts = []
        parts.append('changes=' + dict_to_str(self._changes))
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class InterpolateJointStates(DeserializedMessageFilter):
    """Interpolate some joint states if their frequency is too low or some messages are missing."""

    instance_id = 0

    def __init__(self, joints, max_dt, dt_tolerance=0.05, include_topics=None, add_tags=None, exclude_tags=None,
                 *args, **kwargs):
        # type: (List[STRING_TYPE], float, float, STRING_TYPE, Optional[Set[STRING_TYPE]], List[Any], Any, Any) -> None
        """
        :param joints: The joints to interpolate. If empty, interpolate all joints.
        :param max_dt: The desired time difference between two consecutive messages.
        :param dt_tolerance: If the actual time delta is within this number from max_dt, no interpolation is done.
        :param include_topics: Topics with joint states (defaults to ['joint_states']).
        :param add_tags: Tags to be added to generated joint states messages.
        :param list exclude_tags: If nonempty, the filter will skip messages with these tags. Each element of
                                  this list is itself a set. For a message to be skipped, at least one set has to be
                                  a subset of the tags list of the message.
                                  This filter automatically adds a tag that prevents an instance of the filter to
                                  process messages generated by itself.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        InterpolateJointStates.instance_id += 1
        self._self_tag = "interpolate_joint_states_" + str(InterpolateJointStates.instance_id)
        _exclude_tags = [({t} if isinstance(t, STRING_TYPE) else set(t)) for t in exclude_tags] if exclude_tags else []
        _exclude_tags.append({self._self_tag})

        super(InterpolateJointStates, self).__init__(
            include_topics=('joint_states',) if include_topics is None else include_topics,
            include_types=(JointState._type,), exclude_tags=tuple(_exclude_tags), *args, **kwargs)  # noqa

        self._joints = TopicSet(joints)
        self._max_dt = rospy.Duration(max_dt)
        self._dt_tolerance = rospy.Duration(dt_tolerance)

        self._add_tags = ((set(add_tags) if add_tags else set()).union({self._self_tag}))

        self._last_states = {}

    def filter(self, topic, msg, stamp, header, tags):
        gen_tags = tags_for_generated_msg(tags)
        if self._add_tags:
            gen_tags = gen_tags.union(self._add_tags)

        gen_states = []

        for i in range(len(msg.name)):
            name = msg.name[i]
            if self._joints and name not in self._joints:
                continue

            state = self._get_state(msg, i)

            if name not in self._last_states:
                self._last_states[name] = state
                continue

            prev_state = self._last_states[name]
            self._last_states[name] = state

            dt = state[0] - prev_state[0]
            if dt < self._max_dt + self._dt_tolerance:
                continue

            gen_stamp = prev_state[0] + self._max_dt
            while gen_stamp + self._dt_tolerance < state[0]:
                gen_states.append(self._interpolate(gen_stamp, prev_state, state))
                gen_stamp = gen_stamp + self._max_dt

        gen_stamps = sorted(list(set([s[0] for s in gen_states])))
        gen_msgs = []
        for gen_stamp in gen_stamps:
            states = [s for s in gen_states if s[0] == gen_stamp]
            gen_msg = JointState()
            gen_msg.header.frame_id = msg.header.frame_id
            gen_msg.header.stamp = gen_stamp
            for s in states:
                gen_msg.name.append(s[1])
                if s[2] is not None:
                    gen_msg.position.append(s[2])
                if s[3] is not None:
                    gen_msg.velocity.append(s[3])
                if s[4] is not None:
                    gen_msg.effort.append(s[4])
            gen_msgs.append((topic, gen_msg, gen_stamp, header, gen_tags))

        return [(topic, msg, stamp, header, tags)] + gen_msgs

    def _get_state(self, msg, i):
        return (
            msg.header.stamp,
            msg.name[i],
            msg.position[i] if len(msg.position) > i else None,
            msg.velocity[i] if len(msg.velocity) > i else None,
            msg.effort[i] if len(msg.effort) > i else None,
        )

    def _interpolate(self, stamp, prev_state, state):
        # TODO support proper angular interpolation
        ratio = (stamp - prev_state[0]).to_sec() / (state[0] - prev_state[0]).to_sec()
        return (
            stamp,
            state[1],
            prev_state[2] + ratio * (state[2] - prev_state[2]),
            prev_state[3] + ratio * (state[3] - prev_state[3]),
            prev_state[4] + ratio * (state[4] - prev_state[4]),
        )

    def reset(self):
        self._last_states = {}
        super(InterpolateJointStates, self).reset()

    def _str_params(self):
        parts = []
        parts.append('joints=' + repr(self._joints))
        parts.append('max_dt=' + str(self._max_dt))
        parts.append('dt_tolerance=' + str(self._dt_tolerance))
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class RecomputeAckermannOdometry(DeserializedMessageFilter):
    """Adjust some joint states. If TFs are computed from them, use RecomputeTFFromJointStates to recompute TF."""

    def __init__(self, wheel_radius, wheel_separation, traction_joint, steering_joint, joint_state_topic="joint_states",
                 frame_id="base_link", odom_frame_id="odom", odom_topic="odom", cmd_vel_topic=None,
                 cmd_vel_frame_id=None, min_dt=0.01, heading_change_coef=1.0, add_tags=None, *args, **kwargs):
        """
        :param float wheel_radius: Radius of the traction wheel [m].
        :param float wheel_separation: Separation of the steering and traction axles (wheelbase) [m].
        :param str traction_joint: Joint whose velocity will be used as traction velocity.
        :param str steering_joint: Joint whose position will be used as steering angle.
        :param str joint_state_topic: Topic with joint states.
        :param str frame_id: Child frame ID of the odom messages.
        :param str odom_frame_id: Parent frame ID of the odom messages.
        :param str odom_topic: Odometry topic to publish.
        :param str cmd_vel_topic: If not None, the computed linear and angular velocities are published on this topic
                                  as cmd_vel messages.
        :param str cmd_vel_frame_id: If not None, the published cmd_vel is a stamped message with this frame_id.
        :param float min_dt: If the computed dt is smaller than this, the joint state message is ignored for odometry.
        :param float heading_change_coef: This is a hack. The angular distance added to yaw is multiplied by this number
                                          after being computed from angular velocity and dt.
        :param set add_tags: Tags to be added to the generated odometry messages.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(RecomputeAckermannOdometry, self).__init__(
            include_topics=(joint_state_topic,), include_types=(JointState._type,), *args, **kwargs)  # noqa
        self._wheel_radius = wheel_radius
        self._wheel_separation = wheel_separation
        self._traction_joint = traction_joint
        self._steering_joint = steering_joint
        self._frame_id = frame_id
        self._odom_frame_id = odom_frame_id
        self._odom_topic = odom_topic
        self._cmd_vel_topic = cmd_vel_topic
        self._cmd_vel_frame_id = cmd_vel_frame_id
        self._min_dt = min_dt
        self._heading_change_coef = heading_change_coef
        self._add_tags = add_tags

        self._connection_header = create_connection_header(self._odom_topic, Odometry, False)
        if self._cmd_vel_topic:
            self._connection_header_cmd_vel = create_connection_header(
                self._cmd_vel_topic, TwistStamped if self._cmd_vel_frame_id else Twist, False)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._lin_vel = 0.0
        self._ang_vel = 0.0

        self._steering_angle = 0.0
        self._last_time = None

    def filter(self, topic, msg, stamp, header, tags):
        result = [(topic, msg, stamp, header, tags)]

        if self._steering_joint in msg.name:
            i = msg.name.index(self._steering_joint)
            self._steering_angle = msg.position[i]
        if self._traction_joint in msg.name:
            if self._last_time is None:
                self._last_time = msg.header.stamp
                result.append(self._construct_odom(msg.header.stamp, stamp, tags))
            else:
                i = msg.name.index(self._traction_joint)
                wheel_ang_vel = msg.velocity[i]
                dt = (msg.header.stamp - self._last_time).to_sec()
                if dt > self._min_dt:
                    self._last_time = msg.header.stamp

                    self._lin_vel = wheel_ang_vel * self._wheel_radius
                    linear = self._lin_vel * dt

                    steering_angle = normalize_angle(self._steering_angle)
                    angular = math.tan(steering_angle) * linear / self._wheel_separation
                    self._ang_vel = angular / dt
                    angular *= self._heading_change_coef

                    direction = self._yaw + angular * 0.5
                    self._x += linear * math.cos(direction)
                    self._y += linear * math.sin(direction)
                    self._yaw += angular

                    result.append(self._construct_odom(msg.header.stamp, stamp, tags))
                    if self._cmd_vel_topic:
                        result.append(self._construct_cmd_vel(msg.header.stamp, stamp, tags))

        return result

    def _construct_odom(self, header_stamp, receive_stamp, tags):
        msg = Odometry()
        msg.header.frame_id = self._odom_frame_id
        msg.header.stamp = header_stamp
        msg.child_frame_id = self._frame_id
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = quat_msg_from_rpy(0, 0, self._yaw)
        msg.twist.twist.linear.x = self._lin_vel
        msg.twist.twist.angular.z = self._ang_vel
        odom_tags = tags_for_generated_msg(tags)
        if self._add_tags:
            odom_tags = odom_tags.union(self._add_tags)
        return self._odom_topic, msg, receive_stamp, self._connection_header, odom_tags

    def _construct_cmd_vel(self, header_stamp, receive_stamp, tags):
        twist = Twist()
        twist.linear.x = self._lin_vel
        twist.angular.z = self._ang_vel

        if self._cmd_vel_frame_id:
            msg = TwistStamped()
            msg.header.frame_id = self._cmd_vel_frame_id
            msg.header.stamp = header_stamp
            msg.twist = twist
        else:
            msg = twist

        cmd_vel_tags = tags_for_generated_msg(tags)
        if self._add_tags:
            cmd_vel_tags = cmd_vel_tags.union(self._add_tags)

        return self._cmd_vel_topic, msg, receive_stamp, self._connection_header_cmd_vel, cmd_vel_tags

    def _str_params(self):
        parts = []
        parts.append('wheel_radius=' + str(self._wheel_radius))
        parts.append('wheel_separation=' + str(self._wheel_separation))
        parts.append('traction_joint=' + self._traction_joint)
        parts.append('steering_joint=' + self._steering_joint)
        parts.append('frame_id=' + self._frame_id)
        parts.append('odom_frame_id=' + self._odom_frame_id)
        parts.append('odom_topic=' + self._odom_topic)

        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class OdomToTf(DeserializedMessageFilter):
    """Convert odometry to TF."""

    def __init__(self, odom_topic, tf_topic="tf", add_tags=None, *args, **kwargs):
        # type: (STRING_TYPE, STRING_TYPE, Optional[Set[STRING_TYPE]], Any, Any) -> None
        """
        :param odom_topic: Odometry topic to convert.
        :param tf_topic: The TF topic.
        :param add_tags: Tags to be added to the generated TF messages.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(OdomToTf, self).__init__(
            include_topics=(odom_topic,), include_types=(Odometry._type,), *args, **kwargs)  # noqa

        self._odom_topic = odom_topic
        self._tf_topic = tf_topic
        self._add_tags = add_tags
        self._connection_header = create_connection_header(tf_topic, TFMessage, False)

    def filter(self, topic, msg, stamp, header, tags):
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = msg.header.frame_id
        odom_tf.header.stamp = msg.header.stamp
        odom_tf.child_frame_id = msg.child_frame_id
        odom_tf.transform.translation = msg.pose.pose.position
        odom_tf.transform.rotation = msg.pose.pose.orientation

        tf_msg = TFMessage()
        tf_msg.transforms.append(odom_tf)

        tf_tags = tags_for_generated_msg(tags)
        if self._add_tags:
            tf_tags = tf_tags.union(self._add_tags)

        return [
            (topic, msg, stamp, header, tags),
            (self._tf_topic, tf_msg, stamp, self._connection_header, tf_tags),
        ]

    def _str_params(self):
        parts = []
        parts.append('odom_topic=' + self._odom_topic)
        if self._tf_topic != "tf":
            parts.append('tf_topic=' + self._tf_topic)

        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


CalibrationYAMLType = Union[STRING_TYPE, Tuple[STRING_TYPE, STRING_TYPE]]


class FixCameraCalibration(DeserializedMessageFilter):
    """Adjust some camera calibrations."""

    def __init__(self, calibrations=None, warn_size_change=True, *args, **kwargs):
        # type: (Optional[Dict[STRING_TYPE, CalibrationYAMLType]], bool, Any, Any) -> None  # noqa
        """
        :param calibrations: Dictionary with camera_info topic names as keys and YAML files with calibrations as values.
                             If the calibration is in kalibr format and the camera is not cam0, then pass a tuple
                             (YAML file, cam_name) instead of just directly YAML file
        :param warn_size_change: If True (default), warn if the fixed camera info has different width or height than the
                                 original.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(FixCameraCalibration, self).__init__(include_topics=list(calibrations.keys()) if calibrations else None,
                                                   include_types=(CameraInfo._type,), *args, **kwargs)  # noqa

        self.warn_size_change = warn_size_change
        self._calibrations = {}
        self._cam_names = {}

        if calibrations is None:
            return
        for camera_info, calib_file in calibrations.items():
            calib_file, cam_name = (calib_file, "cam0") if isinstance(calib_file, STRING_TYPE) else calib_file

            calib = None
            try:
                calib = self.interpret_calibration(calib_file, cam_name)
            except AssertionError as e:
                print("Could not interpret the given calibration file for camera %s: %s." % (camera_info, str(e)))
                continue
            if calib is None:
                print("Could not interpret the given calibration file for camera %s." % (camera_info,))
                continue

            self._calibrations[camera_info] = calib

    def interpret_calibration(self, calib_file, cam_name):
        msg = None
        try:
            if rosconsole_set_logger_level("ros.camera_calibration_parsers", RosconsoleLevel.FATAL):
                rosconsole_notifyLoggerLevelsChanged()
            _, msg = readCalibration(calib_file)  # camera_info_manager format
            print("Interpreted calibration file %s using camera_info_manager format." % (calib_file,))

        except Exception:
            pass

        if msg is not None:
            return msg

        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        if "Intrinsics" in calib_data or "CalibParam" in calib_data:  # ikalibr format
            data = None
            cam_type = None

            if "Intrinsics" in calib_data:  # single-camera intrinsics file
                data = calib_data["Intrinsics"]["ptr_wrapper"]["data"]
                cam_type = calib_data["Intrinsics"]["polymorphic_name"]
            else:  # ikalibr_param.yaml file with the overall result of calibration
                data_all = calib_data["CalibParam"]["INTRI"]["Camera"]
                polymorphic_map = ["pinhole_brown_t2"]
                for item in data_all:
                    if "polymorphic_name" in item["value"]:
                        polymorphic_map.append(item["value"]["polymorphic_name"])
                        cam_type = item["value"]["polymorphic_name"]
                    else:
                        polymorphic_id = int(item["value"]["polymorphic_id"])
                        if len(polymorphic_map) >= polymorphic_id - 1:
                            cam_type = polymorphic_map[polymorphic_id]
                    if item["key"] == cam_name:
                        data = item["value"]["ptr_wrapper"]["data"]
                        break

            if data is None or cam_type is None:
                raise RuntimeError("Could not find camera %s in calibration file %s." % (cam_name, calib_file))

            w = data["img_width"]
            h = data["img_height"]

            msg = CameraInfo()
            msg.width = w
            msg.height = h
            msg.D = data["disto_param"]
            msg.distortion_model = \
                EQUIDISTANT if cam_type == "pinhole_fisheye" else (PLUMB_BOB if len(msg.D) < 6 else RATIONAL_POLYNOMIAL)
            msg.K = [
                data["focal_length"][0], 0.0, data["principal_point"][0],
                0.0, data["focal_length"][1], data["principal_point"][1],
                0.0, 0.0, 1.0,
            ]
            msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

            K = np.array(msg.K).reshape((3, 3))
            R = np.array(msg.R).reshape((3, 3))
            D = np.array(msg.D)
            if msg.distortion_model != EQUIDISTANT:
                P, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0.0)
            else:
                P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), R, balance=0.0)
            msg.P = [
                P[0, 0], P[0, 1], P[0, 2], 0.0,
                P[1, 0], P[1, 1], P[1, 2], 0.0,
                P[2, 0], P[2, 1], P[2, 2], 0.0,
            ]
            print("Interpreted calibration file %s using ikalibr format." % (calib_file,))
        elif len(calib_data) > 0 and "cam_overlaps" in calib_data[list(calib_data.keys())[0]]:  # kalibr format
            data = calib_data[cam_name]
            w, h = data["resolution"]
            cam_type = data["distortion_model"]

            msg = CameraInfo()
            msg.width = w
            msg.height = h
            msg.D = data["distortion_coeffs"]
            msg.distortion_model = \
                EQUIDISTANT if cam_type == "equidistant" else (PLUMB_BOB if len(msg.D) < 6 else RATIONAL_POLYNOMIAL)
            msg.K = [
                data["intrinsics"][0], 0.0, data["intrinsics"][2],
                0.0, data["intrinsics"][1], data["intrinsics"][3],
                0.0, 0.0, 1.0,
            ]
            msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

            K = np.array(msg.K).reshape((3, 3))
            R = np.array(msg.R).reshape((3, 3))  # TODO T_cn_cnm1
            D = np.array(msg.D)
            if msg.distortion_model != EQUIDISTANT:
                P, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0.0)
            else:
                P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), R, balance=0.0)
            msg.P = [
                P[0, 0], P[0, 1], P[0, 2], 0.0,
                P[1, 0], P[1, 1], P[1, 2], 0.0,
                P[2, 0], P[2, 1], P[2, 2], 0.0,
            ]
            print("Interpreted calibration file %s using kalibr format." % (calib_file,))
        else:
            raise RuntimeError("Unsupported camera calibration format: " + calib_file)

        return msg

    def filter(self, topic, msg, stamp, header, tags):

        if topic in self._calibrations:
            calib = copy.deepcopy(self._calibrations[topic])
            if self.warn_size_change and (calib.width != msg.width or calib.height != msg.height):
                print("Fixed camera info size (%i, %i) differs from original camera info (%i, %i)" % (
                    calib.width, calib.height, msg.width, msg.height))
            msg.distortion_model = calib.distortion_model
            msg.D = calib.D
            msg.R = calib.R
            msg.K = calib.K
            msg.P = calib.P
            tags.add(MessageTags.CHANGED)

        return topic, msg, stamp, header, tags

    def _str_params(self):
        parts = []
        parts.append('calibrations=' + ",".join(self._calibrations.keys()))
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class FixStaticTF(DeserializedMessageFilterWithTF):
    """Adjust some static transforms."""

    def __init__(self, transforms=None, add_tags=None, *args, **kwargs):
        # type: (Optional[Sequence[Dict[STRING_TYPE, STRING_TYPE]]], Optional[Set[STRING_TYPE]], Any, Any) -> None
        """
        :param transforms: The new transforms. The dicts have to contain keys "frame_id", "child_frame_id", "transform".
                           The transform has to be a 6-tuple (x, y, z, roll, pitch, yaw)
                           or 7-tuple (x, y, z, qx, qy, qz, qw).
        :param add_tags: Tags to be added to the modified TF messages.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(FixStaticTF, self).__init__(
            include_topics=["tf_static"], include_types=(TFMessage._type,), *args, **kwargs)  # noqa

        self._transforms = dict()
        self._add_tags = add_tags

        for t in (transforms if transforms is not None else list()):
            frame_id = t["frame_id"]
            child_frame_id = t["child_frame_id"]
            data = t["transform"]
            if len(data) == 7:
                transform = Transform(Vector3(*data[:3]), Quaternion(*data[3:]))
            elif len(data) == 6:
                transform = Transform(Vector3(*data[:3]), quat_msg_from_rpy(*data[3:]))
            else:
                raise RuntimeError("'transform' has to be either a 6-tuple or 7-tuple.")

            self._transforms[(frame_id, child_frame_id)] = transform

    def filter(self, topic, msg, stamp, header, tags):
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            if key in self._transforms:
                transform.transform = self._transforms[key]
                print("Adjusted transform %s->%s." % (key[0], key[1]))
                tags.add(MessageTags.CHANGED)
                if self._add_tags:
                    tags = tags.union(self._add_tags)

        return topic, msg, stamp, header, tags

    def _str_params(self):
        parts = []
        parts.append('transforms=' + repr(list(self._transforms.keys())))
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class StampTwist(DeserializedMessageFilter):
    """Adjust some static transforms."""

    def __init__(self, source_topic, stamped_topic=None, frame_id="base_link", *args, **kwargs):
        # type: (STRING_TYPE, Optional[STRING_TYPE], STRING_TYPE, Any, Any) -> None
        """
        :param source_topic: The Twist topic to stamp.
        :param stamped_topic: The stamped Twist topic to create.
        :param frame_id: The frame_id to use in the stamped messages.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(StampTwist, self).__init__(include_topics=[source_topic], *args, **kwargs)

        self._source_topic = source_topic
        self._stamped_topic = stamped_topic if stamped_topic is not None else (source_topic + "_stamped")
        self._frame_id = frame_id

        self._connection_header = create_connection_header(self._stamped_topic, TwistStamped)

    def filter(self, topic, msg, stamp, header, tags):
        stamped_msg = TwistStamped()
        stamped_msg.header.frame_id = self._frame_id
        stamped_msg.header.stamp = stamp
        stamped_msg.twist = msg

        return [
            (topic, msg, stamp, header, tags),
            (self._stamped_topic, stamped_msg, stamp, self._connection_header, tags_for_generated_msg(tags))
        ]

    def _str_params(self):
        parts = []
        parts.append('%s=>%s (frame %s)' % (self._source_topic, self._stamped_topic, self._frame_id))
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class FixExtrinsicsFromIKalibr(DeserializedMessageFilterWithTF):
    """Apply extrinsic calibrations from IKalibr."""

    def __init__(self, param_file, ref_imu_frame="imu", frames=None, add_tags=None, *args, **kwargs):
        # type: (STRING_TYPE, STRING_TYPE, Optional[Dict[STRING_TYPE, Dict[STRING_TYPE, STRING_TYPE]]], Optional[Set[STRING_TYPE]], Any, Any) -> None  # noqa
        """
        :param param_file: Path to ikalibr_param.yaml file - the result of extrinsic calibration.
        :param ref_imu_frame: Frame of the reference IMU towards which everything is calibrated.
        :param frames: The TF frames to fix. Keys are the topic names used in iKalibr. Values are dicts with keys
                       "sensor_frame" and optionally "adjust_frame" (if not specified, "sensor_frame" is used).
                       "adjust_frame" specifies the frame whose transform should be changed, which can be useful
                       if you want to change a transform somewhere in the middle of the TF chain and not the last one.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(FixExtrinsicsFromIKalibr, self).__init__(
            include_topics=["tf", "tf_static"], include_types=(TFMessage._type,), *args, **kwargs)  # noqa

        self._ref_imu_frame = ref_imu_frame
        self._frames = frames if frames is not None else dict()
        self._param_file = param_file
        self._add_tags = add_tags

        with open(param_file, 'r') as f:
            self._params = yaml.safe_load(f)

        extri = self._params["CalibParam"]["EXTRI"]
        tfs = dict()
        for key, values in extri.items():
            for item in values:
                ikalibr_name = item["key"]
                value = item["value"]
                if ikalibr_name not in tfs:
                    tfs[ikalibr_name] = Transform()
                if key.startswith("POS_"):
                    msg = Vector3(value["r0c0"], value["r1c0"], value["r2c0"])
                    tfs[ikalibr_name].translation = msg
                elif key.startswith("SO3_"):
                    msg = Quaternion(value["qx"], value["qy"], value["qz"], value["qw"])
                    tfs[ikalibr_name].rotation = msg

        print("Read %i transforms from %s." % (len(tfs), param_file))

        self._sensor_to_adjust_frame = dict()
        self._adjust_frame_to_sensor = dict()
        self._sensor_transforms = dict()
        for ikalibr_name, item in self._frames.items():
            sensor_frame = item["sensor_frame"]
            adjust_frame = item.get("adjust_frame", sensor_frame)

            if adjust_frame in self._adjust_frame_to_sensor:
                raise RuntimeError(
                    "Duplicate appearance of adjust frame %s. This is invalid configuration!" % (adjust_frame,))

            self._sensor_transforms[sensor_frame] = tfs[ikalibr_name]
            self._sensor_to_adjust_frame[sensor_frame] = adjust_frame
            self._adjust_frame_to_sensor[adjust_frame] = sensor_frame

        self._tf = BufferCore()

    @staticmethod
    def fix_transform_type(t):
        """Convert from the ad-hoc rosbag type to true Transform type (needed by ros_numpy)."""
        return Transform(
            Vector3(t.translation.x, t.translation.y, t.translation.z),
            Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    @staticmethod
    def fix_transform_stamped_type(t):
        """Convert from the ad-hoc rosbag type to true TransformStamped type (needed by tf2_py)."""
        return TransformStamped(
            Header(t.header.seq, t.header.stamp, t.header.frame_id),
            t.child_frame_id,
            FixExtrinsicsFromIKalibr.fix_transform_type(t.transform))

    def filter(self, topic, msg, stamp, header, tags):
        if topic in self._tf_static_topics:
            for transform in msg.transforms:
                self._tf.set_transform_static(self.fix_transform_stamped_type(transform), "filter_bag")
        else:
            for transform in msg.transforms:
                self._tf.set_transform(self.fix_transform_stamped_type(transform), "filter_bag")

        latest = rospy.Time(0)
        for transform in msg.transforms:
            if transform.child_frame_id in self._adjust_frame_to_sensor:
                adjust_frame = transform.child_frame_id
                sensor_frame = self._adjust_frame_to_sensor[adjust_frame]
                parent_frame = transform.header.frame_id

                sensor_tf = numpify(self._sensor_transforms[sensor_frame])
                t_parent_imu = numpify(
                    self._tf.lookup_transform_core(self._ref_imu_frame, parent_frame, latest).transform)
                t_adjust_parent = numpify(self.fix_transform_type(transform.transform))
                t_sensor_adjust = numpify(self._tf.lookup_transform_core(adjust_frame, sensor_frame, latest).transform)

                t_correction = \
                    np.matmul(np.matmul(inv(np.matmul(t_parent_imu, t_adjust_parent)), sensor_tf), inv(t_sensor_adjust))
                t_adjust_parent = np.matmul(t_adjust_parent, t_correction)
                transform.transform = msgify(Transform, t_adjust_parent)

                print("Adjusted transform %s->%s by %.3f m (for sensor %s)." % (
                      parent_frame, adjust_frame, np.linalg.norm(t_correction[:3, 3]), sensor_frame))
                tags.add(MessageTags.CHANGED)
                if self._add_tags:
                    tags = tags.union(self._add_tags)

        return topic, msg, stamp, header, tags

    def reset(self):
        self._tf.clear()
        super(FixExtrinsicsFromIKalibr, self).reset()

    def _str_params(self):
        parts = []
        parts.append('param_file=' + self._param_file)
        parts.append('ref_imu_frame=' + self._ref_imu_frame)
        parts.append('frames=' + ",".join(self._frames.keys()))
        parent_params = self._default_str_params(include_types=False)
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class AddSubtitles(NoMessageFilter):
    """Read subtitles from a text file and add them to the output as std_msgs/String messages."""

    def __init__(self, subtitles_file, topic, latch=False, time_offset=0.0):
        """
        :param subtitles_file: The file to read subtitles from.
        :param topic: The topic to put the subtitles on.
        :param latch: Whether the topic should be latched.
        :param time_offset: Relative time offset to add to the absolute subtitle time.
        """
        super(AddSubtitles, self).__init__()
        self._subtitles_file = subtitles_file
        self._topic = topic
        self._start_time = None
        self._latch = latch
        self._time_offset = rospy.Duration(time_offset)

        self._subtitles = None

    def on_filtering_start(self):
        super(AddSubtitles, self).on_filtering_start()
        self._start_time = rospy.Time(self._bag.get_start_time()) + self._time_offset
        subtitles_file = self.resolve_file(self._subtitles_file)
        self._subtitles = self._parse_subtitles(subtitles_file)
        print("Read", len(self._subtitles), "subtitles from file", subtitles_file)

    def _parse_subtitles(self, subtitles_file):
        if subtitles_file.endswith(".srt"):
            return self._parse_srt(subtitles_file)
        raise RuntimeError("Unsupported subtitles type. Only SRT is supported so far.")

    def _parse_srt(self, subtitles_file):
        class State(Enum):
            INIT = 0
            COUNTER_READ = 1
            TIME_READ = 2

        state = State.INIT
        start_time = None
        end_time = None
        lines = list()

        subtitles = list()

        def parse_time_str(time_str):
            h, m, s = time_str.split(':')
            s, ss = s.split(',')
            return rospy.Duration(
                int(h, base=10) * 3600 + int(m, base=10) * 60 + int(s, base=10),
                int(float('0.' + ss) * 1e9))

        with open(subtitles_file, 'r') as f:
            for line in f:
                line = line.strip()
                if state == State.INIT:
                    _ = int(line)
                    state = State.COUNTER_READ
                elif state == State.COUNTER_READ:
                    start_time_str, end_time_str = line.split(" --> ")
                    start_time = parse_time_str(start_time_str)
                    end_time = parse_time_str(end_time_str)
                    state = State.TIME_READ
                elif state == State.TIME_READ:
                    if len(line) == 0:
                        subtitles.append((start_time, end_time, "\n".join(lines)))
                        lines = list()
                        state = State.INIT
                    else:
                        lines.append(line)

        # in case the last empty line is missing
        if len(lines) > 0:
            subtitles.append((start_time, end_time, "\n".join(lines)))

        return subtitles

    def extra_initial_messages(self):
        connection_header = create_connection_header(self._topic, String, self._latch)
        for start_time, end_time, subtitle in self._subtitles:
            msg = String(data=subtitle)
            abs_time = self._start_time + start_time
            yield self._topic, msg, abs_time, connection_header, {MessageTags.GENERATED}

    def _str_params(self):
        params = ["topic=" + self._topic, "subtitles_file=" + self.resolve_file(self._subtitles_file)]
        if self._time_offset != rospy.Duration(0, 0):
            params.append("time_offset=%f" % (self._time_offset.to_sec(),))
        parent_params = super(AddSubtitles, self)._str_params()
        if len(parent_params) > 0:
            params.append(parent_params)
        return ",".join(params)


class DumpRobotModel(NoMessageFilter):
    """Read robot model URDF from ROS params and store it in a file."""

    def __init__(self, urdf_file, param="robot_description", remove_comments=False, pretty_print=False,
                 run_on_start=True):
        """
        :param urdf_file: Path to the URDF file. If relative, it will be resolved relative to the bag set by set_bag.
        :param param: The parameter where robot model should be read.
        :param remove_comments: Whether comments should be removed from the output URDF file.
        :param pretty_print: Whether to pretty-print the URDF file (if False, the original layout is preserved).
        :param run_on_start: If true, the model will be exported before messages are processed. If false, the model will
                             be exported after processing all messages.
        """
        super(DumpRobotModel, self).__init__()
        self._urdf_file = urdf_file
        self._param = param
        self._remove_comments = remove_comments
        self._pretty_print = pretty_print
        self._run_on_start = run_on_start

    def on_filtering_start(self):
        super(DumpRobotModel, self).on_filtering_start()

        if self._run_on_start:
            self.dump_model()

    def on_filtering_end(self):
        super(DumpRobotModel, self).on_filtering_end()

        if not self._run_on_start:
            self.dump_model()

    def dump_model(self):
        urdf = self._get_param(self._param)
        if urdf is None:
            print('Robot model not found on parameter', self._param, file=sys.stderr)
            return

        if self._remove_comments or self._pretty_print:
            parser = etree.XMLParser(remove_comments=self._remove_comments, encoding='utf-8')
            tree = etree.fromstring(urdf.encode('utf-8'), parser=parser)
            urdf = etree.tostring(tree, encoding='utf-8', xml_declaration=True,
                                  pretty_print=self._pretty_print).decode('utf-8')

        dest = self.resolve_file(self._urdf_file)
        with open(dest, 'w+') as f:
            print(urdf, file=f)
        print("Robot model from parameter", self._param, "exported to", dest)

    def _str_params(self):
        params = ["param=" + self._param, "urdf_file=" + self.resolve_file(self._urdf_file)]
        parent_params = super(DumpRobotModel, self)._str_params()
        if len(parent_params) > 0:
            params.append(parent_params)
        return ",".join(params)


class UpdateRobotModel(NoMessageFilter):
    """Read robot model from URDF file and update it in the ROS parameters."""

    def __init__(self, urdf_file, param="robot_description"):
        """
        :param urdf_file: Path to the URDF file. If relative, it will be resolved relative to the bag set by set_bag.
        :param param: The parameter where robot model should be stored.
        """
        super(UpdateRobotModel, self).__init__()
        self._urdf_file = urdf_file
        self._param = param

    def on_filtering_start(self):
        super(UpdateRobotModel, self).on_filtering_start()

        src = self.resolve_file(self._urdf_file)
        if not os.path.exists(src):
            print('Cannot find robot URDF file', src, file=sys.stderr)
            return

        with open(src, 'r', encoding='utf-8') as f:
            urdf = f.read()

        self._set_param(self._param, urdf)
        print('Robot model from %s set to parameter %s' % (src, self._param))

    def _str_params(self):
        params = ["param=" + self._param, "urdf_file=" + self.resolve_file(self._urdf_file)]
        parent_params = super(UpdateRobotModel, self)._str_params()
        if len(parent_params) > 0:
            params.append(parent_params)
        return ",".join(params)


class RemovePasswordsFromParams(NoMessageFilter):
    """Search for passwords in the ROS parameters and remove them."""

    def __init__(self):
        super(RemovePasswordsFromParams, self).__init__()
        self._removed_passwords = set()

    def on_filtering_start(self):
        super(RemovePasswordsFromParams, self).on_filtering_start()
        self.remove_passwords(self._params)

    def remove_passwords(self, params, prefix=""):
        if isinstance(params, dict):
            keys_to_remove = set()
            for key, val in params.items():
                if key.tolower().strip().startswith("pass"):
                    keys_to_remove.add(key)
                else:
                    self.remove_passwords(val, "/".join((prefix, key)))
            for key in keys_to_remove:
                del params[key]
                self._removed_passwords.add("/".join((prefix, key)))
        elif isinstance(params, list):
            for i in range(len(params)):
                self.remove_passwords(params[i], "%s[%i]" % (prefix, i))

    def _str_params(self):
        params = ["removed_passwords=" + repr(self._removed_passwords)]
        parent_params = super(RemovePasswordsFromParams, self)._str_params()
        if len(parent_params) > 0:
            params.append(parent_params)
        return ",".join(params)


class UpdateParams(NoMessageFilter):
    """Update ROS parameters."""

    def __init__(self, update=None, remove=None):
        super(UpdateParams, self).__init__()
        self._update = update if update is not None else dict()
        self._remove = remove if remove is not None else list()

        self._updated_params = set()
        self._removed_params = set()

    def on_filtering_start(self):
        super(UpdateParams, self).on_filtering_start()
        self.update_params(self._params, self._update)
        for param in self._remove:
            self.remove_param(self._params, param)

        if len(self._updated_params) > 0:
            print("Updated %i ROS parameters." % (len(self._updated_params),))
        if len(self._removed_params) > 0:
            print("Removed %i ROS parameters." % (len(self._removed_params),))

    def update_params(self, params, new_params, prefix=""):
        if not isinstance(new_params, dict) or not isinstance(params, dict):
            return
        for k, v in new_params.items():
            if k not in params:
                params[k] = v
                self._updated_params.add("/".join((prefix, k)))
            else:
                self.update_params(params[k], v, "/".join((prefix, k)))

    def remove_param(self, params, param, prefix=""):
        if isinstance(params, dict):
            for key, val in params.items():
                if key == param:
                    del params[key]
                    self._removed_params.add("/".join((prefix, key)))
                    return
                elif "/" in param:
                    param_key, param_rest = param.split("/", maxsplits=1)
                    if param_key == key:
                        self.remove_param(val, param_rest, "/".join((prefix, key)))

    def _str_params(self):
        params = ["removed_params=" + repr(self._removed_params), "updated_params=" + repr(self._updated_params)]
        parent_params = super(UpdateParams, self)._str_params()
        if len(parent_params) > 0:
            params.append(parent_params)
        return ",".join(params)


class FixSpotCams(RawMessageFilter):
    """Fix a problem with Spot robot cameras that publish a bit weird message header."""

    def __init__(self, *args, **kwargs):
        """
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(FixSpotCams, self).__init__(*args, **kwargs)

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        header["message_definition"] = CompressedImage._full_text
        header["md5sum"] = md5sum = CompressedImage._md5sum
        header["type"] = datatype = CompressedImage._type
        pytype = CompressedImage

        return topic, datatype, data, md5sum, pytype, stamp, header, tags


class MaxMessageSize(RawMessageFilter):
    """Drop messages larger than the specified message size."""

    def __init__(self, size_limit, *args, **kwargs):
        """
        :param int size_limit: Maximum message size [B].
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(MaxMessageSize, self).__init__(*args, **kwargs)
        self.size_limit = size_limit

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        if len(data) > self.size_limit:
            return None

        return topic, datatype, data, md5sum, pytype, stamp, header, tags

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
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(MakeLatched, self).__init__(*args, **kwargs)

    def filter(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        header["latching"] = "1"
        return topic, datatype, data, md5sum, pytype, stamp, header, tags


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
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
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

    def filter(self, topic, msg, stamp, header, tags):
        enc = msg.encoding
        is_color = isColor(enc) or isMono(enc) or isBayer(enc) or enc == YUV422
        is_depth = isDepth(enc)

        if (self.only_color and not is_color) or (self.only_depth and not is_depth):
            return topic, msg, stamp, header, tags

        transport = ("compressedDepth" if is_depth else "compressed") if self.transport is None else self.transport
        transport = self.transport_mapping.get(enc, transport)

        compressed_msg, compressed_topic, err = self.get_image_for_transport(msg, topic, transport)
        # If encoding using compressedDepth fails, try with compressed
        if compressed_msg is None and transport == "compressedDepth":
            transport = "compressed"
            compressed_msg, compressed_topic, err = self.get_image_for_transport(msg, topic, transport)

        if compressed_msg is None:
            print('Error converting image: ' + str(err), file=sys.stderr)
            return topic, msg, stamp, header, tags

        return compressed_topic, compressed_msg, stamp, header, tags_for_generated_msg(tags)

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
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(DecompressImages, self).__init__(
            include_types=include_types if include_types is not None else ['sensor_msgs/CompressedImage'],
            *args, **kwargs)

        self._cv = CvBridge()

        # map from topic to desired encoding of the raw images (one of the strings in sensor_msgs/image_encodings.h)
        self.desired_encodings = desired_encodings if desired_encodings is not None else {}
        self.transport = transport
        self.transport_params = transport_params if transport_params is not None else {}

    def filter(self, topic, msg, stamp, header, tags):
        transport = self.transport
        raw_topic = topic
        if transport is None and "/" in topic:
            raw_topic, transport = topic.rsplit("/", 1)

        params = self.transport_params.get(transport, {}) if transport is not None else {}
        raw_img, err = decode(msg, topic, params)
        if raw_img is None:
            print('Error converting image: ' + str(err), file=sys.stderr)
            return topic, msg, stamp, header, tags

        desired_encoding = self.desired_encodings.get(topic, 'passthrough')
        if desired_encoding == 'passthrough':
            return raw_topic, raw_img, stamp, header, tags_for_generated_msg(tags)

        compressed_fmt, compressed_depth_fmt, err = guess_any_compressed_image_transport_format(msg)
        if compressed_fmt is None and compressed_depth_fmt is None:
            print('Error converting image to desired encoding: ' + str(err), file=sys.stderr)
            return raw_topic, raw_img, stamp, header, tags_for_generated_msg(tags)

        raw_encoding = compressed_fmt.rawEncoding if compressed_fmt is not None else compressed_depth_fmt.rawEncoding
        if desired_encoding == raw_encoding:
            return raw_topic, raw_img, stamp, header, tags_for_generated_msg(tags)

        try:
            cv_img = self._cv.imgmsg_to_cv2(raw_img, desired_encoding)
            return raw_topic, self._cv.cv2_to_imgmsg(cv_img, desired_encoding, raw_img.header), stamp, header
        except CvBridgeError as e:
            print('Error converting image to desired encoding: ' + str(e), file=sys.stderr)
            return raw_topic, raw_img, stamp, header, tags_for_generated_msg(tags)

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
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
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

    def filter(self, topic, msg, stamp, header, tags):
        if msg.encoding != TYPE_16UC1 and msg.encoding != TYPE_32FC1:
            # If we encounter a non-depth topic, exclude it from this filter. So only the first message from non-depth
            # topics will be needlessly deserialized because of this filter.
            self._exclude_topics = TopicSet(list(self._exclude_topics) + [topic])
            return topic, msg, stamp, header, tags

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
            new_image = new_topic, new_msg, stamp, header, tags_for_generated_msg(tags)
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


class BlurFaces(DeserializedMessageFilter):
    """Blur faces in color/mono images using 'deface' library."""

    def __init__(self, include_types=None, transport_params=None, ignore_transports=None, threshold=0.2, ellipse=True,
                 scale=(960, 960), mask_scale=1.3, replacewith='blur', replaceimg=None, mosaicsize=20, backend='auto',
                 publish_faces=True, add_tags=None, *args, **kwargs):
        """
        :param list include_types: Types of messages to work on. The default is sensor_msgs/Image and CompressedImage.
        :param dict transport_params: Parameters of image transport(s). Keys are transport names (e.g. 'compressed'),
                                      values are the publisher dynamic reconfigure parameters.
        :param list ignore_transports: List of transport names to ignore (defaults to`['compressedDepth']`).
        :param float threshold: Threshold for face detection.
        :param add_tags: Tags to be added to messages with some blurred faces.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        try:
            import deface.centerface
            import deface.deface
        except ImportError:
            cmds = [
                'sudo apt install python3-numpy python3-skimage python3-pil python3-opencv python3-imageio',
                'pip3 install --user --upgrade pip',
            ]
            accel_cmds = list()
            pip_constraints = "'numpy~=1.17.0' 'pillow~=7.0.0' 'imageio~=2.4.0' 'scikit_image~=0.16.0'"
            if os.path.exists('/etc/nv_tegra_release'):
                # from https://elinux.org/Jetson_Zoo#ONNX_Runtime, version compatible with onnx 1.12, Py 3.8 and JP 5
                url = 'https://nvidia.box.com/shared/static/v59xkrnvederwewo2f1jtv6yurl92xso.whl'
                cmds += [
                    "python3 -m pip install --user " + pip_constraints + " 'onnx~=1.12.0'",
                ]
                accel_cmds += [
                    'wget ' + url + ' -O onnxruntime_gpu-1.12.0-cp38-cp38-linux_aarch64.whl',
                    'python3 -m pip install --user onnxruntime_gpu-1.12.0-cp38-cp38-linux_aarch64.whl',
                ]
            else:
                cmds += [
                    "python3 -m pip install --user " + pip_constraints + " 'onnx~=1.12.0'",
                ]
                accel_cmds += [
                    "python3 -m pip install --user 'onnxruntime-openvino~=1.12.0'",
                ]
            cmds += [
                'python3 -m pip install --user --no-deps deface~=1.4.0',
                'python3 -m pip uninstall pip',
            ]

            print('Error importing deface module. Please install it with the following commands:', file=sys.stderr)
            for cmd in cmds:
                print("\t" + cmd, file=sys.stderr)
            print('To add HW acceleration, please run the following commands:', file=sys.stderr)
            for cmd in accel_cmds:
                print("\t" + cmd, file=sys.stderr)

            raise

        default_types = [Image._type, CompressedImage._type, Config._type]

        super(BlurFaces, self).__init__(
            include_types=include_types if include_types is not None else default_types, *args, **kwargs)

        self.transport_params = transport_params if transport_params is not None else {}
        self.received_transport_params = {}
        self.ignore_transports = ['/' + t for t in ignore_transports] if ignore_transports is not None \
            else ['/compressedDepth']
        self.threshold = threshold
        self.ellipse = ellipse
        self.scale = scale
        self.mask_scale = mask_scale
        self.replacewith = replacewith
        self.replaceimg = replaceimg
        self.mosaicsize = mosaicsize
        self.backend = backend
        self.publish_faces = publish_faces
        self.add_tags = add_tags

        self._cv = CvBridge()
        self._centerface = deface.centerface.CenterFace(in_shape=None, backend=self.backend)

    def consider_message(self, topic, datatype, stamp, header, tags):
        if datatype == Config._type:
            return True

        if not super(BlurFaces, self).consider_message(topic, datatype, stamp, header, tags):
            return False

        for t in self.ignore_transports:
            if topic.endswith(t):
                return False

        return True

    def process_transport_params(self, topic, msg, stamp, header, tags):
        if not topic.endswith('/parameter_updates'):
            return topic, msg, stamp, header, tags
        transport_topic, _ = topic.rsplit('/', 1)
        self.received_transport_params[transport_topic] = decode_config(msg)
        return topic, msg, stamp, header, tags

    def filter(self, topic, msg, stamp, header, tags):
        if msg._type == Config._type:
            return self.process_transport_params(topic, msg, stamp, header, tags)

        if msg._type == Image._type:
            raw_msg = msg
            raw_topic = topic
            transport = 'raw'
        else:
            transport = None
            if "/" in topic:
                raw_topic, transport = topic.rsplit("/", 1)
            if transport is None or len(transport) == 0:
                print("Compressed image on a topic without suffix [%s]. Passing message." % (topic,), file=sys.stderr)
                return topic, msg, stamp, header, tags

            raw_msg, err = decode(msg, topic, {})
            if raw_msg is None:
                print('Error converting image: ' + str(err), file=sys.stderr)
                return topic, msg, stamp, header, tags

        enc = raw_msg.encoding
        is_color = isColor(enc) or isMono(enc) or isBayer(enc) or enc == YUV422
        if not is_color:
            return topic, msg, stamp, header, tags

        img = self._cv.imgmsg_to_cv2(raw_msg)
        if raw_msg.encoding == RGB8 or numChannels(raw_msg.encoding) == 1:
            rgb_img = img
        else:
            rgb_img = self._cv.imgmsg_to_cv2(raw_msg, RGB8)

        if self.scale is not None:
            scale = min(
                min(1.0, self.scale[0] / img.shape[0]),
                min(1.0, self.scale[1] / img.shape[1]))
            self._centerface.in_shape = (int(img.shape[0] * scale), int(img.shape[1] * scale))
        else:
            self._centerface.in_shape = None

        dets, _ = self._centerface(rgb_img, threshold=self.threshold)

        import deface.deface

        bad_dets = list()
        for i, det in enumerate(dets):
            boxes, score = det[:4], det[4]
            x1, y1, x2, y2 = boxes.astype(int)
            x1, y1, x2, y2 = deface.deface.scale_bb(x1, y1, x2, y2, self.mask_scale)
            w, h = x2 - x1, y2 - y1
            if w >= 0.2 * img.shape[1] or h >= 0.2 * img.shape[0]:
                bad_dets.append(i)
        dets = np.delete(dets, bad_dets, axis=0)

        if len(dets) == 0:
            return topic, msg, stamp, header, tags

        img = img.copy()

        img_tags = copy.deepcopy(tags)
        img_tags.add(MessageTags.CHANGED)
        if self.add_tags:
            img_tags = img_tags.union(self.add_tags)

        dets_tags = tags_for_generated_msg(tags)

        if deface.__version__ >= "1.5.0":
            deface.deface.anonymize_frame(
                dets, img, self.mask_scale, self.replacewith, self.ellipse, False, self.replaceimg, self.mosaicsize)
        else:
            deface.deface.anonymize_frame(
                dets, img, self.mask_scale, self.replacewith, self.ellipse, False, self.replaceimg)

        raw_msg = self._cv.cv2_to_imgmsg(img, enc, msg.header)
        if msg._type == Image._type:
            msg = raw_msg
        else:
            msg, err = self.get_image_for_transport(msg, raw_msg, topic, transport)

        if not self.publish_faces:
            return topic, msg, stamp, header, img_tags

        dets_msg = self.dets_to_msg(dets, msg)
        dets_header = copy.deepcopy(header)
        dets_header["topic"] = raw_topic + '/anonymized_faces'  # The rest will be fixed by fix_connection_header()

        return [
            (topic, msg, stamp, header, img_tags),
            (dets_header["topic"], dets_msg, stamp, dets_header, dets_tags)
        ]

    def dets_to_msg(self, dets, msg):
        dets_msg = Detection2DArray()
        dets_msg.header = msg.header
        for i, det in enumerate(dets):
            boxes, score = det[:4], det[4]
            x1, y1, x2, y2 = boxes.astype(float)
            s = self.mask_scale - 1.0
            h, w = y2 - y1, x2 - x1
            y1 -= h * s
            y2 += h * s
            x1 -= w * s
            x2 += w * s

            det_msg = Detection2D()
            det_msg.header = dets_msg.header
            det_msg.bbox.center.x = (x1 + x2) / 2.0
            det_msg.bbox.center.y = (y1 + y2) / 2.0
            det_msg.bbox.size_x = w
            det_msg.bbox.size_y = h
            hyp = ObjectHypothesisWithPose()
            hyp.id = i
            hyp.score = score
            hyp.pose.pose.orientation.w = 1.0
            det_msg.results.append(hyp)
            dets_msg.detections.append(det_msg)
        return dets_msg

    def get_image_for_transport(self, msg, raw_msg, compressed_topic, transport):
        config = copy.deepcopy(self.received_transport_params.get(compressed_topic, {}))
        config.update(self.transport_params.get(transport, {}))

        compressed_fmt, compressed_depth_fmt, _ = guess_any_compressed_image_transport_format(msg)

        if compressed_fmt is not None and "format" not in config:
            config["format"] = compressed_fmt.format.value
        elif compressed_depth_fmt is not None and "format" not in config:
            config["format"] = compressed_depth_fmt.format.value

        compressed_msg, err = encode(raw_msg, compressed_topic, config)

        return compressed_msg, err

    def _str_params(self):
        parts = []
        parts.append('threshold=%f' % (self.threshold,))
        parts.append('ellipse=%r' % (self.ellipse,))
        parts.append('scale=%r' % (self.scale,))
        parts.append('mask_scale=%f' % (self.mask_scale,))
        parts.append('replacewith=%s' % (self.replacewith,))
        if self.replaceimg is not None:
            parts.append('replaceimg=%s' % (self.replaceimg,))
        parts.append('mosaicsize=%f' % (self.mosaicsize,))
        parts.append('backend=%s' % (self.backend,))
        parts.append('publish_faces=%r' % (self.publish_faces,))
        if len(self.transport_params) > 0:
            parts.append('transport_params=%r' % (self.transport_params,))
        parent_params = super(BlurFaces, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)
