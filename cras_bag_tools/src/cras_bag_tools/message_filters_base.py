# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Implementations of common message filters."""

from __future__ import absolute_import, division, print_function

import copy
import csv
import sys
from typing import Any, Iterable, List, Optional, Tuple, Union

import genpy
import rospy
from cras.message_utils import raw_to_msg, msg_to_raw
from cras.string_utils import STRING_TYPE
from dynamic_reconfigure.encoding import decode_config
from dynamic_reconfigure.msg import Config
from image_transport_codecs import decode, encode
from image_transport_codecs.parse_compressed_format import guess_any_compressed_image_transport_format
from sensor_msgs.msg import CompressedImage, Image

from .message_filter import ConnectionHeader, DeserializedMessageData, DeserializedMessageFilter, MessageTags, Tags, \
    UniversalFilter

STR = STRING_TYPE
FilteredImage = Tuple[STR, STR, genpy.Message, genpy.Message, STR, rospy.Time, ConnectionHeader, Tags]
FilteredImageOrAnyMsg = Union[FilteredImage, DeserializedMessageData]


class ImageTransportFilter(UniversalFilter):
    """Base filter subscribing either Image or CompressedImage images and calling callbacks always on Image type."""

    def __init__(self, is_raw, include_topics=None, include_types=None, skip_invalid=True, recompress=True,
                 transport_params=None, ignore_transports=None, add_tags_to_invalid=None, *args, **kwargs):
        """
        :param list include_topics: Image topics to work on.
        :param list include_types: Types of messages to work on. The default is sensor_msgs/Image and CompressedImage.
        :param bool skip_invalid: Whether images should be skipped if (de)compression fails (otherwise they are passed
                                  through unchanged).
        :param bool recompress: Whether the images should be recompressed back to CompressedImage after being filtered.
        :param dict transport_params: Parameters of image transport(s). Keys are transport names (e.g. 'compressed'),
                                      values are the publisher dynamic reconfigure parameters.
        :param list ignore_transports: List of transport names to ignore.
        :param set add_tags_to_invalid: Tags to be added to images that failed (de)compression and are passed through
                                        (only with skip_invalid == False).
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        default_types = [Image._type, CompressedImage._type, Config._type]

        _include_topics = None
        if include_topics is not None:
            _include_topics = [t + '/parameter_updates' for t in include_topics] + list(include_topics)

        _include_types = default_types
        if include_types is not None:
            _include_types += list(include_types)

        super(ImageTransportFilter, self).__init__(
            is_raw, include_topics=_include_topics, include_types=_include_types, *args, **kwargs)

        self.skip_invalid = skip_invalid
        self.recompress = recompress
        self.transport_params = dict(transport_params) if transport_params is not None else {}
        self.add_tags_to_invalid = add_tags_to_invalid
        self.ignore_transports = ['/' + t for t in ignore_transports] if ignore_transports is not None else []

        self._received_transport_params = {}

    def consider_message(self, topic, datatype, stamp, header, tags):
        if datatype == Config._type:
            return True

        if not super(ImageTransportFilter, self).consider_message(topic, datatype, stamp, header, tags):
            return False

        for t in self.ignore_transports:
            if topic.endswith(t):
                return False

        return True

    def process_transport_params(self, topic, msg):
        if not topic.endswith('/parameter_updates'):
            return
        transport_topic, _ = topic.rsplit('/', 1)
        self._received_transport_params[transport_topic] = decode_config(msg)

    def filter_raw(self, topic, datatype, data, md5sum, pytype, stamp, header, tags):
        if datatype == Config._type:
            self.process_transport_params(topic, raw_to_msg(datatype, data, md5sum, pytype))
            return topic, datatype, data, md5sum, pytype, stamp, header, tags

        result = self.filter_any_image(topic, raw_to_msg(datatype, data, md5sum, pytype), stamp, header, tags)
        if result is None:
            return None
        if not isinstance(result, list):
            result = [result]

        final_result = []
        for _result in result:
            if _result is not None:
                _topic, _msg, _stamp, _header, _tags = _result
                _datatype, _data, _md5sum, _pytype = msg_to_raw(_msg)
                final_result.append((_topic, _datatype, _data, _md5sum, _pytype, _stamp, _header, _tags))
            else:
                final_result.append(None)

        return final_result

    def filter_deserialized(self, topic, msg, stamp, header, tags):
        if msg._type == Config._type:
            self.process_transport_params(topic, msg)
            return topic, msg, stamp, header, tags
        return self.filter_any_image(topic, msg, stamp, header, tags)

    def filter_any_image(self, topic, orig_msg, stamp, header, tags):
        if orig_msg._type == Image._type:
            raw_msg = orig_msg
            raw_topic = topic
            transport = 'raw'
        else:
            transport = None
            if "/" in topic:
                raw_topic, transport = topic.rsplit("/", 1)

            if transport is None or len(transport) == 0:
                orig_msg = "Compressed image on a topic without suffix [%s]. %s message."
                if self.skip_invalid:
                    print(orig_msg % (topic, "Skipping"), file=sys.stderr)
                    return None
                else:
                    print(orig_msg % (topic, "Passing unfiltered"), file=sys.stderr)
                    if self.add_tags_to_invalid is not None:
                        tags = tags.union(self.add_tags_to_invalid)
                    return topic, orig_msg, stamp, header, tags

            raw_msg, err = decode(orig_msg, topic, {})
            if raw_msg is None:
                orig_msg = 'Error decompressing image on topic [%s]: %s. %s message.'
                if self.skip_invalid:
                    print(orig_msg % (topic, str(err), "Skipping"), file=sys.stderr)
                    return None
                else:
                    print(orig_msg % (topic, str(err), "Passing unfiltered"), file=sys.stderr)
                    if self.add_tags_to_invalid is not None:
                        tags = tags.union(self.add_tags_to_invalid)
                    return topic, orig_msg, stamp, header, tags

        result = self.filter_image(topic, orig_msg, raw_msg, raw_topic, transport, stamp, header, tags)
        if result is None:
            return None
        if not isinstance(result, list):
            result = [result]

        final_result = []
        for _result in result:
            if _result is None or len(_result) != 8:
                final_result.append(_result)
            else:
                _topic, _orig_msg, _raw_msg, _raw_topic, _transport, _stamp, _header, _tags = _result
                if _topic == _raw_topic or _transport == "raw" or not self.recompress:
                    final_result.append((_raw_topic, _raw_msg, _stamp, _header, _tags))
                else:
                    msg, err = self.get_image_for_transport(_orig_msg, _raw_msg, _topic, _transport)
                    if msg is None:
                        print("Error compressing image on topic [%s]: %s. Skipping message." % (topic, err),
                              file=sys.stderr)
                        continue
                    final_result.append((_topic, msg, _stamp, _header, _tags))

        return final_result

    def filter_image(self,
                     topic,  # type: STRING_TYPE
                     orig_msg,  # type: genpy.Message
                     raw_msg,  # type: genpy.Message
                     raw_topic,  # type: STRING_TYPE
                     transport,  # type: STRING_TYPE
                     stamp,  # type: rospy.Time
                     header,  # type: ConnectionHeader
                     tags  # type: Tags
                     ):
        # type: (...) -> Optional[Union[FilteredImageOrAnyMsg, List[FilteredImageOrAnyMsg]]]
        """Filter the decoded image.

        :param topic: The topic on which the encoded image arrived.
        :param orig_msg: The encoded image.
        :param raw_msg: The decoded image.
        :param raw_topic: The corresponding raw image topic.
        :param transport: The detected image transport.
        :param stamp: Receive timestamp of the original message.
        :param header: Connection header.
        :param tags: Message tags.
        :return: None if the message should be discarded. Otherwise, a list of filtered messages, or just one message.
                 The filter can return a mixture of two types of messages: 8-tuples are image-transport-based images and
                 5-tuples are classical deserialized messages.
        """
        raise NotImplemented

    def get_image_for_transport(self, orig_msg, raw_msg, compressed_topic, transport):
        config = copy.deepcopy(self._received_transport_params.get(compressed_topic, {}))
        config.update(self.transport_params.get(transport, {}))

        compressed_fmt, compressed_depth_fmt, _ = guess_any_compressed_image_transport_format(orig_msg)

        if compressed_fmt is not None and "format" not in config:
            config["format"] = compressed_fmt.format.value
        elif compressed_depth_fmt is not None and "format" not in config:
            config["format"] = compressed_depth_fmt.format.value

        compressed_msg, err = encode(raw_msg, compressed_topic, config)

        return compressed_msg, err

    def _str_params(self):
        parts = []
        parts.append('skip_invalid=%r' % (self.skip_invalid,))
        if len(self.transport_params) > 0:
            parts.append('transport_params=%r' % (self.transport_params,))
        if len(self.ignore_transports) > 0:
            parts.append('ignore_transports=%r' % ([t.lstrip('/') for t in self.ignore_transports],))
        parent_params = super(ImageTransportFilter, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


class MessageToCSVExporterBase(DeserializedMessageFilter):
    """Export messages to a CSV file."""

    def __init__(self, csv_file, max_frequency=None, frequency_from_header_stamp=False, *args, **kwargs):
        # type: (STRING_TYPE, Optional[float], bool, Any, Any) -> None
        """
        :param csv_file: Path to the CSV file.
        :param max_frequency: The maximum frequency on which the CSV data should be exported.
        :param frequency_from_header_stamp: If True, the header stamp will be used as the timestamp for frequency
                                            checking. If False, receive stamp is used.
        :param args: Standard include/exclude and stamp args.
        :param kwargs: Standard include/exclude and stamp kwargs.
        """
        super(MessageToCSVExporterBase, self).__init__(*args, **kwargs)  # noqa

        self._min_dt = rospy.Duration(1.0 / float(max_frequency)) if max_frequency is not None else None
        self._csv_file_path = csv_file
        self._frequency_from_header_stamp = frequency_from_header_stamp

        self._csv_rows = []
        self._last_msg_time = None

    def _get_fields(self):
        # type: () -> Iterable[STRING_TYPE]
        raise NotImplementedError()

    def _msg_to_csv_row(self, topic, msg, stamp, header, tags):
        # type: (STRING_TYPE, genpy.Message, rospy.Time, ConnectionHeader, Tags) -> Optional[Iterable[STRING_TYPE]]
        raise NotImplementedError()

    def on_filtering_start(self):
        self._csv_rows = []

    def on_filtering_end(self):
        csv_file = self.resolve_file(self._csv_file_path)
        with open(csv_file, 'w', newline='') as f:
            fields = tuple(self._get_fields())
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for row in self._csv_rows:
                assert len(row) == len(fields)
                writer.writerow(dict(zip(fields, row)))
        print("Saved CSV with", len(self._csv_rows), "rows:", csv_file)

    def filter(self, topic, msg, stamp, header, tags):
        if self._min_dt is not None:
            msg_stamp = msg.header.stamp if self._frequency_from_header_stamp else stamp
            if self._last_msg_time is not None and self._last_msg_time + self._min_dt > msg_stamp:
                return topic, msg, stamp, header, tags
            self._last_msg_time = msg_stamp

        csv_row = self._msg_to_csv_row(topic, msg, stamp, header, tags)
        if csv_row is not None:
            self._csv_rows.append(tuple(csv_row))

        return topic, msg, stamp, header, tags

    def reset(self):
        super(MessageToCSVExporterBase, self).reset()
        self._last_msg_time = None
        self._csv_rows = []

    def _str_params(self):
        parts = []
        parts.append('csv_file=' + self.resolve_file(self._csv_file_path))
        parent_params = self._default_str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)


__all__ = [
    ImageTransportFilter.__name__,
    MessageToCSVExporterBase.__name__,
]
