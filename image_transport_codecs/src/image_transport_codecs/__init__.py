# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of images compressed with any image transport.

Example usage:

.. code-block:: python

    from image_transport_codecs import decode, encode
    from sensor_msgs.msg import CompressedImage, Image

    raw = Image()
    ... # fill the image
    compressed, err = encode(raw, "compressed")
    if compressed is None:
      rospy.logerr("Error encoding image: " + err)
      return False
    # work with the CompressedImage instance in variable compressed

    # beware, for decoding, we do not specify "raw", but the codec used for encoding
    raw2, err = decode(compressed, "compressed")
    if raw2 is None:
      rospy.logerr("Error encoding image: " + err)
      return False
    # work with the Image instance in variable raw2

    # or you can work directly with a particular codec if you know which one you want in advance:
    from image_transport_codecs import compressed_codec

    compressed2, err = compressed_codec.encode(raw)
"""

from ctypes import RTLD_GLOBAL, c_bool, c_uint8, c_uint32, c_char_p, c_size_t, POINTER, byref
import time

from dynamic_reconfigure.msg import Config, BoolParameter, DoubleParameter, IntParameter, StrParameter
from sensor_msgs.msg import CompressedImage, Image

from cras import get_msg_type
from cras.ctypes_utils import load_library, Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator, \
    get_ro_c_buffer
from cras.string_utils import STRING_TYPE, BufferStringIO


class CompressedImageContent(object):
    """The part of a compressed message that represents the actual image data (i.e. data that can be passed to an
    external decoder).

    It is not guaranteed for every codec that its encoded messages carry some standalone meaning. If there is no
    meaning, it will just produce empty content messages.

    :ivar format: Format of the image. This should be a string recognized by OpenCV, ffmpeg or similar tools.
    :type format: str
    :ivar data: The image content.
    :type data: list or bytearray or None
    """
    def __init__(self, format, data):
        self.format = format
        self.data = data


def dict_to_config(d):
    """Convert configuration dict to :class:`dynamic_reconfigure.msg.Config`.

    :param d: Configuration dict (or already the message, in which case it is just returned).
    :type d: dict or dynamic_reconfigure.msg.Config or None
    :return: The config message.
    :rtype: dynamic_reconfigure.msg.Config
    """
    if d is None:
        return Config()
    if isinstance(d, Config):
        return d
    c = Config()
    for key, value in d.items():
        if isinstance(value, bool):
            c.bools.append(BoolParameter(key, value))
        elif isinstance(value, float):
            c.doubles.append(DoubleParameter(key, value))
        elif isinstance(value, int):
            c.ints.append(IntParameter(key, value))
        elif isinstance(value, STRING_TYPE):
            c.strs.append(StrParameter(key, value))
    return c


__codec = None


def __get_library():
    global __codec
    if __codec is None:
        __codec = load_library('image_transport_codecs', mode=RTLD_GLOBAL)
        if __codec is None:
            return None

        # Add function signatures

        __codec.imageTransportCodecsEncode.restype = c_bool
        __codec.imageTransportCodecsEncode.argtypes = [
            c_char_p,
            c_uint32, c_uint32, c_char_p, c_uint8, c_uint32, c_size_t, POINTER(c_uint8),
            Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            c_size_t, POINTER(c_uint8),
            Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

        __codec.imageTransportCodecsDecode.restype = c_bool
        __codec.imageTransportCodecsDecode.argtypes = [
            c_char_p,
            c_char_p, c_char_p, c_size_t, POINTER(c_uint8),
            POINTER(c_uint32), POINTER(c_uint32), Allocator.ALLOCATOR, POINTER(c_uint8), POINTER(c_uint32),
            Allocator.ALLOCATOR,
            c_size_t, POINTER(c_uint8),
            Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

        __codec.getCompressedImageContents.restype = c_bool
        __codec.getCompressedImageContents.argtypes = [
            c_char_p,
            c_char_p, c_char_p, c_size_t, POINTER(c_uint8), c_char_p,
            POINTER(c_bool), Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

    return __codec


def encode(topic_or_codec, raw, config=None):
    """Encode the given raw image into a compressed image with a suitable codec.

    :param str topic_or_codec: Name of the topic where this image should be published or explicit name of the codec.
    :param sensor_msgs.msg.Image raw: The raw image.
    :param config: Configuration of the encoding process. You can use the same values as those offered by dynamic
                   reconfigure of the corresponding :roswiki:`image_transport` publisher.
    :type config: dict or dynamic_reconfigure.msg.Config or None
    :return: Tuple of compressed image and error string. If the compression fails (e.g. wrong image dimensions or bit
             depth), image is `None` and error string is filled.
    :rtype: (genpy.Message or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    config = dict_to_config(config)
    config_buf = BufferStringIO()
    config.serialize(config_buf)
    config_buf_len = config_buf.tell()
    config_buf.seek(0)

    type_allocator = StringAllocator()
    md5sum_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    args = [
        topic_or_codec.encode("utf-8"),
        raw.height, raw.width, raw.encoding.encode("utf-8"), raw.is_bigendian, raw.step, len(raw.data),
        get_ro_c_buffer(raw.data), type_allocator.get_cfunc(), md5sum_allocator.get_cfunc(), data_allocator.get_cfunc(),
        c_size_t(config_buf_len), get_ro_c_buffer(config_buf),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]

    ret = codec.imageTransportCodecsEncode(*args)

    log_allocator.print_log_messages()
    if ret:
        msg_type = get_msg_type(type_allocator.value)
        compressed = msg_type()
        if md5sum_allocator.value != compressed._md5sum:
            return None, "MD5 sum mismatch for %s: %s vs %s" % (
                type_allocator.value, md5sum_allocator.value, compressed._md5sum)
        compressed.deserialize(data_allocator.value)
        compressed.header = raw.header
        return compressed, ""
    return None, error_allocator.value


def decode(topic_or_codec, compressed, config=None):
    """Decode the given compressed image encoded with any codec into a raw image.

    :param str topic_or_codec: Name of the topic this image comes from or explicit name of the codec.
    :param genpy.Message compressed: The compressed image.
    :param config: Configuration of the decoding process.
    :type config: dict or dynamic_reconfigure.msg.Config or None
    :return: Tuple of raw image and error string. If the decoding fails, image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.Image or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    encoding_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    raw_height = c_uint32()
    raw_width = c_uint32()
    raw_is_big_endian = c_uint8()
    raw_step = c_uint32()

    compressed_buf = BufferStringIO()
    compressed.serialize(compressed_buf)
    compressed_buf_len = compressed_buf.tell()
    compressed_buf.seek(0)

    config = dict_to_config(config)
    config_buf = BufferStringIO()
    config.serialize(config_buf)
    config_buf_len = config_buf.tell()
    config_buf.seek(0)

    args = [
        topic_or_codec.encode("utf-8"),
        compressed._type.encode("utf-8"), compressed._md5sum.encode("utf-8"), compressed_buf_len,
        get_ro_c_buffer(compressed_buf),
        byref(raw_height), byref(raw_width), encoding_allocator.get_cfunc(), byref(raw_is_big_endian),
        byref(raw_step), data_allocator.get_cfunc(),
        c_size_t(config_buf_len), get_ro_c_buffer(config_buf),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]
    ret = codec.imageTransportCodecsDecode(*args)

    log_allocator.print_log_messages()
    if ret:
        raw = Image()
        raw.header = compressed.header
        raw.height = raw_height.value
        raw.width = raw_width.value
        raw.encoding = encoding_allocator.value
        raw.is_bigendian = raw_is_big_endian.value
        raw.step = raw_step.value
        raw.data = data_allocator.value
        return raw, ""
    return None, error_allocator.value


def get_compressed_image_content(compressed, topic_or_codec, match_format=""):
    """Return the part of the encoded message that represents the actual image data (i.e. the part that can be passed
    to external decoders or saved to a file). If the codec messages have no such meaning, empty result is returned.

    :param genpy.Message compressed: The compressed image.
    :param str topic_or_codec: Name of the topic this image comes from or explicit name of the codec.
    :param str match_format: If nonempty, the image data is only returned if their `format` field would match the given
                             one. The matching should be case-insensitive.
    :return: If it makes sense, the contained image bytes as first tuple member. If not, empty result. If an error
             occurred, it is reported in the second tuple member.
    :rtype: (CompressedImageContent or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    format_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    has_data = c_bool()

    compressed_buf = BufferStringIO()
    compressed.serialize(compressed_buf)
    compressed_buf_len = compressed_buf.tell()
    compressed_buf.seek(0)

    args = [
        topic_or_codec.encode("utf-8"),
        compressed._type.encode("utf-8"), compressed._md5sum.encode("utf-8"), compressed_buf_len,
        get_ro_c_buffer(compressed_buf), match_format.encode("utf-8"),
        byref(has_data), format_allocator.get_cfunc(), data_allocator.get_cfunc(),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]
    ret = codec.getCompressedImageContents(*args)

    log_allocator.print_log_messages()
    if ret:
        if not has_data.value:
            return None, ""
        return CompressedImageContent(format_allocator.value, data_allocator.value), ""
    return None, error_allocator.value


if __name__ == '__main__':
    def main():
        import rospy
        raw = Image()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = "test"
        raw.encoding = '16UC1'
        raw.step = 4
        raw.width = raw.height = 2
        # distances in millimeters
        import struct
        data = struct.pack('H', 0) + struct.pack('H', 1000) + struct.pack('H', 5000) + struct.pack('H', 9999)
        import sys
        if sys.version_info[0] == 2:
            raw.data = map(ord, data)

        raw = Image()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = "test"
        raw.encoding = 'rgb8'
        raw.width = 640
        raw.height = 480
        raw.step = raw.width*3
        # distances in millimeters
        raw.data = [i % 255 for i in range(raw.step*raw.height)]
        # raw.data = [0] * (raw.step*raw.height)
        # raw.data = b'\x00' * (raw.step*raw.height)
        
        # import rosbag
        # with rosbag.Bag("/media/data/subt/common/src/ros-utils/image_transport_codecs/test/data/raw.bag", 'r') as b:
        #     for _, raw, _ in b.read_messages(topics=["/spot/camera/hand_color/image"]):
        #         break

        rospy.init_node("test")
        rospy.loginfo("start")
        pub = rospy.Publisher("test/compressed", CompressedImage, queue_size=1, latch=True)
        pub2 = rospy.Publisher("test", Image, queue_size=1, latch=True)
        time.sleep(1)

        start = time.time()
        for i in range(100):
            # compressed = encode(pub.name, raw, {"depth_max": 10.0})
            compressed, err = encode(pub.name, raw)
    
            # print(bool(compressed))
            # if compressed is not None:
            #     pub.publish(compressed)
    
            raw2, err = decode(pub.name, compressed)

        end = time.time()
        print(end - start)
        print(bool(raw))
        print(bool(compressed))
        print(err)
        if compressed is not None:
            pub.publish(compressed)
        if raw is not None:
            pub2.publish(raw)

        rospy.spin()


    main()
