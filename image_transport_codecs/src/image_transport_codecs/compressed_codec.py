# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of images compressed with the 'compressed' transport."""

from ctypes import RTLD_GLOBAL, c_bool, c_uint8, c_uint32, c_int, c_char_p, c_size_t, POINTER, byref
import time

from compressed_image_transport.cfg import CompressedPublisherConfig, CompressedSubscriberConfig
from sensor_msgs.msg import CompressedImage, Image

from cras.ctypes_utils import load_library, Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator,\
    get_ro_c_buffer


__codec = None


def __get_library():
    global __codec
    if __codec is None:
        if load_library('image_transport_codecs', mode=RTLD_GLOBAL) is not None:
            __codec = load_library('compressed_codec')
            if __codec is None:
                return None

            # Add function signatures

            __codec.compressed_codec_has_extra_jpeg_options.restype = c_bool

            __codec.compressedCodecEncode.restype = c_bool
            __codec.compressedCodecEncode.argtypes = [
                c_uint32, c_uint32, c_char_p, c_uint8, c_uint32, c_size_t, POINTER(c_uint8),
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
                c_char_p, c_int,
            ]
            if __codec.compressed_codec_has_extra_jpeg_options():
                __codec.compressedCodecEncode.argtypes += [c_bool, c_bool, c_int]
            __codec.compressedCodecEncode.argtypes += [c_int, Allocator.ALLOCATOR, Allocator.ALLOCATOR]

            __codec.compressedCodecDecode.restype = c_bool
            __codec.compressedCodecDecode.argtypes = [
                c_char_p, c_size_t, POINTER(c_uint8),
                POINTER(c_uint32), POINTER(c_uint32), Allocator.ALLOCATOR, POINTER(c_uint8), POINTER(c_uint32),
                Allocator.ALLOCATOR,
                c_char_p,
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            ]

    return __codec


def encode(raw, config=None):
    """Encode the given raw image into a :sensor_msgs:`CompressedImage` with "compressed" codec.

    :param sensor_msgs.msg.Image raw: The raw image.
    :param dict config: Configuration of the encoding process. You can use the same values as those offered by dynamic
                        reconfigure of the :roswiki:`compressed_image_transport` publisher.
    :return: Tuple of compressed image and error string. If the compression fails (e.g. wrong image dimensions or bit
             depth), the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.CompressedImage or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if config is None:
        config = {}

    default_config = CompressedPublisherConfig.defaults
    config_format = config.get("format", default_config["format"])
    config_jpeg_quality = config.get("jpeg_quality", default_config["jpeg_quality"])
    config_png_level = config.get("png_level", default_config["png_level"])
    config_jpeg_progressive = config.get("jpeg_progressive", default_config.get("jpeg_progressive", False))
    config_jpeg_optimize = config.get("jpeg_optimize", default_config.get("jpeg_optimize", False))
    config_jpeg_restart_interval = config.get("jpeg_restart_interval", default_config.get("jpeg_restart_interval", 0))

    format_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    args = [
        raw.height, raw.width, raw.encoding, raw.is_bigendian, raw.step, len(raw.data), get_ro_c_buffer(raw.data),
        format_allocator.get_cfunc(), data_allocator.get_cfunc(),
        config_format, config_jpeg_quality,
    ]
    if codec.compressed_codec_has_extra_jpeg_options():
        args += [config_jpeg_progressive, config_jpeg_optimize, config_jpeg_restart_interval]
    args += [config_png_level, error_allocator.get_cfunc(), log_allocator.get_cfunc()]

    ret = codec.compressedCodecEncode(*args)

    log_allocator.print_log_messages()
    if ret:
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format = format_allocator.value
        compressed.data = data_allocator.value
        return compressed, ""
    return None, error_allocator.value


def decode(compressed, config=None):
    """Decode the given :sensor_msgs:`CompressedImage` encoded with "compressedDepth" codec into a raw image.

    :param sensor_msgs.msg.CompressedImage compressed: The compressed image.
    :param dict config: Configuration of the decoding process. You can use the same values as those offered by dynamic
                        reconfigure of the :roswiki:`compressed_image_transport` subscriber.
    :return: Tuple of raw image and error string. If decoding failed, the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.Image or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if config is None:
        config = {}

    default_config = CompressedSubscriberConfig.defaults
    config_mode = config.get("mode", default_config["mode"])

    encoding_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    raw_height = c_uint32()
    raw_width = c_uint32()
    raw_is_big_endian = c_uint8()
    raw_step = c_uint32()

    args = [
        compressed.format, len(compressed.data), get_ro_c_buffer(compressed.data),
        byref(raw_height), byref(raw_width), encoding_allocator.get_cfunc(), byref(raw_is_big_endian), byref(raw_step),
        data_allocator.get_cfunc(),
        config_mode,
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]

    ret = codec.compressedCodecDecode(*args)

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


def has_extra_jpeg_options():
    """Return whether the JPEG encoder has options `jpeg_progressive`, `jpeg_optimize` and `jpeg_restart_interval`.

    :return: Whether the JPEG encoder has options `jpeg_progressive`, `jpeg_optimize` and `jpeg_restart_interval`.
    :rtype: bool
    """
    codec = __get_library()
    if codec is None:
        return False
    return codec.compressed_codec_has_extra_jpeg_options()


if __name__ == '__main__':
    def main():
        import rospy
        raw = Image()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = "test"
        raw.encoding = 'bgr8'
        raw.step = 6
        raw.width = raw.height = 2
        raw.data = [0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255]

        rospy.init_node("test")
        pub = rospy.Publisher("test/compressed", CompressedImage, queue_size=1, latch=True)
        pub2 = rospy.Publisher("test", Image, queue_size=1, latch=True)
        time.sleep(1)

        compressed, err = encode(raw, {"jpeg_quality": 80})

        print(bool(compressed), err)
        if compressed is not None:
            pub.publish(compressed)

        raw, err = decode(compressed)

        print(bool(raw), err)
        if raw is not None:
            pub2.publish(raw)

        rospy.spin()


    main()
