# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Methods for parsing the values of field `sensor_msgs::CompressedImage::format` for `compressed` and
`compressedDepth` codecs."""

from collections import namedtuple
from ctypes import RTLD_GLOBAL, c_bool, c_uint32, c_char_p, POINTER, byref
from enum import Enum

from compressed_image_transport.cfg import CompressedPublisherConfig
from compressed_depth_image_transport.cfg import CompressedDepthPublisherConfig
from sensor_msgs.msg import CompressedImage, Image

from cras.ctypes_utils import load_library, Allocator, StringAllocator


class CompressedTransportCompressionFormat(Enum):
    """Compression format of `compressed` codec (JPEG/PNG)."""

    JPEG = CompressedPublisherConfig.CompressedPublisher_jpeg
    """JPEG compression"""

    PNG = CompressedPublisherConfig.CompressedPublisher_png
    """PNG compression"""


class CompressedDepthTransportCompressionFormat(Enum):
    """Compression format of `compressedDepth` codec (PNG/RVL). RVL is only usable in Noetic."""

    PNG = getattr(CompressedDepthPublisherConfig, "CompressedDepthPublisher_png", "png")
    """PNG compression"""

    RVL = getattr(CompressedDepthPublisherConfig, "CompressedDepthPublisher_rvl", "rvl")
    """RVL compression (added in Noetic)"""


class CompressedTransportFormat(namedtuple(
    "CompressedTransportFormat",
        ("format", "format_string", "raw_encoding", "compressed_encoding", "num_channels", "bit_depth", "is_color"))):
    """Decoded meaning of field `sensor_msgs::CompressedImage::format` for `compressed` transport.

    :param CompressedTransportCompressionFormat format: The compression format (JPEG/PNG).
    :param str format_string: Text version of the compression format ("jpeg"/"png").
    :param str rawEncoding: Encoding of the raw image (before compression, after decompression).
    :param str compressedEncoding: Encoding of the compressed image (i.e. `bgr8` for JPEG).
    :param int numChannels: Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
    :param int bitDepth: Number of bits used for encoding one raw channel value.
    :param bool isColor: Whether the image is a color image or not.
    """
    __slots__ = ()  # prevent adding more fields


class CompressedDepthTransportFormat(namedtuple(
        "CompressedDepthTransportFormat", ("format", "format_string", "raw_encoding", "bit_depth"))):
    """Decoded meaning of field `sensor_msgs::CompressedImage::format` for `compressedDepth` transport.

    :param CompressedDepthTransportCompressionFormat format: The compression format (PNG/RVL).
    :param str format_string: Text version of the compression format ("png"/"rvl").
    :param str rawEncoding: Encoding of the raw image (before compression, after decompression).
    :param int bitDepth: Number of bits used for encoding one raw channel value.
    """
    __slots__ = ()  # prevent adding more fields


compressed_depth_has_rvl = hasattr(CompressedDepthPublisherConfig, "CompressedDepthPublisher_png")


__codec = None


def __get_library():
    global __codec
    if __codec is None:
        __codec = load_library('image_transport_codecs', mode=RTLD_GLOBAL)
        if __codec is None:
            return None

        # Add function signatures

        __codec.parseCompressedTransportFormat.restype = c_bool
        __codec.parseCompressedTransportFormat.argtypes = [
            c_char_p,
            Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            POINTER(c_uint32), POINTER(c_uint32), POINTER(c_bool),
            Allocator.ALLOCATOR,
        ]

        __codec.makeCompressedTransportFormat.restype = c_bool
        __codec.makeCompressedTransportFormat.argtypes = [
            c_char_p, c_char_p, c_char_p, c_uint32, c_uint32, c_bool,
            Allocator.ALLOCATOR,
            Allocator.ALLOCATOR,
        ]

        __codec.extractCompressedTransportFormat.restype = c_bool
        __codec.extractCompressedTransportFormat.argtypes = [
            c_char_p, c_char_p,
            Allocator.ALLOCATOR, POINTER(c_uint32), POINTER(c_uint32), POINTER(c_bool),
            Allocator.ALLOCATOR,
        ]

        __codec.parseCompressedDepthTransportFormat.restype = c_bool
        __codec.parseCompressedDepthTransportFormat.argtypes = [
            c_char_p,
            Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            POINTER(c_uint32),
            Allocator.ALLOCATOR,
        ]

        __codec.makeCompressedDepthTransportFormat.restype = c_bool
        __codec.makeCompressedDepthTransportFormat.argtypes = [
            c_char_p, c_char_p, c_uint32,
            Allocator.ALLOCATOR,
            Allocator.ALLOCATOR,
        ]

        __codec.extractCompressedDepthTransportFormat.restype = c_bool
        __codec.extractCompressedDepthTransportFormat.argtypes = [
            c_char_p, c_char_p,
            POINTER(c_uint32),
            Allocator.ALLOCATOR,
        ]

    return __codec


def parse_compressed_transport_format(format):
    """Parse the string from field `sensor_msgs::CompressedImage::format` using `compressed` transport into
    :py:class:`CompressedTransportFormat` structure.

    :param format: The `format` field text, or a :sensor_msgs:`CompressedImage`.
    :type format: str or sensor_msgs.msg.CompressedImage
    :return: Tuple of the parsed structure or error string. If the parsing fails, structure is `None` and error string
             is filled.
    :rtype: (CompressedTransportFormat or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if isinstance(format, CompressedImage):
        format = format.format

    compression_format_allocator = StringAllocator()
    raw_encoding_allocator = StringAllocator()
    compressed_encoding_allocator = StringAllocator()
    error_allocator = StringAllocator()

    num_channels = c_uint32()
    bit_depth = c_uint32()
    is_color = c_bool()

    args = [
        format,
        compression_format_allocator.get_cfunc(), raw_encoding_allocator.get_cfunc(),
        compressed_encoding_allocator.get_cfunc(), byref(num_channels), byref(bit_depth), byref(is_color),
        error_allocator.get_cfunc(),
    ]

    ret = codec.parseCompressedTransportFormat(*args)

    if ret:
        return CompressedTransportFormat(
            CompressedTransportCompressionFormat(compression_format_allocator.value),
            compression_format_allocator.value, raw_encoding_allocator.value, compressed_encoding_allocator.value,
            int(num_channels.value), int(bit_depth.value), is_color.value), ""
    return None, error_allocator.value


def make_compressed_transport_format(format):
    """Convert the :py:class:`CompressedTransportFormat` structure into a string to be filled in field
    `sensor_msgs::CompressedImage::format` of `compressed` transport image.

    :param CompressedTransportFormat format: The format to convert.
    :return: The string for the `format` field.
    :rtype: (str or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    format_allocator = StringAllocator()
    error_allocator = StringAllocator()

    args = [
        format.format_string, format.raw_encoding, format.compressed_encoding, format.num_channels, format.bit_depth,
        format.is_color,
        format_allocator.get_cfunc(),
        error_allocator.get_cfunc(),
    ]

    ret = codec.makeCompressedTransportFormat(*args)

    if ret:
        return format_allocator.value, ""
    return None, error_allocator.value


def extract_compressed_transport_format(image_encoding, compression_format):
    """Create the :py:class:`CompressedTransportFormat` structure for the given raw image compressed with the given
    method.

    :param image_encoding: `encoding` field of the raw image, or a :sensor_msgs:`Image`.
    :type image_encoding: str or sensor_msgs.msg.Image
    :param CompressedTransportCompressionFormat compression_format: The target compression method.
    :return: Tuple of the parsed structure or error string. If the parsing fails, structure is `None` and error string
             is filled.
    :rtype: (CompressedTransportFormat or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if isinstance(image_encoding, Image):
        image_encoding = image_encoding.encoding

    compressed_encoding_allocator = StringAllocator()
    error_allocator = StringAllocator()

    num_channels = c_uint32()
    bit_depth = c_uint32()
    is_color = c_bool()

    args = [
        image_encoding, compression_format.value,
        compressed_encoding_allocator.get_cfunc(), byref(num_channels), byref(bit_depth), byref(is_color),
        error_allocator.get_cfunc(),
    ]

    ret = codec.extractCompressedTransportFormat(*args)

    if ret:
        return CompressedTransportFormat(
            compression_format, compression_format.value, image_encoding, compressed_encoding_allocator.value,
            int(num_channels.value), int(bit_depth.value), is_color.value), ""
    return None, error_allocator.value


def parse_compressed_depth_transport_format(format):
    """Parse the string from field `sensor_msgs::CompressedImage::format` using `compressedDepth` transport into
    :py:class:`CompressedTransportFormat` structure.

    :param format: The `format` field text, or a :sensor_msgs:`CompressedImage`.
    :type format: str or sensor_msgs.msg.CompressedImage
    :return: Tuple of the parsed structure or error string. If the parsing fails, structure is `None` and error string
             is filled.
    :rtype: (CompressedTransportFormat or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if isinstance(format, CompressedImage):
        format = format.format

    compression_format_allocator = StringAllocator()
    raw_encoding_allocator = StringAllocator()
    error_allocator = StringAllocator()

    bit_depth = c_uint32()

    args = [
        format,
        compression_format_allocator.get_cfunc(), raw_encoding_allocator.get_cfunc(), byref(bit_depth),
        error_allocator.get_cfunc(),
    ]

    ret = codec.parseCompressedDepthTransportFormat(*args)

    if ret:
        return CompressedDepthTransportFormat(
            CompressedDepthTransportCompressionFormat(compression_format_allocator.value),
            compression_format_allocator.value, raw_encoding_allocator.value, int(bit_depth.value)), ""
    return None, error_allocator.value


def make_compressed_depth_transport_format(format):
    """Convert the :py:class:`CompressedDepthTransportFormat` structure into a string to be filled in field
    `sensor_msgs::CompressedImage::format` of `compressedDepth` transport image.

    :param CompressedDepthTransportFormat format: The format to convert.
    :return: The string for the `format` field.
    :rtype: (str or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    format_allocator = StringAllocator()
    error_allocator = StringAllocator()

    args = [
        format.format_string, format.raw_encoding, format.bit_depth,
        format_allocator.get_cfunc(),
        error_allocator.get_cfunc(),
    ]

    ret = codec.makeCompressedDepthTransportFormat(*args)

    if ret:
        return format_allocator.value, ""
    return None, error_allocator.value


def extract_compressed_depth_transport_format(image_encoding, compression_format):
    """Create the :py:class:`CompressedDepthTransportFormat` structure for the given raw image compressed with the given
    method.

    :param image_encoding: `encoding` field of the raw image, or a :sensor_msgs:`Image`.
    :type image_encoding: str or sensor_msgs.msg.Image
    :param CompressedDepthTransportCompressionFormat compression_format: The target compression method.
    :return: Tuple of the parsed structure or error string. If the parsing fails, structure is `None` and error string
             is filled.
    :rtype: (CompressedDepthTransportFormat or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if isinstance(image_encoding, Image):
        image_encoding = image_encoding.encoding

    error_allocator = StringAllocator()

    bit_depth = c_uint32()

    args = [
        image_encoding, compression_format.value,
        byref(bit_depth),
        error_allocator.get_cfunc(),
    ]

    ret = codec.extractCompressedDepthTransportFormat(*args)

    if ret:
        return CompressedDepthTransportFormat(
            compression_format, compression_format.value, image_encoding, int(bit_depth.value)), ""
    return None, error_allocator.value


if __name__ == '__main__':
    def main():
        print(parse_compressed_transport_format("bgr8; jpeg compressed bgr8"))
        print(parse_compressed_transport_format(CompressedImage(format="bgr8; jpeg compressed bgr8")))
        print(extract_compressed_transport_format("bgr8", CompressedTransportCompressionFormat.JPEG))
        print(extract_compressed_transport_format(Image(encoding="bgr8"), CompressedTransportCompressionFormat.JPEG))
        print(make_compressed_transport_format(CompressedTransportFormat(
            CompressedTransportCompressionFormat.JPEG, "jpeg", "bgr8", "bgr8", 3, 8, True)))
        print(parse_compressed_transport_format("mono8; png compressed "))
        print(parse_compressed_transport_format(CompressedImage(format="mono8; png compressed ")))
        print(extract_compressed_transport_format("mono8", CompressedTransportCompressionFormat.PNG))
        print(extract_compressed_transport_format(Image(encoding="mono8"), CompressedTransportCompressionFormat.PNG))
        print(make_compressed_transport_format(CompressedTransportFormat(
            CompressedTransportCompressionFormat.PNG, "png", "mono8", "mono8", 1, 8, False)))
        print(parse_compressed_transport_format("16UC1; png compressed "))
        print(parse_compressed_transport_format(CompressedImage(format="16UC1; png compressed ")))
        print(extract_compressed_transport_format("16UC1", CompressedTransportCompressionFormat.PNG))
        print(extract_compressed_transport_format(Image(encoding="16UC1"), CompressedTransportCompressionFormat.PNG))
        print(make_compressed_transport_format(CompressedTransportFormat(
            CompressedTransportCompressionFormat.PNG, "png", "16UC1", "16UC1", 1, 16, False)))
        print(parse_compressed_transport_format("invalid"))

        print(parse_compressed_depth_transport_format("16UC1; compressedDepth"))
        print(parse_compressed_depth_transport_format(CompressedImage(format="16UC1; compressedDepth")))
        print(extract_compressed_depth_transport_format("16UC1", CompressedDepthTransportCompressionFormat.PNG))
        print(extract_compressed_depth_transport_format(Image(encoding="16UC1"),
                                                        CompressedDepthTransportCompressionFormat.PNG))
        print(make_compressed_depth_transport_format(CompressedDepthTransportFormat(
            CompressedDepthTransportCompressionFormat.PNG, "png", "16UC1", 16)))
        print(parse_compressed_depth_transport_format("16UC1; compressedDepth rvl"))
        print(parse_compressed_depth_transport_format(CompressedImage(format="16UC1; compressedDepth rvl")))
        print(extract_compressed_depth_transport_format("16UC1", CompressedDepthTransportCompressionFormat.RVL))
        print(extract_compressed_depth_transport_format(Image(encoding="16UC1"),
                                                        CompressedDepthTransportCompressionFormat.RVL))
        print(make_compressed_depth_transport_format(CompressedDepthTransportFormat(
            CompressedDepthTransportCompressionFormat.RVL, "rvl", "16UC1", 16)))
        print(parse_compressed_depth_transport_format("invalid"))


    main()
