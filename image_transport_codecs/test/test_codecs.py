#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for image_transport_codecs."""

import os
import struct
import unittest
import sys

from compressed_image_transport.cfg.CompressedPublisherConfig import CompressedPublisher_jpeg, CompressedPublisher_png
from rosbag import Bag
from sensor_msgs.msg import CompressedImage, Image

from image_transport_codecs import compressed_codec
from image_transport_codecs import compressed_depth_codec
from image_transport_codecs import decode, encode, get_compressed_image_content, CompressedImageContent


def _byte(b):
    if sys.version_info[0] == 2:
        return ord(b)
    return b


def bytes_to_float(b):
    if len(b) == 4:
        return struct.unpack('<f', b)[0]
    return struct.unpack('<%if' % (len(b) / 4,), b)


def float_to_bytes(f):
    if isinstance(f, float):
        return bytes(struct.pack('<f',  f))
    return bytes(struct.pack('<%if' % (len(f),), *f))


class CodecsTest(unittest.TestCase):

    def test_raw(self):
        raw = Image()
        raw.header.frame_id = "test"
        raw.header.stamp.secs = 10
        raw.width = 2
        raw.height = 2
        raw.step = 2
        raw.encoding = "mono8"
        raw.is_bigendian = False
        raw.data = b'\x01\x02\x03\x04'

        compressed, err = encode(raw, "raw")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, Image)
        self.assertEqual(compressed, raw)

        raw2, err = decode(compressed, "raw")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2, raw)

        content, err = get_compressed_image_content(compressed, "raw")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "raw")
        self.assertEqual(content.data, raw.data)

        content, err = get_compressed_image_content(compressed, "raw", "raw")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "raw")
        self.assertEqual(content.data, raw.data)

        content, err = get_compressed_image_content(compressed, "raw", "jpg")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_jpeg(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "bgr8"
        raw.width = raw.height = 2
        raw.step = 6
        raw.data = b'\x00\x00\x00\x64\x64\x64\xc8\xc8\xc8\xff\xff\xff'

        compressed, err = encode(raw, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "bgr8; jpeg compressed bgr8")

        raw2, err = decode(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(len(raw.data)):
            self.assertLess(abs(_byte(raw2.data[i]) - _byte(raw.data[i])), 20)

        compressed2, err = encode(raw, "compressed", {"jpeg_quality": 50})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed2)
        self.assertIsInstance(compressed2, CompressedImage)
        self.assertEqual(compressed2.header, raw.header)
        self.assertEqual(compressed2.format, "bgr8; jpeg compressed bgr8")
        self.assertNotEqual(compressed2.data, compressed.data)

        if compressed_codec.has_extra_jpeg_options():
            compressed3, err = encode(raw, "compressed", {"jpeg_quality": 50, "jpeg_progressive": True})
            self.assertEqual(err, "")
            self.assertIsNotNone(compressed3)
            self.assertIsInstance(compressed3, CompressedImage)
            self.assertEqual(compressed3.header, raw.header)
            self.assertEqual(compressed3.format, "bgr8; jpeg compressed bgr8")
            self.assertNotEqual(compressed3.data, compressed.data)
            self.assertNotEqual(compressed3.data, compressed2.data)

        content, err = get_compressed_image_content(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "jpeg")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "compressed", "jpeg")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "jpeg")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "compressed", "jp2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_png(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "bgr8"
        raw.width = raw.height = 2
        raw.step = 6
        raw.data = b'\x00\x00\x00\x64\x64\x64\xc8\xc8\xc8\xff\xff\xff'

        compressed, err = encode(raw, "compressed", {"format": CompressedPublisher_png})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "bgr8; png compressed bgr8")

        raw2, err = decode(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        for i in range(len(raw.data)):
            self.assertEqual(raw2.data[i], raw.data[i])

        content, err = get_compressed_image_content(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "png")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "compressed", "png")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "png")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "compressed", "pn2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_depth_inv(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "32FC1"
        raw.width = raw.height = 2
        raw.step = 8
        raw.data = float_to_bytes((1.0, 2.0, 3.0, 4.0))

        compressed, err = encode(raw, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        if compressed_depth_codec.has_rvl():
            self.assertEqual(compressed.format, "32FC1; compressedDepth png")
        else:
            self.assertEqual(compressed.format, "32FC1; compressedDepth")

        raw2, err = decode(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(0, len(raw.data), 4):
            val1 = bytes_to_float(raw.data[i:(i + 4)])
            val2 = bytes_to_float(raw2.data[i:(i + 4)])
            self.assertAlmostEqual(val2, val1, delta=1e-3)

        content, err = get_compressed_image_content(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "png")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "png")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "png")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "pn2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_depth_uc(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "16UC1"
        raw.width = raw.height = 2
        raw.step = 4
        raw.data = b'\x00\x01\x00\x02\x00\x03\x00\x04'

        compressed, err = encode(raw, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        if compressed_depth_codec.has_rvl():
            self.assertEqual(compressed.format, "16UC1; compressedDepth png")
        else:
            self.assertEqual(compressed.format, "16UC1; compressedDepth")

        raw2, err = decode(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        for i in range(0, len(raw.data), 4):
            val1 = bytes_to_float(raw.data[i:(i + 4)])
            val2 = bytes_to_float(raw2.data[i:(i + 4)])
            # 16UC1 compression is lossless
            self.assertEqual(val2, val1)

        content, err = get_compressed_image_content(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "png")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "png")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "png")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "pn2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_depth_inv_rvl(self):
        if not compressed_depth_codec.has_rvl():
            self.skipTest("RVL codec is not available")

        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "32FC1"
        raw.width = raw.height = 2
        raw.step = 8
        raw.data = float_to_bytes((1.0, 2.0, 3.0, 4.0))

        compressed, err = encode(raw, "compressedDepth", {"format": "rvl"})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "32FC1; compressedDepth rvl")

        raw2, err = decode(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(0, len(raw.data), 4):
            val1 = bytes_to_float(raw.data[i:(i + 4)])
            val2 = bytes_to_float(raw2.data[i:(i + 4)])
            self.assertAlmostEqual(val2, val1, delta=1e-3)

        content, err = get_compressed_image_content(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "rvl")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "rvl")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "rvl")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "rv2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_depth_uc_rvl(self):
        if not compressed_depth_codec.has_rvl():
            self.skipTest("RVL codec is not available")

        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "16UC1"
        raw.width = raw.height = 2
        raw.step = 4
        raw.data = b'\x00\x01\x00\x02\x00\x03\x00\x04'

        compressed, err = encode(raw, "compressedDepth", {"format": "rvl"})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "16UC1; compressedDepth rvl")

        raw2, err = decode(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        for i in range(0, len(raw.data), 4):
            val1 = bytes_to_float(raw.data[i:(i + 4)])
            val2 = bytes_to_float(raw2.data[i:(i + 4)])
            # 16UC1 compression is lossless
            self.assertEqual(val2, val1)

        content, err = get_compressed_image_content(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "rvl")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "rvl")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "rvl")
        self.assertEqual(content.data, compressed.data[12:])

        content, err = get_compressed_image_content(compressed, "compressedDepth", "rv2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_compressed_wrong_type(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "32FC1"
        raw.width = raw.height = 2
        raw.step = 8
        raw.data = float_to_bytes((1.0, 2.0, 3.0, 4.0))

        compressed, err = encode(raw, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)

        raw2, err = decode(compressed, "wrong")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        raw2, err = decode(compressed, "compressed")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "bgr8; jpeg compressed bgr8"
        raw2, err = decode(compressed, "compressed")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "bgr8; png compressed bgr8"
        raw2, err = decode(compressed, "compressed")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "bgr8; compressed bgr8"
        raw2, err = decode(compressed, "compressed")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

    def test_compressed_wrong_type2(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "bgr8"
        raw.width = raw.height = 2
        raw.step = 6
        raw.data = b'\x00\x00\x00\x64\x64\x64\xc8\xc8\xc8\xff\xff\xff'

        compressed, err = encode(raw, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)

        raw2, err = decode(compressed, "wrong")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        raw2, err = decode(compressed, "compressedDepth")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "32FC1; compressedDepth png"
        raw2, err = decode(compressed, "compressedDepth")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "16UC1; compressedDepth png"
        raw2, err = decode(compressed, "compressedDepth")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "32FC1; compressedDepth rvl"
        raw2, err = decode(compressed, "compressedDepth")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

        compressed.format = "16UC1; compressedDepth rvl"
        raw2, err = decode(compressed, "compressedDepth")
        self.assertNotEqual(err, "")
        self.assertIsNone(raw2)

    def test_bag(self):
        d = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
        raw_bag = os.path.join(d, "raw.bag")
        compressed_bag = os.path.join(d, "compressed.bag")
        compressed_depth_bag = os.path.join(d, "compressedDepth.bag")

        with Bag(raw_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                msg.is_bigendian = 0  # Spot driver sets some images to 1 for some reason
                if topic == "/spot/camera/frontleft/image":
                    body_mono_raw = msg
                elif topic == "/spot/camera/hand_color/image":
                    hand_color_raw = msg
                elif topic == "/spot/camera/hand_mono/image":
                    hand_mono_raw = msg
                elif topic == "/spot/depth/frontleft/image":
                    body_depth_raw = msg

        with Bag(compressed_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                if topic == "/spot/camera/frontleft/image/compressed":
                    body_mono_compressed = msg
                elif topic == "/spot/camera/hand_color/image/compressed":
                    hand_color_compressed = msg
                elif topic == "/spot/camera/hand_mono/image/compressed":
                    hand_mono_compressed = msg
                elif topic == "/spot/depth/frontleft/image/compressed":
                    body_depth_compressed = msg

        with Bag(compressed_depth_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                if topic == "/spot/depth/frontleft/image/compressedDepth":
                    # The official encoder passes uninitialized memory to depthParam in case it encodes 16UC1 images,
                    # so we zero it out
                    data = bytearray(msg.data)
                    data[4:12] = b'\x00' * 8
                    msg.data = bytes(data)
                    body_depth_compressed_depth = msg

        # Body mono

        compressed, err = encode(body_mono_raw, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header.stamp, body_mono_compressed.header.stamp)
        self.assertEqual(compressed.header.frame_id, body_mono_compressed.header.frame_id)
        self.assertEqual(compressed.format, body_mono_compressed.format)
        self.assertEqual(compressed.data, body_mono_compressed.data)

        raw2, err = decode(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, body_mono_raw.header)
        self.assertEqual(raw2.step, body_mono_raw.step)
        self.assertEqual(raw2.width, body_mono_raw.width)
        self.assertEqual(raw2.height, body_mono_raw.height)
        self.assertEqual(raw2.encoding, body_mono_raw.encoding)
        self.assertEqual(raw2.is_bigendian, body_mono_raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(len(body_mono_raw.data)):
            self.assertLess(abs(_byte(raw2.data[i]) - _byte(body_mono_raw.data[i])), 20)

        # Body depth compressed

        compressed, err = encode(body_depth_raw, "compressed", {"format": CompressedPublisher_png, "png_level": 6})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header.stamp, body_depth_compressed.header.stamp)
        self.assertEqual(compressed.header.frame_id, body_depth_compressed.header.frame_id)
        self.assertEqual(compressed.format, body_depth_compressed.format)
        self.assertEqual(compressed.data, body_depth_compressed.data)

        raw2, err = decode(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, body_depth_raw.header)
        self.assertEqual(raw2.step, body_depth_raw.step)
        self.assertEqual(raw2.width, body_depth_raw.width)
        self.assertEqual(raw2.height, body_depth_raw.height)
        self.assertEqual(raw2.encoding, body_depth_raw.encoding)
        self.assertEqual(raw2.is_bigendian, body_depth_raw.is_bigendian)
        for i in range(len(body_depth_raw.data)):
            self.assertEqual(raw2.data[i], body_depth_raw.data[i])

        # Body depth compressedDepth

        # Enforce PNG compression 9 which was also used on the image
        compressed, err = encode(body_depth_raw, "compressedDepth", {"png_level": 9})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header.stamp, body_depth_compressed_depth.header.stamp)
        self.assertEqual(compressed.header.frame_id, body_depth_compressed_depth.header.frame_id)
        import re
        # account for the mismatch between Melodic and Noetic
        self.assertEqual(re.sub(' png$', '', compressed.format), body_depth_compressed_depth.format)
        self.assertEqual(bytes(compressed.data), body_depth_compressed_depth.data)

        raw2, err = decode(compressed, "compressedDepth")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, body_depth_raw.header)
        self.assertEqual(raw2.step, body_depth_raw.step)
        self.assertEqual(raw2.width, body_depth_raw.width)
        self.assertEqual(raw2.height, body_depth_raw.height)
        self.assertEqual(raw2.encoding, body_depth_raw.encoding)
        self.assertEqual(raw2.is_bigendian, body_depth_raw.is_bigendian)
        for i in range(len(body_depth_raw.data)):
            self.assertEqual(raw2.data[i], body_depth_raw.data[i])

        # Hand color

        for it in range(3):  # Test several iterations
            compressed, err = encode(hand_color_raw, "compressed")
            self.assertEqual(err, "")
            self.assertIsNotNone(compressed)
            self.assertIsInstance(compressed, CompressedImage)
            self.assertEqual(compressed.header.stamp, hand_color_compressed.header.stamp)
            self.assertEqual(compressed.header.frame_id, hand_color_compressed.header.frame_id)
            self.assertEqual(compressed.format, hand_color_compressed.format)
            # The binary representation of the JPEG might differ a bit, but the binary size should roughly match
            self.assertAlmostEqual(len(compressed.data), len(hand_color_compressed.data),
                                   delta=0.01*len(compressed.data))
    
            raw2, err = decode(compressed, "compressed")
            self.assertEqual(err, "")
            self.assertIsNotNone(raw2)
            self.assertIsInstance(raw2, Image)
            self.assertEqual(raw2.header, hand_color_raw.header)
            self.assertEqual(raw2.step, hand_color_raw.step)
            self.assertEqual(raw2.width, hand_color_raw.width)
            self.assertEqual(raw2.height, hand_color_raw.height)
            self.assertEqual(raw2.encoding, hand_color_raw.encoding)
            self.assertEqual(raw2.is_bigendian, hand_color_raw.is_bigendian)

            # JPEG compression changes this image quite a lot, so we examine the error histogram.
            # It is a 1920x1080 image, so having 7000 pixels with color difference of 20-30 is quite okay.
            err20 = 0
            err30 = 0
            err40 = 0
            err50 = 0
            err80 = 0
            err = 0
            for i in range(len(hand_color_raw.data)):
                e = abs(_byte(raw2.data[i]) - _byte(hand_color_raw.data[i]))
                if e < 20:
                    err20 += 1
                elif e < 30:
                    err30 += 1
                elif e < 40:
                    err40 += 1
                elif e < 50:
                    err50 += 1
                elif e < 80:
                    err80 += 1
                else:
                    err += 1
            # err20 is ok in any amount
            self.assertLessEqual(err30, 7000)
            self.assertLessEqual(err40, 500)
            self.assertLessEqual(err50, 100)
            self.assertLessEqual(err80, 30)
            self.assertLessEqual(0, err)

        # Hand mono

        compressed, err = encode(hand_mono_raw, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header.stamp, hand_mono_compressed.header.stamp)
        self.assertEqual(compressed.header.frame_id, hand_mono_compressed.header.frame_id)
        self.assertEqual(compressed.format, hand_mono_compressed.format)
        self.assertEqual(compressed.data, hand_mono_compressed.data)

        raw2, err = decode(compressed, "compressed")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, hand_mono_raw.header)
        self.assertEqual(raw2.step, hand_mono_raw.step)
        self.assertEqual(raw2.width, hand_mono_raw.width)
        self.assertEqual(raw2.height, hand_mono_raw.height)
        self.assertEqual(raw2.encoding, hand_mono_raw.encoding)
        self.assertEqual(raw2.is_bigendian, hand_mono_raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(len(hand_mono_raw.data)):
            self.assertLess(abs(_byte(raw2.data[i]) - _byte(hand_mono_raw.data[i])), 30)


if __name__ == '__main__':
    unittest.main()
