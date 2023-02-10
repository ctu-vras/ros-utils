#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for parse_compressed_format."""

import os
import unittest

from rosbag.bag import Bag

from image_transport_codecs.parse_compressed_format import *


class ParseCompressedFormat(unittest.TestCase):

    def test_parse_compressed(self):
        f = "bgr8; jpeg compressed bgr8"
        ff = CompressedTransportFormat(CompressedTransportCompressionFormat.JPEG, "jpeg", "bgr8", "bgr8", 3, 8, True)
        format, err = parse_compressed_transport_format(f)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = parse_compressed_transport_format(CompressedImage(format=f))
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_transport_format(ff.raw_encoding, ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_transport_format(Image(encoding=ff.raw_encoding), ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = make_compressed_transport_format(ff)
        self.assertEqual(format, f)
        self.assertEqual(err, "")

        f = "mono8; png compressed "
        ff = CompressedTransportFormat(CompressedTransportCompressionFormat.PNG, "png", "mono8", "mono8", 1, 8, False)
        format, err = parse_compressed_transport_format(f)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = parse_compressed_transport_format(CompressedImage(format=f))
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_transport_format(ff.raw_encoding, ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_transport_format(Image(encoding=ff.raw_encoding), ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = make_compressed_transport_format(ff)
        self.assertEqual(format, f)
        self.assertEqual(err, "")

        f = "16UC1; png compressed "
        ff = CompressedTransportFormat(CompressedTransportCompressionFormat.PNG, "png", "16UC1", "16UC1", 1, 16, False)
        format, err = parse_compressed_transport_format(f)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = parse_compressed_transport_format(CompressedImage(format=f))
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_transport_format(ff.raw_encoding, ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_transport_format(Image(encoding=ff.raw_encoding), ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = make_compressed_transport_format(ff)
        self.assertEqual(format, f)
        self.assertEqual(err, "")

        format, err = parse_compressed_transport_format("invalid")
        self.assertIsNone(format)
        self.assertEqual(err, "compressed transport format 'invalid' is invalid.")

    def test_parse_compressed_depth(self):
        f = "16UC1; compressedDepth png" if compressed_depth_has_rvl else "16UC1; compressedDepth"
        ff = ff_png = CompressedDepthTransportFormat(CompressedDepthTransportCompressionFormat.PNG, "png", "16UC1", 16)
        format, err = parse_compressed_depth_transport_format(f)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = parse_compressed_depth_transport_format(CompressedImage(format=f))
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_depth_transport_format(ff.raw_encoding, ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_depth_transport_format(Image(encoding=ff.raw_encoding), ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = make_compressed_depth_transport_format(ff)
        self.assertEqual(format, f)
        self.assertEqual(err, "")

        f = "16UC1; compressedDepth rvl" if compressed_depth_has_rvl else "16UC1; compressedDepth"
        ff = CompressedDepthTransportFormat(CompressedDepthTransportCompressionFormat.RVL, "rvl", "16UC1", 16)
        format, err = parse_compressed_depth_transport_format(f)
        self.assertEqual(format, ff if compressed_depth_has_rvl else ff_png)
        self.assertEqual(err, "")
        format, err = parse_compressed_depth_transport_format(CompressedImage(format=f))
        self.assertEqual(format, ff if compressed_depth_has_rvl else ff_png)
        self.assertEqual(err, "")
        format, err = extract_compressed_depth_transport_format(ff.raw_encoding, ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = extract_compressed_depth_transport_format(Image(encoding=ff.raw_encoding), ff.format)
        self.assertEqual(format, ff)
        self.assertEqual(err, "")
        format, err = make_compressed_depth_transport_format(ff)
        self.assertEqual(format, f)
        self.assertEqual(err, "")

        format, err = parse_compressed_depth_transport_format("invalid")
        self.assertIsNone(format)
        self.assertEqual(err, "compressedDepth transport format 'invalid' is invalid.")

    def test_guess_empty_image(self):
        image = CompressedImage()

        image.format = "jpeg"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNotNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed.format, CompressedTransportCompressionFormat.JPEG)

        image.format = "rvl"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNotNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed_depth.format, CompressedDepthTransportCompressionFormat.RVL)

        image.format = "png"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

        image.format = "png"
        image.data = [0] * 64
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNotNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed_depth.format, CompressedDepthTransportCompressionFormat.PNG)

        image.format = "png"
        image.data = [0] * 64
        image.data[:8] = (0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a)
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNotNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed.format, CompressedTransportCompressionFormat.PNG)

    def test_guess_wrong(self):
        image = CompressedImage()

        image.format = ""
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

        image.format = "wrong"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

        image.format = "jpeg compressed wrong"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

        image.format = "png compressed wrong"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

        image.format = "png compressedDepth wrong"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

        image.format = "rvl compressedDepth wrong"
        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(image)
        self.assertIsNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertNotEqual(err, "")

    def test_guess_bag(self):
        d = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
        compressed_bag = os.path.join(d, "compressed.bag")
        compressed_depth_bag = os.path.join(d, "compressedDepth.bag")

        with Bag(compressed_bag) as bag:
            for topic, msg, _ in bag.read_messages():
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
                if topic == "/spot/depth/frontleft/image/compressedDepth":
                    body_depth_compressed_depth = msg

        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(body_mono_compressed)
        self.assertIsNotNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed.format, CompressedTransportCompressionFormat.JPEG)
        self.assertEqual(compressed.bit_depth, 8)
        self.assertEqual(compressed.is_color, False)
        self.assertEqual(compressed.num_channels, 1)
        self.assertEqual(compressed.compressed_encoding, "mono8")
        self.assertEqual(compressed.raw_encoding, "mono8")

        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(hand_color_compressed)
        self.assertIsNotNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed.format, CompressedTransportCompressionFormat.JPEG)
        self.assertEqual(compressed.bit_depth, 8)
        self.assertEqual(compressed.is_color, True)
        self.assertEqual(compressed.num_channels, 3)
        self.assertEqual(compressed.compressed_encoding, "bgr8")
        self.assertEqual(compressed.raw_encoding, "rgb8")

        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(hand_mono_compressed)
        self.assertIsNotNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed.format, CompressedTransportCompressionFormat.JPEG)
        self.assertEqual(compressed.bit_depth, 8)
        self.assertEqual(compressed.is_color, False)
        self.assertEqual(compressed.num_channels, 1)
        self.assertEqual(compressed.compressed_encoding, "mono8")
        self.assertEqual(compressed.raw_encoding, "mono8")

        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(body_depth_compressed)
        self.assertIsNotNone(compressed)
        self.assertIsNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed.format, CompressedTransportCompressionFormat.PNG)
        self.assertEqual(compressed.bit_depth, 16)
        self.assertEqual(compressed.is_color, False)
        self.assertEqual(compressed.num_channels, 1)
        self.assertEqual(compressed.compressed_encoding, "16UC1")
        self.assertEqual(compressed.raw_encoding, "16UC1")

        compressed, compressed_depth, err = guess_any_compressed_image_transport_format(body_depth_compressed_depth)
        self.assertIsNone(compressed)
        self.assertIsNotNone(compressed_depth)
        self.assertEqual(err, "")
        self.assertEqual(compressed_depth.format, CompressedDepthTransportCompressionFormat.PNG)
        self.assertEqual(compressed_depth.bit_depth, 16)
        self.assertEqual(compressed_depth.raw_encoding, "16UC1")


if __name__ == '__main__':
    unittest.main()
