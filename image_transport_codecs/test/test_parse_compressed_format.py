#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for parse_compressed_format."""

from image_transport_codecs.parse_compressed_format import *
import unittest


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


if __name__ == '__main__':
    unittest.main()
