#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.image_encodings"""

import unittest

from cras.image_encodings import *


class ImageEncodings(unittest.TestCase):

    def test_constants(self):
        self.assertEqual(RGB8, "rgb8")
        self.assertEqual(BGRA16, "bgra16")
        self.assertEqual(TYPE_8SC1, "8SC1")
        self.assertEqual(TYPE_32FC1, "32FC1")
        self.assertEqual(BAYER_BGGR8, "bayer_bggr8")
        self.assertEqual(YUV422, "yuv422")

    def test_is_color(self):
        self.assertFalse(isColor(MONO8))
        self.assertTrue(isColor(RGB8))
        self.assertTrue(isColor(BGR8))
        self.assertFalse(isColor(BAYER_BGGR8))
        self.assertFalse(isColor(TYPE_8SC1))
        self.assertFalse(isColor(YUV422))

    def test_is_mono(self):
        self.assertTrue(isMono(MONO8))
        self.assertFalse(isMono(RGB8))
        self.assertFalse(isMono(BGR8))
        self.assertFalse(isMono(BAYER_BGGR8))
        self.assertFalse(isMono(TYPE_8SC1))
        self.assertFalse(isMono(YUV422))

    def test_is_bayer(self):
        self.assertFalse(isBayer(MONO8))
        self.assertFalse(isBayer(RGB8))
        self.assertFalse(isBayer(BGR8))
        self.assertTrue(isBayer(BAYER_BGGR8))
        self.assertFalse(isBayer(TYPE_8SC1))
        self.assertFalse(isBayer(YUV422))

    def test_is_depth(self):
        self.assertFalse(isDepth(MONO8))
        self.assertFalse(isDepth(RGB8))
        self.assertFalse(isDepth(BGR8))
        self.assertFalse(isDepth(BAYER_BGGR8))
        self.assertFalse(isDepth(TYPE_8SC1))
        self.assertFalse(isDepth(YUV422))
        self.assertTrue(isDepth(TYPE_16UC1))
        self.assertTrue(isDepth(TYPE_32FC1))

    def test_has_alpha(self):
        self.assertFalse(hasAlpha(MONO8))
        self.assertFalse(hasAlpha(RGB8))
        self.assertFalse(hasAlpha(BGR8))
        self.assertTrue(hasAlpha(RGBA8))
        self.assertFalse(hasAlpha(BAYER_BGGR8))
        self.assertFalse(hasAlpha(TYPE_8SC1))
        self.assertFalse(hasAlpha(YUV422))

    def test_num_channels(self):
        self.assertEqual(numChannels(MONO8), 1)
        self.assertEqual(numChannels(RGB8), 3)
        self.assertEqual(numChannels(BGR8), 3)
        self.assertEqual(numChannels(RGBA8), 4)
        self.assertEqual(numChannels(BAYER_BGGR8), 1)
        self.assertEqual(numChannels(TYPE_8SC1), 1)
        self.assertEqual(numChannels(YUV422), 2)

    def test_bit_depth(self):
        self.assertEqual(bitDepth(MONO8), 8)
        self.assertEqual(bitDepth(RGB8), 8)
        self.assertEqual(bitDepth(BGR8), 8)
        self.assertEqual(bitDepth(RGBA8), 8)
        self.assertEqual(bitDepth(BAYER_BGGR8), 8)
        self.assertEqual(bitDepth(TYPE_8SC1), 8)
        self.assertEqual(bitDepth(YUV422), 8)


if __name__ == '__main__':
    unittest.main()
