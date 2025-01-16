#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.distortion_models"""

import unittest

from cras.distortion_models import *


class DistortionModels(unittest.TestCase):

    def test_constants(self):
        self.assertEqual(PLUMB_BOB, "plumb_bob")
        self.assertEqual(RATIONAL_POLYNOMIAL, "rational_polynomial")
        self.assertEqual(EQUIDISTANT, "equidistant")


if __name__ == '__main__':
    unittest.main()
