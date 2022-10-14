#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cras.geometry_utils"""

import math
import numpy as np
import unittest

from geometry_msgs.msg import Quaternion

from cras import quat_get_rpy, quat_get_roll, quat_get_pitch, quat_get_yaw, quat_msg_from_rpy, quat_tuple_from_rpy


sqrt2_2 = math.sqrt(2.0) / 2.0
pi_2 = math.pi / 2.0

# The exact values are taken from cras_cpp_common test_tf2_utils test run in debugger.
quat_test_cases = (
    ((0, 0, 0, 1), (0, 0, 0)),
    ((sqrt2_2, 0, 0, sqrt2_2), (pi_2, 0, 0)),
    ((0, sqrt2_2, 0, sqrt2_2), (0, pi_2, 0)),
    ((0, 0, sqrt2_2, sqrt2_2), (0, 0, pi_2)),
    (( 0.0499167083234140, 0.0499167083234140, -0.0024979173609871, 0.997502082639012), (0.1, 0.1, 0.0)),
    (( 0.2397127693021015, 0.2397127693021015, -0.0612087190548136, 0.938791280945186), (0.5, 0.5, 0.0)),
    (( 0.4207354924039482, 0.4207354924039482, -0.2298488470659301, 0.770151152934069), (1.0, 1.0, 0.0)),
    ((-0.0024979173609871, 0.0499167083234140,  0.0499167083234140, 0.997502082639012), (0.0, 0.1, 0.1)),
    ((-0.0612087190548136, 0.2397127693021015,  0.2397127693021015, 0.938791280945186), (0.0, 0.5, 0.5)),
    ((-0.2298488470659301, 0.4207354924039482,  0.4207354924039482, 0.770151152934069), (0.0, 1.0, 1.0)),
    (( 0.0499167083234140, 0.0024979173609871,  0.0499167083234140, 0.997502082639012), (0.1, 0.0, 0.1)),
    (( 0.2397127693021015, 0.0612087190548136,  0.2397127693021015, 0.938791280945186), (0.5, 0.0, 0.5)),
    (( 0.4207354924039482, 0.2298488470659301,  0.4207354924039482, 0.770151152934069), (1.0, 0.0, 1.0)),
    (( 0.0473595298213383, 0.0523491210508003,  0.0473595298213383, 0.996380308614844), (0.1, 0.1, 0.1)),
    (( 0.1729547916102582, 0.2915665680286702,  0.1729547916102582, 0.924749812936497), (0.5, 0.5, 0.5)),
    (( 0.1675187912463969, 0.5709414713577317,  0.1675187912463969, 0.786066629136844), (1.0, 1.0, 1.0)),
)


class GeometryUtils(unittest.TestCase):

    def test_get_rpy(self):
        for q, rpy in quat_test_cases:
            # noinspection PyPep8Naming
            Q = Quaternion(*q)
            self.assertTrue(np.allclose(rpy, quat_get_rpy(q), rtol=0, atol=1e-4))
            self.assertTrue(np.allclose(rpy, quat_get_rpy(*q), rtol=0, atol=1e-4))
            self.assertTrue(np.allclose(rpy, quat_get_rpy(Q), rtol=0, atol=1e-4))
            self.assertTrue(np.allclose(rpy, (quat_get_roll(q), quat_get_pitch(q), quat_get_yaw(q)), rtol=0, atol=1e-4))
            self.assertTrue(
                np.allclose(rpy, (quat_get_roll(*q), quat_get_pitch(*q), quat_get_yaw(*q)), rtol=0, atol=1e-4))
            self.assertTrue(np.allclose(rpy, (quat_get_roll(Q), quat_get_pitch(Q), quat_get_yaw(Q)), rtol=0, atol=1e-4))

    def test_get_rpy_fail(self):
        self.assertRaises(ValueError, quat_get_rpy, (1, 1, 1))
        self.assertRaises(ValueError, quat_get_rpy, (1, 1, 1, 1, 1))
        self.assertRaises(ValueError, quat_get_rpy, 1, 1, 1)
        self.assertRaises(ValueError, quat_get_rpy, 1, 1, 1, 1, 1)
        self.assertRaises(ValueError, quat_get_rpy, "", "", "", "")

    def test_quat_from_rpy(self):
        for q, rpy in quat_test_cases:
            self.assertTrue(np.allclose(q, quat_tuple_from_rpy(rpy), rtol=0, atol=1e-4))
            self.assertTrue(np.allclose(q, quat_tuple_from_rpy(*rpy), rtol=0, atol=1e-4))
            # noinspection PyPep8Naming
            Q = quat_msg_from_rpy(rpy)
            self.assertTrue(np.allclose(q, (Q.x, Q.y, Q.z, Q.w), rtol=0, atol=1e-4))
            Q = quat_msg_from_rpy(*rpy)
            self.assertTrue(np.allclose(q, (Q.x, Q.y, Q.z, Q.w), rtol=0, atol=1e-4))

    def test_quat_from_rpy_fail(self):
        self.assertRaises(ValueError, quat_tuple_from_rpy, (1, 1))
        self.assertRaises(ValueError, quat_tuple_from_rpy, (1, 1, 1, 1))
        self.assertRaises(ValueError, quat_tuple_from_rpy, 1, 1)
        self.assertRaises(ValueError, quat_tuple_from_rpy, "", "", "")


if __name__ == '__main__':
    unittest.main()
