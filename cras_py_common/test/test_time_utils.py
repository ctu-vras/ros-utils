#!/usr/bin/env python

"""Unit test for cras.time_utils"""

import unittest

import rospy
from rospy import Duration, Rate

from cras import frequency, safe_rate, rate_equals, DURATION_MAX, DURATION_MIN, slowest_rate, slowest_negative_rate


class TimeUtils(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TimeUtils, self).__init__(*args, **kwargs)
        rospy.rostime.set_rostime_initialized(True)

    def test_extreme_rates(self):
        self.assertEqual(DURATION_MAX, slowest_rate().sleep_dur)
        self.assertEqual(DURATION_MIN, slowest_negative_rate().sleep_dur)

    def test_frequency(self):
        self.assertAlmostEqual(1.0, frequency(Rate(1.0)), delta=1e-3)
        self.assertAlmostEqual(-1.0, frequency(Rate(-1.0)), delta=1e-3)
        self.assertAlmostEqual(10.0, frequency(Rate(10.0)), delta=1e-3)
        self.assertAlmostEqual(-10.0, frequency(Rate(-10.0)), delta=1e-3)
        self.assertAlmostEqual(3.14, frequency(Rate(3.14)), delta=1e-3)
        self.assertAlmostEqual(-3.14, frequency(Rate(-3.14)), delta=1e-3)

        self.assertAlmostEqual(1.0, frequency(Rate(1.0), True), delta=1e-3)
        self.assertAlmostEqual(-1.0, frequency(Rate(-1.0), True), delta=1e-3)
        self.assertAlmostEqual(10.0, frequency(Rate(10.0), True), delta=1e-3)
        self.assertAlmostEqual(-10.0, frequency(Rate(-10.0), True), delta=1e-3)
        self.assertAlmostEqual(3.14, frequency(Rate(3.14), True), delta=1e-3)
        self.assertAlmostEqual(-3.14, frequency(Rate(-3.14), True), delta=1e-3)

        self.assertAlmostEqual(0, frequency(slowest_rate()), delta=1e-3)
        self.assertNotEqual(0, frequency(slowest_rate()))  # it is very small, but not zero
        self.assertEqual(0, frequency(slowest_rate(), True))
        self.assertAlmostEqual(0, frequency(slowest_negative_rate()), delta=1e-3)
        self.assertNotEqual(0, frequency(slowest_negative_rate()))  # it is very small, but not zero
        self.assertEqual(0, frequency(slowest_negative_rate(), True))

    def test_safe_rate(self):
        self.assertEqual(DURATION_MAX, safe_rate(0).sleep_dur)
        self.assertEqual(DURATION_MAX, safe_rate(1e-10).sleep_dur)
        self.assertGreater(DURATION_MAX, safe_rate(1e-9).sleep_dur)
        self.assertEqual(Duration(1), safe_rate(1).sleep_dur)
        self.assertEqual(Duration(1e-9), safe_rate(1e9).sleep_dur)
        self.assertEqual(Duration(0), safe_rate(float('inf')).sleep_dur)

    def test_rate_equals(self):
        self.assertTrue(rate_equals(Rate(1), Rate(1)))
        self.assertFalse(rate_equals(Rate(1), Rate(2)))


if __name__ == '__main__':
    unittest.main()
