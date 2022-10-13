#!/usr/bin/env python

"""Unit test for cras.time_utils"""
import time
import unittest

import rospy
from rospy import Duration, Rate

from cras import frequency, safe_rate, rate_equals, DURATION_MAX, DURATION_MIN, slowest_rate, slowest_negative_rate, \
    WallRate, WallTime, SteadyRate, SteadyTime, wallsleep, Timer, monotonic


class TimeUtils(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TimeUtils, self).__init__(*args, **kwargs)
        rospy.rostime.set_rostime_initialized(True)
        self.num_timer_events_wall = 0
        self.num_timer_events_steady = 0

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

    def test_monotonic(self):
        for i in range(100):
            now = monotonic()
            self.assertGreaterEqual(monotonic(), now)

    def test_sleep(self):
        now = monotonic()
        wallsleep(0.1)
        self.assertGreater(monotonic() - now, 0.075)

        now = monotonic()
        wallsleep(rospy.Duration(0.1))
        self.assertGreater(monotonic() - now, 0.075)

    def test_wall_time(self):
        t = WallTime(1, 0)
        self.assertEqual(1, t.secs)
        self.assertEqual(0, t.nsecs)

        t = WallTime(3.14)
        self.assertEqual(3, t.secs)
        self.assertAlmostEqual(140000000, t.nsecs, delta=10)
        self.assertAlmostEqual(3.14, t.to_sec(), delta=1e-8)

        self.assertAlmostEqual(time.time(), WallTime.now().to_sec(), delta=1e-2)

    def test_steady_time(self):
        t = SteadyTime(1, 0)
        self.assertEqual(1, t.secs)
        self.assertEqual(0, t.nsecs)

        t = SteadyTime(3.14)
        self.assertEqual(3, t.secs)
        self.assertAlmostEqual(140000000, t.nsecs, delta=10)
        self.assertAlmostEqual(3.14, t.to_sec(), delta=1e-8)

        now = SteadyTime.now()
        time.sleep(0.1)
        self.assertGreater((SteadyTime.now() - now).to_sec(), 0.075)

        now = SteadyTime.now()
        now2 = SteadyTime.now()
        self.assertGreaterEqual(now2, now)

    def test_wall_rate(self):
        r = WallRate(100)
        self.assertAlmostEqual(time.time(), r.now().to_sec(), delta=1e-2)
        self.assertAlmostEqual(0.01, r.sleep_dur.to_sec(), delta=1e-6)

        t1 = time.time()
        r.sleep()
        t2 = time.time()

        self.assertAlmostEqual(0.01, t2 - t1, delta=0.02)

    def test_steady_rate(self):

        r = SteadyRate(100)
        self.assertAlmostEqual(0.01, r.sleep_dur.to_sec(), delta=1e-6)

        t1 = time.time()
        r.sleep()
        t2 = time.time()

        self.assertAlmostEqual(0.01, t2 - t1, delta=0.02)

    def timer_cb_wall(self, timer_event):
        self.assertTrue(isinstance(timer_event, rospy.timer.TimerEvent))
        self.num_timer_events_wall += 1

    def timer_cb_steady(self, timer_event):
        self.assertTrue(isinstance(timer_event, rospy.timer.TimerEvent))
        self.num_timer_events_steady += 1

    def test_timer_wall(self):
        self.assertEqual(self.num_timer_events_wall, 0)

        r = WallRate(100)
        timer = Timer(r, self.timer_cb_wall)

        wallsleep(0.1)

        timer.shutdown()

        # Ideally, it would be exactly 10, but give it some slack
        self.assertGreaterEqual(20, self.num_timer_events_wall)
        self.assertLessEqual(5, self.num_timer_events_wall)

    def test_timer_steady(self):
        self.assertEqual(self.num_timer_events_steady, 0)

        r = SteadyRate(100)
        timer = Timer(r, self.timer_cb_steady)

        wallsleep(0.1)

        timer.shutdown()

        # Ideally, it would be exactly 10, but give it some slack
        self.assertGreaterEqual(20, self.num_timer_events_steady)
        self.assertLessEqual(5, self.num_timer_events_steady)


if __name__ == '__main__':
    unittest.main()
