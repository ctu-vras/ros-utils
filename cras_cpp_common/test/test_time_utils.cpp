/**
 * \file
 * \brief Unit test for time_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <limits>
#include <memory>
#include <thread>

#include <cras_cpp_common/time_utils.hpp>
#include <cras_cpp_common/time_utils/interruptible_sleep_interface.h>

#include <ros/duration.h>
#include <ros/time.h>

using namespace cras;
using namespace ros;

// The following testcase has to be the first one because it tests what happens when time is not initialized!
TEST(TimeUtils, NowFallbackToWall)  // NOLINT
{
  // Before time is initialized, nowFallbackToWall() should return wall time
  auto t = cras::nowFallbackToWall();
  const auto wall = ros::WallTime::now();
  const auto wallTime = ros::Time(wall.sec, wall.nsec);
  EXPECT_LT((wallTime > t) ? (wallTime - t) : (t - wallTime), ros::Duration(0.1));

  // This is an unrelated testcase, but we need to test both before time is initialized
  EXPECT_THROW(remainingTime({99, 0}, {2, 0}), ros::TimeNotInitializedException);

  // After time initialization, it should return ROS time
  Time::init();
  Time::setNow({100, 0});

  t = cras::nowFallbackToWall();
  EXPECT_EQ(ros::Time::now(), t);

  Time::shutdown();
}

TEST(TimeUtils, RemainingTimeDuration)  // NOLINT
{
  Time::init();
  Time::setNow({100, 0});

  EXPECT_EQ(9.0, remainingTime({99, 0}, {10, 0}).toSec());
  EXPECT_EQ(1.0, remainingTime({99, 0}, {2, 0}).toSec());
  EXPECT_EQ(0.0, remainingTime({90, 0}, {2, 0}).toSec());  // time's up
  Time::shutdown();
}

TEST(TimeUtils, RemainingTimeDouble)  // NOLINT
{
  Time::init();
  Time::setNow({100, 0});

  EXPECT_EQ(9.0, remainingTime({99, 0}, 10.0).toSec());
  EXPECT_EQ(1.0, remainingTime({99, 0}, 2.0).toSec());
  EXPECT_EQ(0.0, remainingTime({90, 0}, 2.0).toSec());  // time's up
  Time::shutdown();
}

TEST(TimeUtils, Frequency)  // NOLINT
{
  EXPECT_NEAR(1.0, frequency(ros::Rate(1.0)), 1e-3);
  EXPECT_NEAR(-1.0, frequency(ros::Rate(-1.0)), 1e-3);
  EXPECT_NEAR(10.0, frequency(ros::Rate(10.0)), 1e-3);
  EXPECT_NEAR(-10.0, frequency(ros::Rate(-10.0)), 1e-3);
  EXPECT_NEAR(3.14, frequency(ros::Rate(3.14)), 1e-3);
  EXPECT_NEAR(-3.14, frequency(ros::Rate(-3.14)), 1e-3);

  EXPECT_NEAR(1.0, frequency(ros::Rate(1.0), true), 1e-3);
  EXPECT_NEAR(-1.0, frequency(ros::Rate(-1.0), true), 1e-3);
  EXPECT_NEAR(10.0, frequency(ros::Rate(10.0), true), 1e-3);
  EXPECT_NEAR(-10.0, frequency(ros::Rate(-10.0), true), 1e-3);
  EXPECT_NEAR(3.14, frequency(ros::Rate(3.14), true), 1e-3);
  EXPECT_NEAR(-3.14, frequency(ros::Rate(-3.14), true), 1e-3);

  EXPECT_EQ(std::numeric_limits<double>::infinity(), frequency(ros::Rate(ros::Duration(0))));

  EXPECT_NEAR(0, frequency(ros::Rate(ros::DURATION_MAX)), 1e-3);
  EXPECT_NE(0, frequency(ros::Rate(ros::DURATION_MAX)));  // it is very small, but not zero
  EXPECT_EQ(0, frequency(ros::Rate(ros::DURATION_MAX), true));
  EXPECT_NEAR(0, frequency(ros::Rate(ros::DURATION_MIN)), 1e-3);
  EXPECT_NE(0, frequency(ros::Rate(ros::DURATION_MIN)));  // it is very small, but not zero
  EXPECT_EQ(0, frequency(ros::Rate(ros::DURATION_MIN), true));
}

TEST(TimeUtils, WallFrequency)  // NOLINT
{
  EXPECT_NEAR(1.0, frequency(ros::WallRate(1.0)), 1e-3);
  EXPECT_NEAR(-1.0, frequency(ros::WallRate(-1.0)), 1e-3);
  EXPECT_NEAR(10.0, frequency(ros::WallRate(10.0)), 1e-3);
  EXPECT_NEAR(-10.0, frequency(ros::WallRate(-10.0)), 1e-3);
  EXPECT_NEAR(3.14, frequency(ros::WallRate(3.14)), 1e-3);
  EXPECT_NEAR(-3.14, frequency(ros::WallRate(-3.14)), 1e-3);

  EXPECT_NEAR(1.0, frequency(ros::WallRate(1.0), true), 1e-3);
  EXPECT_NEAR(-1.0, frequency(ros::WallRate(-1.0), true), 1e-3);
  EXPECT_NEAR(10.0, frequency(ros::WallRate(10.0), true), 1e-3);
  EXPECT_NEAR(-10.0, frequency(ros::WallRate(-10.0), true), 1e-3);
  EXPECT_NEAR(3.14, frequency(ros::WallRate(3.14), true), 1e-3);
  EXPECT_NEAR(-3.14, frequency(ros::WallRate(-3.14), true), 1e-3);

  EXPECT_EQ(std::numeric_limits<double>::infinity(), frequency(ros::WallRate(ros::Duration(0))));

  EXPECT_NEAR(0, frequency(ros::WallRate(ros::DURATION_MAX)), 1e-3);
  EXPECT_NE(0, frequency(ros::WallRate(ros::DURATION_MAX)));  // it is very small, but not zero
  EXPECT_EQ(0, frequency(ros::WallRate(ros::DURATION_MAX), true));
  EXPECT_NEAR(0, frequency(ros::WallRate(ros::DURATION_MIN)), 1e-3);
  EXPECT_NE(0, frequency(ros::WallRate(ros::DURATION_MIN)));  // it is very small, but not zero
  EXPECT_EQ(0, frequency(ros::WallRate(ros::DURATION_MIN), true));
}

TEST(TimeUtils, SafeRate)  // NOLINT
{
  EXPECT_EQ(ros::DURATION_MAX, safeRate(0).expectedCycleTime());
  EXPECT_EQ(ros::DURATION_MAX, safeRate(1e-10).expectedCycleTime());
  EXPECT_GT(ros::DURATION_MAX, safeRate(1e-9).expectedCycleTime());
  EXPECT_EQ(ros::Duration(1), safeRate(1).expectedCycleTime());
  EXPECT_EQ(ros::Duration(1e-9), safeRate(1e9).expectedCycleTime());
  EXPECT_EQ(ros::Duration(0), safeRate(std::numeric_limits<double>::infinity()).expectedCycleTime());
}

TEST(TimeUtils, SafeWallRate)  // NOLINT
{
  EXPECT_EQ(ros::DURATION_MAX.toNSec(), safeWallRate(0).expectedCycleTime().toNSec());
  EXPECT_EQ(ros::DURATION_MAX.toNSec(), safeWallRate(1e-10).expectedCycleTime().toNSec());
  EXPECT_GT(ros::DURATION_MAX.toNSec(), safeWallRate(1e-9).expectedCycleTime().toNSec());
  EXPECT_EQ(ros::Duration(1).toNSec(), safeWallRate(1).expectedCycleTime().toNSec());
  EXPECT_EQ(ros::Duration(1e-9).toNSec(), safeWallRate(1e9).expectedCycleTime().toNSec());
  EXPECT_EQ(ros::Duration(0).toNSec(),
            safeWallRate(std::numeric_limits<double>::infinity()).expectedCycleTime().toNSec());
}

TEST(TimeUtils, RateEquals)  // NOLINT
{
  EXPECT_TRUE(ros::Rate(1.0) == ros::Rate(1.0));
  EXPECT_TRUE(ros::Rate(2.0) == ros::Rate(2.0));
  EXPECT_TRUE(ros::Rate(3.14) == ros::Rate(3.14));
  EXPECT_TRUE(ros::Rate(1e9) == ros::Rate(1e9));
  EXPECT_TRUE(ros::Rate(ros::DURATION_MAX) == ros::Rate(ros::DURATION_MAX));
  EXPECT_TRUE(ros::Rate(ros::DURATION_MIN) == ros::Rate(ros::DURATION_MIN));
  EXPECT_FALSE(ros::Rate(3.13) == ros::Rate(3.14));
  EXPECT_FALSE(ros::Rate(-3.14) == ros::Rate(3.14));
  EXPECT_FALSE(ros::Rate(3.0) == ros::Rate(3.14));
  EXPECT_FALSE(ros::Rate(1.0) == ros::Rate(2.0));
  EXPECT_FALSE(ros::Rate(1.0) == ros::Rate(-1.0));
  EXPECT_FALSE(ros::Rate(ros::DURATION_MIN) == ros::Rate(ros::DURATION_MAX));
  EXPECT_FALSE(ros::Rate(ros::DURATION_MAX) == ros::Rate(ros::DURATION_MIN));
}

TEST(TimeUtils, WallRateEquals)  // NOLINT
{
  EXPECT_TRUE(ros::WallRate(1.0) == ros::WallRate(1.0));
  EXPECT_TRUE(ros::WallRate(2.0) == ros::WallRate(2.0));
  EXPECT_TRUE(ros::WallRate(3.14) == ros::WallRate(3.14));
  EXPECT_TRUE(ros::WallRate(1e9) == ros::WallRate(1e9));
  EXPECT_TRUE(ros::WallRate(ros::DURATION_MAX) == ros::WallRate(ros::DURATION_MAX));
  EXPECT_TRUE(ros::WallRate(ros::DURATION_MIN) == ros::WallRate(ros::DURATION_MIN));
  EXPECT_FALSE(ros::WallRate(3.13) == ros::WallRate(3.14));
  EXPECT_FALSE(ros::WallRate(-3.14) == ros::WallRate(3.14));
  EXPECT_FALSE(ros::WallRate(3.0) == ros::WallRate(3.14));
  EXPECT_FALSE(ros::WallRate(1.0) == ros::WallRate(2.0));
  EXPECT_FALSE(ros::WallRate(1.0) == ros::WallRate(-1.0));
  EXPECT_FALSE(ros::WallRate(ros::DURATION_MIN) == ros::WallRate(ros::DURATION_MAX));
  EXPECT_FALSE(ros::WallRate(ros::DURATION_MAX) == ros::WallRate(ros::DURATION_MIN));
}

TEST(TimeUtils, DurationMultiply)  // NOLINT
{
  EXPECT_EQ(ros::Duration(2, 0), ros::Duration(1, 0) * ros::Duration(2, 0));
  EXPECT_EQ(ros::Duration(400, 0), ros::Duration(20, 0) * ros::Duration(20, 0));
  EXPECT_EQ(ros::Duration(0, 250000000), ros::Duration(0, 500000000) * ros::Duration(0, 500000000));
  EXPECT_EQ(ros::Duration(0, 0), ros::Duration(0, 1) * ros::Duration(0, 1));  // too small value
  EXPECT_EQ(ros::Duration(0, 0), ros::Duration(1000, 0) * ros::Duration(0, 0));
  EXPECT_EQ(ros::Duration(-1000, 0), ros::Duration(-1000, 0) * ros::Duration(1, 0));
  EXPECT_EQ(ros::Duration(-1000, 0), ros::Duration(1, 0) * ros::Duration(-1000, 0));
  EXPECT_EQ(ros::Duration(1000000, 0), ros::Duration(-1000, 0) * ros::Duration(-1000, 0));
  EXPECT_THROW(ros::Duration(1000000000, 0) * ros::Duration(1000000000, 0), std::runtime_error);
}

TEST(TimeUtils, WallDurationMultiply)  // NOLINT
{
  EXPECT_EQ(ros::WallDuration(2, 0), ros::WallDuration(1, 0) * ros::WallDuration(2, 0));
  EXPECT_EQ(ros::WallDuration(400, 0), ros::WallDuration(20, 0) * ros::WallDuration(20, 0));
  EXPECT_EQ(ros::WallDuration(0, 250000000), ros::WallDuration(0, 500000000) * ros::WallDuration(0, 500000000));
  EXPECT_EQ(ros::WallDuration(0, 0), ros::WallDuration(0, 1) * ros::WallDuration(0, 1));  // too small value
  EXPECT_EQ(ros::WallDuration(0, 0), ros::WallDuration(1000, 0) * ros::WallDuration(0, 0));
  EXPECT_EQ(ros::WallDuration(-1000, 0), ros::WallDuration(-1000, 0) * ros::WallDuration(1, 0));
  EXPECT_EQ(ros::WallDuration(-1000, 0), ros::WallDuration(1, 0) * ros::WallDuration(-1000, 0));
  EXPECT_EQ(ros::WallDuration(1000000, 0), ros::WallDuration(-1000, 0) * ros::WallDuration(-1000, 0));
  EXPECT_THROW(ros::WallDuration(1000000000, 0) * ros::WallDuration(1000000000, 0), std::runtime_error);
}

TEST(TimeUtils, DurationDivide)  // NOLINT
{
  EXPECT_EQ(ros::Duration(1, 0), ros::Duration(2, 0) / ros::Duration(2, 0));
  EXPECT_EQ(ros::Duration(0, 0), ros::Duration(0, 0) / ros::Duration(2, 0));
  EXPECT_EQ(ros::Duration(0, 500000000), ros::Duration(1, 0) / ros::Duration(2, 0));
  EXPECT_EQ(ros::Duration(0, 250000000), ros::Duration(1, 0) / ros::Duration(4, 0));
  EXPECT_EQ(ros::Duration(0, 125000000), ros::Duration(1, 0) / ros::Duration(8, 0));
  EXPECT_EQ(ros::Duration(0, 125000000), ros::Duration(2, 0) / ros::Duration(16, 0));
  EXPECT_EQ(ros::Duration(2, 0), ros::Duration(2, 0) / ros::Duration(1, 0));
  EXPECT_EQ(ros::Duration(4, 0), ros::Duration(4, 0) / ros::Duration(1, 0));
  EXPECT_EQ(ros::Duration(8, 0), ros::Duration(8, 0) / ros::Duration(1, 0));
  EXPECT_EQ(ros::Duration(8, 0), ros::Duration(16, 0) / ros::Duration(2, 0));
  EXPECT_EQ(ros::Duration(1, 0), ros::Duration(1000000000, 0) / ros::Duration(1000000000, 0));
  EXPECT_EQ(ros::Duration(-2, 0), ros::Duration(-4, 0) / ros::Duration(2, 0));
  EXPECT_EQ(ros::Duration(-2, 0), ros::Duration(4, 0) / ros::Duration(-2, 0));
  EXPECT_EQ(ros::Duration(2, 0), ros::Duration(-4, 0) / ros::Duration(-2, 0));
  EXPECT_THROW(ros::Duration(1, 0) / ros::Duration(0, 0), std::runtime_error);
  EXPECT_THROW(ros::Duration(1, 0) / ros::Duration(-0, 0), std::runtime_error);
}

TEST(TimeUtils, WallDurationDivide)  // NOLINT
{
  EXPECT_EQ(ros::WallDuration(1, 0), ros::WallDuration(2, 0) / ros::WallDuration(2, 0));
  EXPECT_EQ(ros::WallDuration(0, 0), ros::WallDuration(0, 0) / ros::WallDuration(2, 0));
  EXPECT_EQ(ros::WallDuration(0, 500000000), ros::WallDuration(1, 0) / ros::WallDuration(2, 0));
  EXPECT_EQ(ros::WallDuration(0, 250000000), ros::WallDuration(1, 0) / ros::WallDuration(4, 0));
  EXPECT_EQ(ros::WallDuration(0, 125000000), ros::WallDuration(1, 0) / ros::WallDuration(8, 0));
  EXPECT_EQ(ros::WallDuration(0, 125000000), ros::WallDuration(2, 0) / ros::WallDuration(16, 0));
  EXPECT_EQ(ros::WallDuration(2, 0), ros::WallDuration(2, 0) / ros::WallDuration(1, 0));
  EXPECT_EQ(ros::WallDuration(4, 0), ros::WallDuration(4, 0) / ros::WallDuration(1, 0));
  EXPECT_EQ(ros::WallDuration(8, 0), ros::WallDuration(8, 0) / ros::WallDuration(1, 0));
  EXPECT_EQ(ros::WallDuration(8, 0), ros::WallDuration(16, 0) / ros::WallDuration(2, 0));
  EXPECT_EQ(ros::WallDuration(1, 0), ros::WallDuration(1000000000, 0) / ros::WallDuration(1000000000, 0));
  EXPECT_EQ(ros::WallDuration(-2, 0), ros::WallDuration(-4, 0) / ros::WallDuration(2, 0));
  EXPECT_EQ(ros::WallDuration(-2, 0), ros::WallDuration(4, 0) / ros::WallDuration(-2, 0));
  EXPECT_EQ(ros::WallDuration(2, 0), ros::WallDuration(-4, 0) / ros::WallDuration(-2, 0));
  EXPECT_THROW(ros::WallDuration(1, 0) / ros::WallDuration(0, 0), std::runtime_error);
  EXPECT_THROW(ros::WallDuration(1, 0) / ros::WallDuration(-0, 0), std::runtime_error);
}

TEST(TimeUtils, SaturateAddTime)  // NOLINT
{
  EXPECT_EQ(ros::Time(2, 0), cras::saturateAdd(ros::Time(1, 0), ros::Duration(1, 0)));
  EXPECT_EQ(ros::Time(0, 0), cras::saturateAdd(ros::Time(1, 0), ros::Duration(-1, 0)));
  EXPECT_EQ(ros::Time(0, 0), cras::saturateAdd(ros::Time(1, 0), ros::Duration(-2, 0)));
  EXPECT_EQ(ros::Time(0, 0), cras::saturateAdd(ros::Time(1, 0), ros::Duration(-10, 0)));
  EXPECT_EQ(ros::Time(0, 0), cras::saturateAdd(ros::Time(1, 0), -ros::Duration::MAX));
  EXPECT_EQ(ros::Time(10, 0), cras::saturateAdd(ros::Time(6, 0), ros::Duration(4, 0)));
  EXPECT_EQ(ros::Time::MAX, cras::saturateAdd(ros::Time::MAX, ros::Duration(0, 0)));
  EXPECT_EQ(ros::Time::MAX, cras::saturateAdd(ros::Time::MAX, ros::Duration(1, 0)));
  EXPECT_EQ(ros::Time::MAX, cras::saturateAdd(ros::Time::MAX, ros::Duration(2, 0)));
  EXPECT_EQ(ros::Time::MAX, cras::saturateAdd(ros::Time::MAX, ros::Duration::MAX));
  EXPECT_EQ(ros::Time(ros::Duration::MAX.sec, ros::Duration::MAX.nsec),
    cras::saturateAdd(ros::Time::ZERO, ros::Duration::MAX));
}

TEST(TimeUtils, SaturateAddWallTime)  // NOLINT
{
  EXPECT_EQ(ros::WallTime(2, 0), cras::saturateAdd(ros::WallTime(1, 0), ros::WallDuration(1, 0)));
  EXPECT_EQ(ros::WallTime(0, 0), cras::saturateAdd(ros::WallTime(1, 0), ros::WallDuration(-1, 0)));
  EXPECT_EQ(ros::WallTime(0, 0), cras::saturateAdd(ros::WallTime(1, 0), ros::WallDuration(-2, 0)));
  EXPECT_EQ(ros::WallTime(0, 0), cras::saturateAdd(ros::WallTime(1, 0), ros::WallDuration(-10, 0)));
  EXPECT_EQ(ros::WallTime(0, 0), cras::saturateAdd(ros::WallTime(1, 0), -ros::WallDuration::MAX));
  EXPECT_EQ(ros::WallTime(10, 0), cras::saturateAdd(ros::WallTime(6, 0), ros::WallDuration(4, 0)));
  EXPECT_EQ(ros::WallTime::MAX, cras::saturateAdd(ros::WallTime::MAX, ros::WallDuration(0, 0)));
  EXPECT_EQ(ros::WallTime::MAX, cras::saturateAdd(ros::WallTime::MAX, ros::WallDuration(1, 0)));
  EXPECT_EQ(ros::WallTime::MAX, cras::saturateAdd(ros::WallTime::MAX, ros::WallDuration(2, 0)));
  EXPECT_EQ(ros::WallTime::MAX, cras::saturateAdd(ros::WallTime::MAX, ros::WallDuration::MAX));
  EXPECT_EQ(ros::WallTime(ros::WallDuration::MAX.sec, ros::WallDuration::MAX.nsec),
    cras::saturateAdd(ros::WallTime::ZERO, ros::WallDuration::MAX));
}

TEST(TimeUtils, SaturateAddSteadyTime)  // NOLINT
{
  EXPECT_EQ(ros::SteadyTime(2, 0), cras::saturateAdd(ros::SteadyTime(1, 0), ros::WallDuration(1, 0)));
  EXPECT_EQ(ros::SteadyTime(0, 0), cras::saturateAdd(ros::SteadyTime(1, 0), ros::WallDuration(-1, 0)));
  EXPECT_EQ(ros::SteadyTime(0, 0), cras::saturateAdd(ros::SteadyTime(1, 0), ros::WallDuration(-2, 0)));
  EXPECT_EQ(ros::SteadyTime(0, 0), cras::saturateAdd(ros::SteadyTime(1, 0), ros::WallDuration(-10, 0)));
  EXPECT_EQ(ros::SteadyTime(0, 0), cras::saturateAdd(ros::SteadyTime(1, 0), -ros::WallDuration::MAX));
  EXPECT_EQ(ros::SteadyTime(10, 0), cras::saturateAdd(ros::SteadyTime(6, 0), ros::WallDuration(4, 0)));
  EXPECT_EQ(ros::SteadyTime::MAX, cras::saturateAdd(ros::SteadyTime::MAX, ros::WallDuration(0, 0)));
  EXPECT_EQ(ros::SteadyTime::MAX, cras::saturateAdd(ros::SteadyTime::MAX, ros::WallDuration(1, 0)));
  EXPECT_EQ(ros::SteadyTime::MAX, cras::saturateAdd(ros::SteadyTime::MAX, ros::WallDuration(2, 0)));
  EXPECT_EQ(ros::SteadyTime::MAX, cras::saturateAdd(ros::SteadyTime::MAX, ros::WallDuration::MAX));
  EXPECT_EQ(ros::SteadyTime(ros::WallDuration::MAX.sec, ros::WallDuration::MAX.nsec),
    cras::saturateAdd(ros::SteadyTime::ZERO, ros::WallDuration::MAX));
}

class TestSleepInterface : public cras::InterruptibleSleepInterface
{
public:
  bool isOk {true};
  bool ok() const override
  {
    return isOk;
  }
};

/**
 * Test that the sleep in InterruptibleSleepInterface has the right duration.
 */
TEST(TimeUtils, SleepInterfaceSimTime)  // NOLINT
{
  Time::init();
  Time::setNow({10, 0});

  TestSleepInterface i;

  // Test normal sleep behavior without interruption.

  bool started = false;
  bool executed = false;
  std::thread([&](){started = true;
    EXPECT_TRUE(i.sleep({1, 0}));
  executed = true;}).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::Time::setNow(ros::Time(10.99));

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_FALSE(executed);

  ros::Time::setNow(ros::Time(11, 0));

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_TRUE(executed);

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
}

/**
 * Test that the sleep in InterruptibleSleepInterface has the right duration.
 */
TEST(TimeUtils, SleepInterfaceWallTime)  // NOLINT
{
  Time::init();
  ASSERT_TRUE(Time::isSystemTime());

  TestSleepInterface i;

  // Test normal sleep behavior without interruption.

  bool started = false;
  bool executed = false;
  std::thread([&](){started = true;
    auto startTime = ros::WallTime::now();
    EXPECT_TRUE(i.sleep({1, 0}));
    auto duration = ros::WallTime::now() - startTime;
    EXPECT_GT(1.1, duration.toSec());
    EXPECT_LT(1.0, duration.toSec());
  executed = true;}).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(1.5);
  while ((!started || !executed) && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_TRUE(executed);
}

/**
 * Test that the sleeps in InterruptibleSleepInterface can be interrupted by various methods.
 */
TEST(TimeUtils, SleepInterfaceInterrupt)  // NOLINT
{
  Time::init();
  Time::setNow({10, 0});

  TestSleepInterface i;

  // Test normal sleep behavior without interruption.

  bool started = false;
  bool executed = false;
  std::thread([&](){started = true;
    EXPECT_TRUE(i.sleep({1, 0}));
  executed = true;}).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.2).sleep();
  EXPECT_FALSE(executed);

  ros::Time::setNow({11, 1000});

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test interrupting a running sleep by setting ok() to false.

  ros::Time::setNow({10, 0});

  started = false;
  executed = false;
  std::thread([&](){started = true;
    EXPECT_FALSE(i.sleep({1, 0}));
  executed = true;}).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.2).sleep();
  EXPECT_FALSE(executed);

  i.isOk = false;

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test two simultaneous sleeps where the second one should end earlier than the first one.

  ros::Time::setNow({10, 0});
  i.isOk = true;

  started = false;
  executed = false;
  std::thread([&](){started = true;
    EXPECT_TRUE(i.sleep({1, 0}));
  executed = true;}).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  auto started2 = false;
  auto executed2 = false;
  std::thread([&](){started2 = true;
    EXPECT_TRUE(i.sleep(ros::Duration(0.1)));
  executed2 = true;}).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started2 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_TRUE(started2);
  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  ros::WallDuration(0.2).sleep();
  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  ros::Time::setNow(ros::Time(10.11));

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed2 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_FALSE(executed);
  EXPECT_TRUE(executed2);

  ros::Time::setNow({11, 1000});

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test simultaneous sleeps and setting ok() to false during them.

  ros::Time::setNow({10, 0});
  i.isOk = true;

  started = false;
  executed = false;
  std::thread([&](){started = true;
    EXPECT_FALSE(i.sleep({1, 0}));
  executed = true;}).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  started2 = false;
  executed2 = false;
  std::thread([&](){started2 = true;
    EXPECT_FALSE(i.sleep({1, 0}));
  executed2 = true;}).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started2 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_TRUE(started2);
  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  ros::WallDuration(0.1).sleep();
  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  ros::Time::setNow(ros::Time(10.1));

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed2 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  i.isOk = false;

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);
  EXPECT_TRUE(executed2);

  Time::shutdown();
}

/**
 * \brief Test that InterruptibleSleepInterface can interrupt an ongoing sleep also when using WallTime.
 */
TEST(TimeUtils, SleepInterfaceInterruptWallTime)  // NOLINT
{
  Time::init();
  ASSERT_TRUE(Time::isSystemTime());

  auto i = std::make_shared<TestSleepInterface>();

  auto startTime = ros::WallTime::now();

  bool started = false;
  bool executed = false;
  std::thread([&](){started = true;
    EXPECT_FALSE(i->sleep({10, 0}));
  executed = true;}).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.2).sleep();
  EXPECT_FALSE(executed);

  i->isOk = false;

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  const auto duration = ros::WallTime::now() - startTime;
  EXPECT_GT(10.0, duration.toSec());
}

/**
 * \brief Test that InterruptibleSleepInterface can interrupt an ongoing sleep if it is being destroyed.
 */
TEST(TimeUtils, SleepInterfaceDestructor)  // NOLINT
{
  Time::init();
  Time::setNow({10, 0});

  auto i = std::make_shared<TestSleepInterface>();

  bool started = false;
  bool executed = false;
  std::thread([&](){started = true;
    EXPECT_FALSE(i->sleep({1, 0}));
  executed = true;}).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(0.2);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.2).sleep();
  EXPECT_FALSE(executed);

  bool started2 = false;
  bool executed2 = false;

  // Destroy the sleep interface object and make sure both the destruction and the ongoing sleep have finished on time.

  std::thread([&](){started2 = true;
    i.reset();
  executed2 = true;}).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.2);
  while ((!started2 || !executed2 || !executed) && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);
  EXPECT_TRUE(executed2);
  EXPECT_TRUE(started2);
}

  int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
