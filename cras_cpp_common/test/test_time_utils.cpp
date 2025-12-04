// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for time_utils.hpp.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <limits>
#include <memory>
#include <thread>

#include <cras_cpp_common/time_utils.hpp>
// #include <cras_cpp_common/time_utils/interruptible_sleep_interface.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

using namespace cras;
using namespace rclcpp;

// // The following testcase has to be the first one because it tests what happens when time is not initialized!
// TEST(TimeUtils, NowFallbackToWall)  // NOLINT
// {
//   // Before time is initialized, nowFallbackToWall() should return wall time
//   auto t = cras::nowFallbackToWall();
//   const auto wall = rclcpp::WallTime::now();
//   const auto wallTime = rclcpp::Time(wall.sec, wall.nsec);
//   EXPECT_LT((wallTime > t) ? (wallTime - t) : (t - wallTime), rclcpp::Duration(0.1));
//
//   // This is an unrelated testcase, but we need to test both before time is initialized
//   EXPECT_THROW(remainingTime({99, 0}, {2, 0}), rclcpp::TimeNotInitializedException);
//
//   // After time initialization, it should return ROS time
//   const auto clock = createTestClock();
//   setTime(clock, {100, 0});
//
//   t = cras::nowFallbackToWall();
//   EXPECT_EQ(rclcpp::Time::now(), t);
//
//   Time::shutdown();
// }

Clock::SharedPtr createTestClock()
{
  const auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  const auto ret = rcl_enable_ros_time_override(clock->get_clock_handle());
  if (ret != RMW_RET_OK)
    exceptions::throw_from_rcl_error(ret, "Error creating clock");
  return clock;
}

void setTime(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Time& time)
{
  const auto ret = rcl_set_ros_time_override(
    clock->get_clock_handle(), cras::convertTime<rcl_time_point_value_t>(time));
  if (ret != RMW_RET_OK)
    exceptions::throw_from_rcl_error(ret, "Error setting time");
}

TEST(TimeUtils, RemainingTimeDuration)  // NOLINT
{
  const auto clock = createTestClock();
  setTime(clock, {100, 0});

  EXPECT_EQ(rclcpp::Duration(9, 0), remainingTime({99, 0, RCL_ROS_TIME}, {10, 0}, clock));
  EXPECT_EQ(rclcpp::Duration(1, 0), remainingTime({99, 0, RCL_ROS_TIME}, {2, 0}, clock));
  EXPECT_EQ(rclcpp::Duration(0, 0), remainingTime({90, 0, RCL_ROS_TIME}, {2, 0}, clock));  // time's up
}

TEST(TimeUtils, RemainingTimeDouble)  // NOLINT
{
  const auto clock = createTestClock();
  setTime(clock, {100, 0});

  EXPECT_EQ(rclcpp::Duration(9, 0), remainingTime({99, 0, RCL_ROS_TIME}, 10.0, clock));
  EXPECT_EQ(rclcpp::Duration(1, 0), remainingTime({99, 0, RCL_ROS_TIME}, 2.0, clock));
  EXPECT_EQ(rclcpp::Duration(0, 0), remainingTime({90, 0, RCL_ROS_TIME}, 2.0, clock));  // time's up
}

TEST(TimeUtils, Frequency)  // NOLINT
{
  EXPECT_NEAR(1.0, frequency(rclcpp::Rate(1.0)), 1e-3);
  EXPECT_NEAR(10.0, frequency(rclcpp::Rate(10.0)), 1e-3);
  EXPECT_NEAR(3.14, frequency(rclcpp::Rate(3.14)), 1e-3);

  EXPECT_NEAR(1.0, frequency(rclcpp::Rate(1.0), true), 1e-3);
  EXPECT_NEAR(10.0, frequency(rclcpp::Rate(10.0), true), 1e-3);
  EXPECT_NEAR(3.14, frequency(rclcpp::Rate(3.14), true), 1e-3);

  EXPECT_NEAR(0, frequency(rclcpp::Rate(rclcpp::Duration::max())), 1e-3);
  EXPECT_NE(0, frequency(rclcpp::Rate(rclcpp::Duration::max())));  // it is very small, but not zero
  EXPECT_EQ(0, frequency(rclcpp::Rate(rclcpp::Duration::max()), true));
}

TEST(TimeUtils, WallFrequency)  // NOLINT
{
  EXPECT_NEAR(1.0, frequency(rclcpp::WallRate(1.0)), 1e-3);
  EXPECT_NEAR(10.0, frequency(rclcpp::WallRate(10.0)), 1e-3);
  EXPECT_NEAR(3.14, frequency(rclcpp::WallRate(3.14)), 1e-3);

  EXPECT_NEAR(1.0, frequency(rclcpp::WallRate(1.0), true), 1e-3);
  EXPECT_NEAR(10.0, frequency(rclcpp::WallRate(10.0), true), 1e-3);
  EXPECT_NEAR(3.14, frequency(rclcpp::WallRate(3.14), true), 1e-3);

  EXPECT_NEAR(0, frequency(rclcpp::WallRate(rclcpp::Duration::max())), 1e-3);
  EXPECT_NE(0, frequency(rclcpp::WallRate(rclcpp::Duration::max())));  // it is very small, but not zero
  EXPECT_EQ(0, frequency(rclcpp::WallRate(rclcpp::Duration::max()), true));
}

TEST(TimeUtils, SafeRate)  // NOLINT
{
  EXPECT_EQ(rclcpp::Duration::max(), safeRate(0).period());
  EXPECT_EQ(rclcpp::Duration::max(), safeRate(1e-10).period());
  EXPECT_GT(rclcpp::Duration::max(), safeRate(1e-9).period());
  EXPECT_EQ(rclcpp::Duration(1, 0), safeRate(1).period());
  EXPECT_EQ(rclcpp::Duration::from_seconds(1e-9), safeRate(1e9).period());
  EXPECT_EQ(rclcpp::Duration(0, 1), safeRate(std::numeric_limits<double>::infinity()).period());
}

TEST(TimeUtils, SafeWallRate)  // NOLINT
{
  EXPECT_EQ(rclcpp::Duration::max(), safeWallRate(0).period());
  EXPECT_EQ(rclcpp::Duration::max(), safeWallRate(1e-10).period());
  EXPECT_GT(rclcpp::Duration::max(), safeWallRate(1e-9).period());
  EXPECT_EQ(rclcpp::Duration(1, 0), safeWallRate(1).period());
  EXPECT_EQ(rclcpp::Duration::from_seconds(1e-9), safeWallRate(1e9).period());
  EXPECT_EQ(rclcpp::Duration(0, 1), safeWallRate(std::numeric_limits<double>::infinity()).period());
}

TEST(TimeUtils, RateEquals)  // NOLINT
{
  EXPECT_TRUE(rclcpp::Rate(1.0) == rclcpp::Rate(1.0));
  EXPECT_TRUE(rclcpp::Rate(2.0) == rclcpp::Rate(2.0));
  EXPECT_TRUE(rclcpp::Rate(3.14) == rclcpp::Rate(3.14));
  EXPECT_TRUE(rclcpp::Rate(1e9) == rclcpp::Rate(1e9));
  EXPECT_TRUE(rclcpp::Rate(rclcpp::Duration::max()) == rclcpp::Rate(rclcpp::Duration::max()));
  EXPECT_FALSE(rclcpp::Rate(3.13) == rclcpp::Rate(3.14));
  EXPECT_FALSE(rclcpp::Rate(3.0) == rclcpp::Rate(3.14));
  EXPECT_FALSE(rclcpp::Rate(1.0) == rclcpp::Rate(2.0));
  EXPECT_FALSE(rclcpp::Rate(rclcpp::Duration(0, 1)) == rclcpp::Rate(rclcpp::Duration::max()));
  EXPECT_FALSE(rclcpp::Rate(rclcpp::Duration::max()) == rclcpp::Rate(rclcpp::Duration(0, 1)));
}

TEST(TimeUtils, WallRateEquals)  // NOLINT
{
  EXPECT_TRUE(rclcpp::WallRate(1.0) == rclcpp::WallRate(1.0));
  EXPECT_TRUE(rclcpp::WallRate(2.0) == rclcpp::WallRate(2.0));
  EXPECT_TRUE(rclcpp::WallRate(3.14) == rclcpp::WallRate(3.14));
  EXPECT_TRUE(rclcpp::WallRate(1e9) == rclcpp::WallRate(1e9));
  EXPECT_TRUE(rclcpp::WallRate(rclcpp::Duration::max()) == rclcpp::WallRate(rclcpp::Duration::max()));
  EXPECT_FALSE(rclcpp::WallRate(3.13) == rclcpp::WallRate(3.14));
  EXPECT_FALSE(rclcpp::WallRate(3.0) == rclcpp::WallRate(3.14));
  EXPECT_FALSE(rclcpp::WallRate(1.0) == rclcpp::WallRate(2.0));
  EXPECT_FALSE(rclcpp::WallRate(rclcpp::Duration(0, 1)) == rclcpp::WallRate(rclcpp::Duration::max()));
  EXPECT_FALSE(rclcpp::WallRate(rclcpp::Duration::max()) == rclcpp::WallRate(rclcpp::Duration(0, 1)));
}

TEST(TimeUtils, DurationMultiply)  // NOLINT
{
  EXPECT_EQ(rclcpp::Duration(2, 0), rclcpp::Duration(1, 0) * rclcpp::Duration(2, 0));
  EXPECT_EQ(rclcpp::Duration(400, 0), rclcpp::Duration(20, 0) * rclcpp::Duration(20, 0));
  EXPECT_EQ(rclcpp::Duration(0, 250000000), rclcpp::Duration(0, 500000000) * rclcpp::Duration(0, 500000000));
  EXPECT_EQ(rclcpp::Duration(0, 0), rclcpp::Duration(0, 1) * rclcpp::Duration(0, 1));  // too small value
  EXPECT_EQ(rclcpp::Duration(0, 0), rclcpp::Duration(1000, 0) * rclcpp::Duration(0, 0));
  EXPECT_EQ(rclcpp::Duration(-1000, 0), rclcpp::Duration(-1000, 0) * rclcpp::Duration(1, 0));
  EXPECT_EQ(rclcpp::Duration(-1000, 0), rclcpp::Duration(1, 0) * rclcpp::Duration(-1000, 0));
  EXPECT_EQ(rclcpp::Duration(1000000, 0), rclcpp::Duration(-1000, 0) * rclcpp::Duration(-1000, 0));
  EXPECT_THROW(rclcpp::Duration(1000000000, 0) * rclcpp::Duration(1000000000, 0), std::invalid_argument);
}

TEST(TimeUtils, DurationDivide)  // NOLINT
{
  EXPECT_EQ(rclcpp::Duration(1, 0), rclcpp::Duration(2, 0) / rclcpp::Duration(2, 0));
  EXPECT_EQ(rclcpp::Duration(0, 0), rclcpp::Duration(0, 0) / rclcpp::Duration(2, 0));
  EXPECT_EQ(rclcpp::Duration(0, 500000000), rclcpp::Duration(1, 0) / rclcpp::Duration(2, 0));
  EXPECT_EQ(rclcpp::Duration(0, 250000000), rclcpp::Duration(1, 0) / rclcpp::Duration(4, 0));
  EXPECT_EQ(rclcpp::Duration(0, 125000000), rclcpp::Duration(1, 0) / rclcpp::Duration(8, 0));
  EXPECT_EQ(rclcpp::Duration(0, 125000000), rclcpp::Duration(2, 0) / rclcpp::Duration(16, 0));
  EXPECT_EQ(rclcpp::Duration(2, 0), rclcpp::Duration(2, 0) / rclcpp::Duration(1, 0));
  EXPECT_EQ(rclcpp::Duration(4, 0), rclcpp::Duration(4, 0) / rclcpp::Duration(1, 0));
  EXPECT_EQ(rclcpp::Duration(8, 0), rclcpp::Duration(8, 0) / rclcpp::Duration(1, 0));
  EXPECT_EQ(rclcpp::Duration(8, 0), rclcpp::Duration(16, 0) / rclcpp::Duration(2, 0));
  EXPECT_EQ(rclcpp::Duration(1, 0), rclcpp::Duration(1000000000, 0) / rclcpp::Duration(1000000000, 0));
  EXPECT_EQ(rclcpp::Duration(-2, 0), rclcpp::Duration(-4, 0) / rclcpp::Duration(2, 0));
  EXPECT_EQ(rclcpp::Duration(-2, 0), rclcpp::Duration(4, 0) / rclcpp::Duration(-2, 0));
  EXPECT_EQ(rclcpp::Duration(2, 0), rclcpp::Duration(-4, 0) / rclcpp::Duration(-2, 0));
  EXPECT_THROW(rclcpp::Duration(1, 0) / rclcpp::Duration(0, 0), std::runtime_error);
  EXPECT_THROW(rclcpp::Duration(1, 0) / rclcpp::Duration(-0, 0), std::runtime_error);
}

TEST(TimeUtils, SaturateAddTime)  // NOLINT
{
  EXPECT_EQ(rclcpp::Time(2, 0), cras::saturateAdd(rclcpp::Time(1, 0), rclcpp::Duration(1, 0)));
  EXPECT_EQ(rclcpp::Time(0, 0), cras::saturateAdd(rclcpp::Time(1, 0), rclcpp::Duration(-1, 0)));
  EXPECT_EQ(rclcpp::Time(0, 0), cras::saturateAdd(rclcpp::Time(1, 0), rclcpp::Duration(-2, 0)));
  EXPECT_EQ(rclcpp::Time(0, 0), cras::saturateAdd(rclcpp::Time(1, 0), rclcpp::Duration(-10, 0)));
  EXPECT_EQ(rclcpp::Time(0, 0), cras::saturateAdd(rclcpp::Time(1, 0), rclcpp::Duration::max() * -1));
  EXPECT_EQ(rclcpp::Time(10, 0), cras::saturateAdd(rclcpp::Time(6, 0), rclcpp::Duration(4, 0)));
  EXPECT_EQ(rclcpp::Time::max(), cras::saturateAdd(rclcpp::Time::max(), rclcpp::Duration(0, 0)));
  EXPECT_EQ(rclcpp::Time::max(), cras::saturateAdd(rclcpp::Time::max(), rclcpp::Duration(1, 0)));
  EXPECT_EQ(rclcpp::Time::max(), cras::saturateAdd(rclcpp::Time::max(), rclcpp::Duration(2, 0)));
  EXPECT_EQ(rclcpp::Time::max(), cras::saturateAdd(rclcpp::Time::max(), rclcpp::Duration::max()));
  builtin_interfaces::msg::Duration maxDur = rclcpp::Duration::max();
  EXPECT_EQ(rclcpp::Time(maxDur.sec, maxDur.nanosec), cras::saturateAdd(rclcpp::Time(0, 0), rclcpp::Duration::max()));
}

// class TestSleepInterface : public cras::InterruptibleSleepInterface
// {
// public:
//   bool isOk {true};
//   bool ok() const override
//   {
//     return isOk;
//   }
// };
//
// /**
//  * Test that the sleep in InterruptibleSleepInterface has the right duration.
//  */
// TEST(TimeUtils, SleepInterfaceSimTime)  // NOLINT
// {
//   const auto clock = createTestClock();
//   setTime(clock, {10, 0});
//
//   TestSleepInterface i;
//
//   // Test normal sleep behavior without interruption.
//
//   bool started = false;
//   bool executed = false;
//   std::thread([&](){started = true;
//     EXPECT_TRUE(i.sleep({1, 0}));
//   executed = true;}).detach();
//
//   auto end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   rclcpp::setTime(clock, rclcpp::Time(10.99));
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//   EXPECT_FALSE(executed);
//
//   rclcpp::setTime(clock, rclcpp::Time(11, 0));
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//   EXPECT_TRUE(executed);
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
// }
//
// /**
//  * Test that the sleep in InterruptibleSleepInterface has the right duration.
//  */
// TEST(TimeUtils, SleepInterfaceWallTime)  // NOLINT
// {
//   const auto clock = createTestClock();
//   ASSERT_TRUE(Time::isSystemTime());
//
//   TestSleepInterface i;
//
//   // Test normal sleep behavior without interruption.
//
//   bool started = false;
//   bool executed = false;
//   std::thread([&](){started = true;
//     auto startTime = rclcpp::WallTime::now();
//     EXPECT_TRUE(i.sleep({1, 0}));
//     auto duration = rclcpp::WallTime::now() - startTime;
//     EXPECT_GT(1.1, duration.seconds());
//     EXPECT_LT(1.0, duration.seconds());
//   executed = true;}).detach();
//
//   auto end = rclcpp::WallTime::now() + rclcpp::WallDuration(1.5);
//   while ((!started || !executed) && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_TRUE(executed);
// }
//
// /**
//  * Test that the sleeps in InterruptibleSleepInterface can be interrupted by various methods.
//  */
// TEST(TimeUtils, SleepInterfaceInterrupt)  // NOLINT
// {
//   const auto clock = createTestClock();
//   setTime(clock, {10, 0});
//
//   TestSleepInterface i;
//
//   // Test normal sleep behavior without interruption.
//
//   bool started = false;
//   bool executed = false;
//   std::thread([&](){started = true;
//     EXPECT_TRUE(i.sleep({1, 0}));
//   executed = true;}).detach();
//
//   auto end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   rclcpp::WallDuration(0.2).sleep();
//   EXPECT_FALSE(executed);
//
//   rclcpp::setTime(clock, {11, 1000});
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(executed);
//
//   // Test interrupting a running sleep by setting ok() to false.
//
//   rclcpp::setTime(clock, {10, 0});
//
//   started = false;
//   executed = false;
//   std::thread([&](){started = true;
//     EXPECT_FALSE(i.sleep({1, 0}));
//   executed = true;}).detach();
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   rclcpp::WallDuration(0.2).sleep();
//   EXPECT_FALSE(executed);
//
//   i.isOk = false;
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(executed);
//
//   // Test two simultaneous sleeps where the second one should end earlier than the first one.
//
//   rclcpp::setTime(clock, {10, 0});
//   i.isOk = true;
//
//   started = false;
//   executed = false;
//   std::thread([&](){started = true;
//     EXPECT_TRUE(i.sleep({1, 0}));
//   executed = true;}).detach();
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   auto started2 = false;
//   auto executed2 = false;
//   std::thread([&](){started2 = true;
//     EXPECT_TRUE(i.sleep(rclcpp::Duration(0.1)));
//   executed2 = true;}).detach();
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started2 && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_TRUE(started2);
//   EXPECT_FALSE(executed);
//   EXPECT_FALSE(executed2);
//
//   rclcpp::WallDuration(0.2).sleep();
//   EXPECT_FALSE(executed);
//   EXPECT_FALSE(executed2);
//
//   rclcpp::setTime(clock, rclcpp::Time(10.11));
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed2 && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_FALSE(executed);
//   EXPECT_TRUE(executed2);
//
//   rclcpp::setTime(clock, {11, 1000});
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(executed);
//
//   // Test simultaneous sleeps and setting ok() to false during them.
//
//   rclcpp::setTime(clock, {10, 0});
//   i.isOk = true;
//
//   started = false;
//   executed = false;
//   std::thread([&](){started = true;
//     EXPECT_FALSE(i.sleep({1, 0}));
//   executed = true;}).detach();
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   started2 = false;
//   executed2 = false;
//   std::thread([&](){started2 = true;
//     EXPECT_FALSE(i.sleep({1, 0}));
//   executed2 = true;}).detach();
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started2 && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_TRUE(started2);
//   EXPECT_FALSE(executed);
//   EXPECT_FALSE(executed2);
//
//   rclcpp::WallDuration(0.1).sleep();
//   EXPECT_FALSE(executed);
//   EXPECT_FALSE(executed2);
//
//   rclcpp::setTime(clock, rclcpp::Time(10.1));
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed2 && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_FALSE(executed);
//   EXPECT_FALSE(executed2);
//
//   i.isOk = false;
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(executed);
//   EXPECT_TRUE(executed2);
//
// }
//
// /**
//  * \brief Test that InterruptibleSleepInterface can interrupt an ongoing sleep also when using WallTime.
//  */
// TEST(TimeUtils, SleepInterfaceInterruptWallTime)  // NOLINT
// {
//   const auto clock = createTestClock();
//   ASSERT_TRUE(Time::isSystemTime());
//
//   auto i = std::make_shared<TestSleepInterface>();
//
//   auto startTime = rclcpp::WallTime::now();
//
//   bool started = false;
//   bool executed = false;
//   std::thread([&](){started = true;
//     EXPECT_FALSE(i->sleep({10, 0}));
//   executed = true;}).detach();
//
//   auto end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   rclcpp::WallDuration(0.2).sleep();
//   EXPECT_FALSE(executed);
//
//   i->isOk = false;
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!executed && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(executed);
//
//   const auto duration = rclcpp::WallTime::now() - startTime;
//   EXPECT_GT(10.0, duration.seconds());
// }
//
// /**
//  * \brief Test that InterruptibleSleepInterface can interrupt an ongoing sleep if it is being destroyed.
//  */
// TEST(TimeUtils, SleepInterfaceDestructor)  // NOLINT
// {
//   const auto clock = createTestClock();
//   setTime(clock, {10, 0});
//
//   auto i = std::make_shared<TestSleepInterface>();
//
//   bool started = false;
//   bool executed = false;
//   std::thread([&](){started = true;
//     EXPECT_FALSE(i->sleep({1, 0}));
//   executed = true;}).detach();
//
//   auto end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while (!started && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(started);
//   EXPECT_FALSE(executed);
//
//   rclcpp::WallDuration(0.2).sleep();
//   EXPECT_FALSE(executed);
//
//   bool started2 = false;
//   bool executed2 = false;
//
//   // Destroy the sleep interface object and make sure both the destruction and the ongoing sleep have finished on time.
//
//   std::thread([&](){started2 = true;
//     i.reset();
//   executed2 = true;}).detach();
//
//   end = rclcpp::WallTime::now() + rclcpp::WallDuration(0.2);
//   while ((!started2 || !executed2 || !executed) && rclcpp::WallTime::now() < end)
//     rclcpp::WallDuration(0.01).sleep();
//
//   EXPECT_TRUE(executed);
//   EXPECT_TRUE(executed2);
//   EXPECT_TRUE(started2);
// }

TEST(TimeUtils, TimeType)  // NOLINT
{
  static_assert(cras::TimeType<rclcpp::Time>::value);
  static_assert(cras::TimeType<rcl_time_point_t>::value);
  static_assert(cras::TimeType<rcl_time_point_value_t>::value);
  static_assert(cras::TimeType<builtin_interfaces::msg::Time>::value);
  static_assert(cras::TimeType<int64_t>::value);
  static_assert(cras::TimeType<double>::value);
  static_assert(cras::TimeType<std::chrono::system_clock::time_point>::value);
  static_assert(cras::TimeType<std::chrono::steady_clock::time_point>::value);

  static_assert(!cras::TimeType<rclcpp::Duration>::value);
  static_assert(!cras::TimeType<rcl_duration_t>::value);
  static_assert(!cras::TimeType<std::chrono::nanoseconds>::value);
  static_assert(!cras::TimeType<builtin_interfaces::msg::Duration>::value);
}

TEST(TimeUtils, TimeSecNsec)  // NOLINT
{
  EXPECT_EQ(123, cras::sec(rclcpp::Time(123, 321)));
  EXPECT_EQ(321, cras::nanosec(rclcpp::Time(123, 321)));
  EXPECT_EQ(std::pair(std::pair<int32_t, uint32_t>{123, 321}), cras::secNsec(rclcpp::Time(123, 321)));

  EXPECT_EQ(0, cras::sec(rclcpp::Time(0, 321)));
  EXPECT_EQ(321, cras::nanosec(rclcpp::Time(0, 321)));
  EXPECT_EQ(std::pair(std::pair<int32_t, uint32_t>{0, 321}), cras::secNsec(rclcpp::Time(0, 321)));
}

TEST(TimeUtils, DurationType)  // NOLINT
{
  static_assert(cras::DurationType<rclcpp::Duration>::value);
  static_assert(cras::DurationType<rcl_duration_t>::value);
  static_assert(cras::DurationType<rcl_duration_value_t>::value);
  static_assert(cras::DurationType<builtin_interfaces::msg::Duration>::value);
  static_assert(cras::DurationType<int64_t>::value);
  static_assert(cras::DurationType<double>::value);
  static_assert(cras::DurationType<std::chrono::nanoseconds>::value);

  static_assert(!cras::DurationType<rclcpp::Time>::value);
  static_assert(!cras::DurationType<rcl_time_point_t>::value);
  static_assert(!cras::DurationType<std::chrono::system_clock::time_point>::value);
  static_assert(!cras::DurationType<builtin_interfaces::msg::Time>::value);
}

TEST(TimeUtils, DurationSecNsec)  // NOLINT
{
  EXPECT_EQ(123, cras::sec(rclcpp::Duration(123, 321)));
  EXPECT_EQ(321, cras::nanosec(rclcpp::Duration(123, 321)));
  EXPECT_EQ(std::pair(std::pair<int32_t, uint32_t>{123, 321}), cras::secNsec(rclcpp::Duration(123, 321)));

  EXPECT_EQ(0, cras::sec(rclcpp::Duration(0, 321)));
  EXPECT_EQ(321, cras::nanosec(rclcpp::Duration(0, 321)));
  EXPECT_EQ(std::pair(std::pair<int32_t, uint32_t>{0, 321}), cras::secNsec(rclcpp::Duration(0, 321)));
}

TEST(TimeUtils, ConvertTime)  // NOLINT
{
  builtin_interfaces::msg::Time timeMsg;
  timeMsg.sec = 1;
  timeMsg.nanosec = 2;

  const rclcpp::Time timeRclcpp(1, 2, RCL_SYSTEM_TIME);
  const rclcpp::Time timeRclcppRos(1, 2, RCL_ROS_TIME);

  EXPECT_EQ(timeRclcpp, cras::convertTime(timeMsg, RCL_SYSTEM_TIME));
  EXPECT_EQ(timeRclcppRos, cras::convertTime(timeMsg, RCL_ROS_TIME));
  EXPECT_EQ(timeMsg, cras::convertTime<builtin_interfaces::msg::Time>(timeRclcpp));
}

TEST(TimeUtils, ConvertDuration)  // NOLINT
{
  builtin_interfaces::msg::Duration durationMsg;
  durationMsg.sec = 1;
  durationMsg.nanosec = 2;

  const rclcpp::Duration durationRclcpp(1, 2);

  EXPECT_EQ(durationRclcpp, cras::convertDuration<rclcpp::Duration>(durationMsg));
  EXPECT_EQ(durationMsg, cras::convertDuration<builtin_interfaces::msg::Duration>(durationRclcpp));
}

bool operator==(const tm& t1, const tm& t2)
{
  return
    t1.tm_year == t2.tm_year &&
    t1.tm_mon == t2.tm_mon &&
    t1.tm_mday == t2.tm_mday &&
    t1.tm_hour == t2.tm_hour &&
    t1.tm_min == t2.tm_min &&
    t1.tm_sec == t2.tm_sec &&
    t1.tm_gmtoff == t2.tm_gmtoff;
}

TEST(TimeUtils, ToStructTm)  // NOLINT
{
  tm t{};
  t.tm_year = 1970 - 1900;
  t.tm_mon = 1 - 1;
  t.tm_mday = 1;
  t.tm_hour = 0;
  t.tm_min = 0;
  t.tm_sec = 0;
  t.tm_isdst = 0;
  t.tm_gmtoff = 0;

  EXPECT_EQ(t, cras::convertTime<tm>(rclcpp::Time()));

  t.tm_year = 2024 - 1900;
  t.tm_mon = 11 - 1;
  t.tm_mday = 13;
  t.tm_hour = 13;
  t.tm_min = 44;
  t.tm_sec = 04;
  t.tm_isdst = 0;
  t.tm_gmtoff = 0;

  EXPECT_EQ(t, cras::convertTime<tm>(rclcpp::Time(1731505444, 0)));
  EXPECT_EQ(t, cras::convertTime<tm>(rclcpp::Time(1731505444, 500000000)));
}

TEST(TimeUtils, GetYear)  // NOLINT
{
  EXPECT_EQ(1970, cras::getYear(rclcpp::Time()));

  EXPECT_EQ(2024, cras::getYear(rclcpp::Time(1731505444, 0)));
  EXPECT_EQ(2024, cras::getYear(rclcpp::Time(1731505444, 500000000)));
}

TEST(TimeUtils, FromStructTm)  // NOLINT
{
  tm t{};

  EXPECT_FALSE(cras::fromStructTm(t).has_value());

  t.tm_year = 1970 - 1900; t.tm_mon = 1 - 1; t.tm_mday = 1; t.tm_hour = 0; t.tm_min = 0; t.tm_sec = 0;
  ASSERT_TRUE(cras::fromStructTm(t).has_value());
  EXPECT_EQ(rclcpp::Time(), cras::fromStructTm(t));

  t.tm_year = 2024 - 1900; t.tm_mon = 11 - 1; t.tm_mday = 13; t.tm_hour = 13; t.tm_min = 44; t.tm_sec = 4;
  ASSERT_TRUE(cras::fromStructTm(t).has_value());
  EXPECT_EQ(rclcpp::Time(1731505444, 0), cras::fromStructTm(t));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
