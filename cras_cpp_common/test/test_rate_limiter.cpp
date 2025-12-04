// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for rate_limiter.h .
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

#include <cras_cpp_common/rate_limiter.h>
#include <cras_cpp_common/time_utils.hpp>

using namespace cras;

rclcpp::Clock::SharedPtr createTestClock()
{
  const auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  const auto ret = rcl_enable_ros_time_override(clock->get_clock_handle());
  if (ret != RMW_RET_OK)
    rclcpp::exceptions::throw_from_rcl_error(ret, "Error creating clock");
  return clock;
}

void setTime(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Time& time)
{
  const auto ret = rcl_set_ros_time_override(
    clock->get_clock_handle(), cras::convertTime<rcl_time_point_value_t>(time));
  if (ret != RMW_RET_OK)
    rclcpp::exceptions::throw_from_rcl_error(ret, "Error setting time");
}

void setTime(const rclcpp::Clock::SharedPtr& clock, const double time)
{
  setTime(clock, convertTime(time, RCL_ROS_TIME));
}

std::vector<rclcpp::Time> createRegularSequence(
  const rclcpp::Time& start, const rclcpp::Duration& period, const size_t numTimes)
{
  const rclcpp::Logger log = rclcpp::get_logger("test_logger");

  std::vector<rclcpp::Time> result;
  result.resize(numTimes);
  for (size_t i = 0; i < numTimes; ++i)
    result[i] = start + period * i;
  return result;
}

TEST(ThrottleLimiter, BadConstruct)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  EXPECT_THROW(cras::ThrottleLimiter(clk, rclcpp::Duration(-1, 0)), std::invalid_argument);
}

TEST(ThrottleLimiter, RegularSequence)  // NOLINT
{
  cras::ThrottleLimiter limiter(rclcpp::Rate(7));

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration::from_seconds(0.1), 10);
  const std::vector<bool> results = {true, false, true, false, true, false, true, false, true, false};

  for (size_t i = 0; i < times.size(); ++i)
  {
    SCOPED_TRACE("Iteration " + std::to_string(i));
    EXPECT_EQ(results[i], limiter.shouldPublish(times[i]));
  }
}

TEST(ThrottleLimiter, RegularSequenceRatio)  // NOLINT
{
  cras::ThrottleLimiter limiter(rclcpp::Rate(7));

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration::from_seconds(0.1), 1000);
  size_t numPublished {0};

  for (const auto& time : times)
    numPublished += limiter.shouldPublish(time);

  EXPECT_EQ(500, numPublished);  // Ideally 700. But throttle is not very good at achieving the 70% throughput rate.
}

TEST(ThrottleLimiter, IrregularSequence)  // NOLINT
{
  cras::ThrottleLimiter limiter(rclcpp::Rate(7));

  const std::vector<rclcpp::Time> times = {
    cras::convertTime<rclcpp::Time>(1.0), cras::convertTime<rclcpp::Time>(1.01),
    cras::convertTime<rclcpp::Time>(1.02), cras::convertTime<rclcpp::Time>(1.03),
    cras::convertTime<rclcpp::Time>(1.15), cras::convertTime<rclcpp::Time>(1.16),
    cras::convertTime<rclcpp::Time>(1.17), cras::convertTime<rclcpp::Time>(1.27),
    cras::convertTime<rclcpp::Time>(1.30), cras::convertTime<rclcpp::Time>(1.31),
    cras::convertTime<rclcpp::Time>(1.32), cras::convertTime<rclcpp::Time>(1.33),
  };
  // the period is slightly less than 0.15 seconds
  const std::vector<bool> results = {
    true, false, false, false,
    true, false, false, false,
    true, false, false, false
  };

  for (size_t i = 0; i < times.size(); ++i)
  {
    SCOPED_TRACE("Iteration " + std::to_string(i));
    EXPECT_EQ(results[i], limiter.shouldPublish(times[i]));
  }
}

TEST(ThrottleLimiter, Reset)  // NOLINT
{
  cras::ThrottleLimiter limiter(rclcpp::Rate(1));

  EXPECT_TRUE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.0)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.1)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.2)));

  limiter.reset();
  EXPECT_TRUE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.3)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.4)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.5)));
}

TEST(ThrottleLimiter, JumpBack)  // NOLINT
{
  const auto clk = createTestClock();
  cras::ThrottleLimiter limiter(rclcpp::Rate(1, clk));

  setTime(clk, 10.0); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
  setTime(clk, 10.1); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 9.9); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 8.9); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 7.9); EXPECT_FALSE(limiter.shouldPublish(clk->now()));

  // Jump back more than 3 seconds
  setTime(clk, 1.3); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
  setTime(clk, 1.4); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 1.5); EXPECT_FALSE(limiter.shouldPublish(clk->now()));

  EXPECT_THROW(limiter.setJumpBackTolerance(rclcpp::Duration(-1,0)), std::invalid_argument);

  // Set jump tolerance to 5
  setTime(clk, 10.0); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
  limiter.setJumpBackTolerance(rclcpp::Duration(5, 0));
  // And jump by 4 seconds, should not trigger reset
  setTime(clk, 6.0); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  // Now jump by more than 5 seconds, should trigger reset
  setTime(clk, 1.0); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
}

TEST(TokenBucketLimiter, BadConstruct)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  EXPECT_THROW(cras::TokenBucketLimiter(rclcpp::Rate(-1)), std::invalid_argument);
  EXPECT_THROW(cras::TokenBucketLimiter(clk, rclcpp::Duration(-1,0)), std::invalid_argument);
}

TEST(TokenBucketLimiter, RegularSequence)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  std::vector<cras::TokenBucketLimiter*> limiters;
  std::map<cras::TokenBucketLimiter*, std::string> names;

  cras::TokenBucketLimiter limiter03(clk, rclcpp::Duration(3, 0), 2, 1); limiters.push_back(&limiter03); names[&limiter03] = "03";  // NOLINT
  cras::TokenBucketLimiter limiter05(rclcpp::Rate(0.5), 2, 1); limiters.push_back(&limiter05); names[&limiter05] = "05";
  cras::TokenBucketLimiter limiter1(rclcpp::Rate(1), 2, 1); limiters.push_back(&limiter1); names[&limiter1] = "1";
  cras::TokenBucketLimiter limiter2(rclcpp::Rate(2), 2, 1); limiters.push_back(&limiter2); names[&limiter2] = "2";
  cras::TokenBucketLimiter limiter4(rclcpp::Rate(4), 2, 1); limiters.push_back(&limiter4); names[&limiter4] = "4";
  cras::TokenBucketLimiter limiter5(rclcpp::Rate(5), 2, 1); limiters.push_back(&limiter5); names[&limiter5] = "5";
  cras::TokenBucketLimiter limiter7(rclcpp::Rate(7), 2, 1); limiters.push_back(&limiter7); names[&limiter7] = "7";
  cras::TokenBucketLimiter limiter10(rclcpp::Rate(10), 2, 1); limiters.push_back(&limiter10); names[&limiter10] = "10";
  cras::TokenBucketLimiter limiter20(rclcpp::Rate(20), 2, 1); limiters.push_back(&limiter20); names[&limiter20] = "20";

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration::from_seconds(0.1), 32);

  std::map<cras::TokenBucketLimiter*, std::vector<bool>> expected = {
    //             1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
    {&limiter03, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}},  // NOLINT
    {&limiter05, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},  // NOLINT
    {&limiter1,  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}},  // NOLINT
    {&limiter2,  { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}},  // NOLINT
    {&limiter4,  { 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0}},  // NOLINT
    {&limiter5,  { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0}},  // NOLINT
    {&limiter7,  { 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0}},  // NOLINT
    {&limiter10, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},  // NOLINT
    {&limiter20, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},  // NOLINT
  };

  std::map<cras::TokenBucketLimiter*, std::vector<bool>> results;

  for (const auto& time : times)
  {
    for (const auto& limiter : limiters)
      results[limiter].push_back(limiter->shouldPublish(time));
  }

  for (const auto& limiter : limiters)
  {
    SCOPED_TRACE("limiter" + names[limiter]);
    EXPECT_EQ(expected[limiter], results[limiter]);
  }
}

TEST(TokenBucketLimiter, RegularSequenceRatio)  // NOLINT
{
  std::vector<double> rates = {0.3, 0.5, 1, 2, 4, 5, 7, 10, 20};

  std::map<cras::TokenBucketLimiter*, std::string> names;
  std::vector<std::unique_ptr<cras::TokenBucketLimiter>> limiters;
  std::map<cras::TokenBucketLimiter*, size_t> numPublished;
  std::map<cras::TokenBucketLimiter*, size_t> expectedPublished;

  for (const auto& rate : rates)
  {
    auto limiter = std::make_unique<cras::TokenBucketLimiter>(rclcpp::Rate(rate));
    names[limiter.get()] = "limiter " + std::to_string(rate);
    numPublished[limiter.get()] = 0;
    const auto expectedRate = (std::min)(10.0, rate);
    expectedPublished[limiter.get()] = 100 * expectedRate;
    limiters.push_back(std::move(limiter));
  }

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration::from_seconds(0.1), 1000);
  for (const auto& time : times)
  {
    for (const auto& limiter : limiters)
      numPublished[limiter.get()] += limiter->shouldPublish(time);
  }

  for (const auto& limiter : limiters)
  {
    SCOPED_TRACE(names[limiter.get()]);
    EXPECT_NEAR(expectedPublished[limiter.get()], numPublished[limiter.get()], 1);
  }
}

TEST(TokenBucketLimiter, IrregularSequence)  // NOLINT
{
  cras::TokenBucketLimiter limiter(rclcpp::Rate(7), 2, 1);

  const std::vector<rclcpp::Time> times = {
    cras::convertTime<rclcpp::Time>(1.0), cras::convertTime<rclcpp::Time>(1.01),
    cras::convertTime<rclcpp::Time>(1.02), cras::convertTime<rclcpp::Time>(1.03),
    cras::convertTime<rclcpp::Time>(1.15), cras::convertTime<rclcpp::Time>(1.16),
    cras::convertTime<rclcpp::Time>(1.17), cras::convertTime<rclcpp::Time>(1.27),
    cras::convertTime<rclcpp::Time>(1.30), cras::convertTime<rclcpp::Time>(1.31),
    cras::convertTime<rclcpp::Time>(1.32), cras::convertTime<rclcpp::Time>(1.33),
  };
  // the period is slightly less than 0.15 seconds
  const std::vector<bool> results = {
    true, false, false, false,
    true, false, false, false,
    true, false, false, false
  };

  for (size_t i = 0; i < times.size(); ++i)
  {
    SCOPED_TRACE("Iteration " + std::to_string(i));
    EXPECT_EQ(results[i], limiter.shouldPublish(times[i]));
  }
}

TEST(TokenBucketLimiter, Reset)  // NOLINT
{
  cras::TokenBucketLimiter limiter(rclcpp::Rate(1), 2, 1);

  EXPECT_TRUE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.0)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.1)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.2)));

  limiter.reset();
  EXPECT_TRUE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.3)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.4)));
  EXPECT_FALSE(limiter.shouldPublish(cras::convertTime<rclcpp::Time>(1.5)));
}

TEST(TokenBucketLimiter, JumpBack)  // NOLINT
{
  const auto clk = createTestClock();
  cras::TokenBucketLimiter limiter(rclcpp::Rate(1, clk), 2, 1);

  setTime(clk, 10.0); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
  setTime(clk, 10.1); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 9.9); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 8.9); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 7.9); EXPECT_FALSE(limiter.shouldPublish(clk->now()));

  // Jump back more than 3 seconds
  setTime(clk, 1.3); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
  setTime(clk, 1.4); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  setTime(clk, 1.5); EXPECT_FALSE(limiter.shouldPublish(clk->now()));

  EXPECT_THROW(limiter.setJumpBackTolerance(rclcpp::Duration(-1, 0)), std::invalid_argument);

  // Set jump tolerance to 5
  setTime(clk, 10.0); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
  limiter.setJumpBackTolerance(rclcpp::Duration(5, 0));
  // And jump by 4 seconds, should not trigger reset
  setTime(clk, 6.0); EXPECT_FALSE(limiter.shouldPublish(clk->now()));
  // Now jump by more than 5 seconds, should trigger reset
  setTime(clk, 1.0); EXPECT_TRUE(limiter.shouldPublish(clk->now()));
}

TEST(TokenBucketLimiter, Params)  // NOLINT
{
  cras::TokenBucketLimiter emptyStart(rclcpp::Rate(1), 2, 0);
  cras::TokenBucketLimiter fullStart(rclcpp::Rate(1), 2, 2);

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.0)));
  EXPECT_TRUE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.0)));

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.1)));
  EXPECT_TRUE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.1)));

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.2)));
  EXPECT_FALSE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.2)));

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.9)));
  EXPECT_FALSE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(10.9)));

  EXPECT_TRUE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(11.1)));
  EXPECT_TRUE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(11.1)));

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(11.2)));
  EXPECT_FALSE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(11.2)));

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(11.9)));
  EXPECT_FALSE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(11.9)));

  EXPECT_TRUE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(12.1)));
  EXPECT_TRUE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(12.1)));

  EXPECT_FALSE(emptyStart.shouldPublish(cras::convertTime<rclcpp::Time>(12.2)));
  EXPECT_FALSE(fullStart.shouldPublish(cras::convertTime<rclcpp::Time>(12.2)));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
