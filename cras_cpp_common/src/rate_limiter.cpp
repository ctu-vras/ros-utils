// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Various implementations of rate-limiting algorithms.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

/// HACK: Access the internal clock instance from rate objects.
#include <sstream>
#define private public
#include <rclcpp/rate.hpp>
#undef private

#include <algorithm>
#include <cras_cpp_common/rate_limiter.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <stdexcept>

#include "cras_cpp_common/time_utils.hpp"

namespace cras
{

RateLimiter::RateLimiter(const ::rclcpp::Rate& rate) : RateLimiter(rate.clock_, rate.period())
{
}

RateLimiter::RateLimiter(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& period)
  : rate(period, clock), period(period)
{
  if (this->period < rclcpp::Duration(0, 0))
    throw std::invalid_argument("Negative rate is not supported.");

  this->jumpHandler = this->rate.clock_->create_jump_callback(
    nullptr, std::bind_front(&RateLimiter::onJump, this), this->jumpBackTolerance);
}

RateLimiter::~RateLimiter() = default;

void RateLimiter::reset()
{
  this->lastJump.reset();
}

void RateLimiter::setJumpBackTolerance(const rclcpp::Duration& tolerance)
{
  if (tolerance < rclcpp::Duration(0, 0))
    throw std::invalid_argument("Jump back tolerance cannot be negative");
  this->jumpBackTolerance.min_backward.nanoseconds = -cras::convertDuration<rcl_duration_value_t>(tolerance);
  this->jumpHandler = this->rate.clock_->create_jump_callback(
    nullptr, std::bind_front(&RateLimiter::onJump, this), this->jumpBackTolerance);
}

void RateLimiter::setJumpBackTolerance(const rcl_jump_threshold_t& tolerance)
{
  this->jumpBackTolerance = tolerance;
  this->jumpHandler = this->rate.clock_->create_jump_callback(
    nullptr, std::bind_front(&RateLimiter::onJump, this), this->jumpBackTolerance);
}

void RateLimiter::onJump(const rcl_time_jump_t& timeJump)
{
  this->lastJump = std::make_tuple(this->rate.clock_->now(), timeJump);
}

ThrottleLimiter::ThrottleLimiter(const rclcpp::Rate& rate) : RateLimiter(rate)
{
  this->lastPublishTime.rcl_time_.clock_type = rate.get_type();
}

ThrottleLimiter::ThrottleLimiter(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& period)
  : RateLimiter(clock, period)
{
  this->lastPublishTime.rcl_time_.clock_type = rate.get_type();
}

bool ThrottleLimiter::shouldPublish(const rclcpp::Time& stamp)
{
  // If time jumped back, always allow
  if (this->lastJump.has_value())
  {
    this->lastJump.reset();
    this->lastPublishTime = stamp;
    return true;
  }

  bool result {false};
  if (stamp >= (this->lastPublishTime + this->period))
  {
    result = true;
    this->lastPublishTime = stamp;
  }

  return result;
}

void ThrottleLimiter::reset()
{
  RateLimiter::reset();
  this->lastPublishTime = {0, 0, this->lastPublishTime.get_clock_type()};
}

TokenBucketLimiter::TokenBucketLimiter(const rclcpp::Rate& rate, const size_t bucketCapacity,
  const double initialTokensAvailable)
  : RateLimiter(rate), tokensAvailable(rclcpp::Duration::from_seconds(initialTokensAvailable))
{
  this->bucketCapacity = bucketCapacity;
  this->initialTokensAvailable = (std::min)(initialTokensAvailable, static_cast<double>(bucketCapacity));
  this->tokensAvailable = rclcpp::Duration::from_seconds(this->initialTokensAvailable);
}

TokenBucketLimiter::TokenBucketLimiter(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& period,
  const size_t bucketCapacity, const double initialTokensAvailable)
  : RateLimiter(clock, period), tokensAvailable(rclcpp::Duration::from_seconds(initialTokensAvailable))
{
  this->bucketCapacity = bucketCapacity;
  this->initialTokensAvailable = (std::min)(initialTokensAvailable, static_cast<double>(bucketCapacity));
  this->tokensAvailable = rclcpp::Duration::from_seconds(this->initialTokensAvailable);
}

bool TokenBucketLimiter::shouldPublish(const rclcpp::Time& stamp)
{
  // If time jumped back by a lot, reset
  if (this->lastJump.has_value())
    this->reset();

  // If we're processing the first message, record its stamp and say that dt == 0, so nothing will be added to bucket
  if (this->lastCheckTime.nanoseconds() == 0)
    this->lastCheckTime = stamp;

  // Do not allow if time jumped back just a bit (large jumps are solved above)
  if (stamp < this->lastCheckTime)
  {
    this->lastCheckTime = stamp;
    return false;
  }

  bool result {false};

  const auto dt = stamp - this->lastCheckTime;
  this->lastCheckTime = stamp;

  // Refill rate is 1 token per every period
  this->tokensAvailable += dt / this->period;

  // Limit by bucket capacity
  this->tokensAvailable = (std::min)(
    this->tokensAvailable, rclcpp::Duration(std::chrono::nanoseconds(this->bucketCapacity * 1000000000)));

  // If there is at least one whole token in the bucket, allow publishing
  if (this->tokensAvailable >= rclcpp::Duration(1, 0))
  {
    result = true;
    this->tokensAvailable -= rclcpp::Duration(1, 0);
  }

  return result;
}

void TokenBucketLimiter::reset()
{
  RateLimiter::reset();
  this->lastCheckTime = {0, 0, this->lastCheckTime.get_clock_type()};
  this->tokensAvailable = rclcpp::Duration::from_seconds(this->initialTokensAvailable);
}

}
