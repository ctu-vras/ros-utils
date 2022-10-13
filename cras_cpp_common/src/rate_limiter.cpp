/**
 * \file
 * \brief Various implementations of rate-limiting algorithms.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <algorithm>
#include <stdexcept>

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

#include <cras_cpp_common/rate_limiter.h>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

RateLimiter::RateLimiter(const ros::Rate& rate) : rate(rate), period(rate.expectedCycleTime())
{
  if (this->period < ros::Duration(0))
    throw std::invalid_argument("Negative rate is not supported.");
}

RateLimiter::RateLimiter(const ros::Duration& period) : rate({period}), period(period)
{
  if (this->period < ros::Duration(0))
    throw std::invalid_argument("Negative rate is not supported.");
}

void RateLimiter::setJumpBackTolerance(const ros::Duration& tolerance)
{
  if (tolerance < ros::Duration(0, 0))
    throw std::invalid_argument("Jump back tolerance cannot be negative");
  this->jumpBackTolerance = tolerance;
}

bool RateLimiter::jumpedBack(const ::ros::Time& stamp, const ::ros::Time& previousStamp) const
{
  // It is important to not subtract the tolerance on the right - we could get into negative time, which is not allowed
  return stamp + this->jumpBackTolerance < previousStamp;
}

ThrottleLimiter::ThrottleLimiter(const ros::Rate& rate) : RateLimiter(rate)
{
}

ThrottleLimiter::ThrottleLimiter(const ros::Duration& period) : RateLimiter(period)
{
}

bool ThrottleLimiter::shouldPublish(const ros::Time& stamp)
{
  // If time jumped back, always allow
  if (this->jumpedBack(stamp, this->lastPublishTime))
  {
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
  this->lastPublishTime = {0, 0};
}

TokenBucketLimiter::TokenBucketLimiter(const ros::Rate& rate, const size_t bucketCapacity,
  const double initialTokensAvailable) : RateLimiter(rate)
{
  this->bucketCapacity = bucketCapacity;
  this->initialTokensAvailable = (std::min)(initialTokensAvailable, static_cast<double>(bucketCapacity));
  this->tokensAvailable = ros::Duration(this->initialTokensAvailable);
}

TokenBucketLimiter::TokenBucketLimiter(const ros::Duration& period, const size_t bucketCapacity,
  const double initialTokensAvailable) : RateLimiter(period)
{
  this->bucketCapacity = bucketCapacity;
  this->initialTokensAvailable = (std::min)(initialTokensAvailable, static_cast<double>(bucketCapacity));
  this->tokensAvailable = ros::Duration(this->initialTokensAvailable);
}

bool TokenBucketLimiter::shouldPublish(const ros::Time& stamp)
{
  // If time jumped back by a lot, reset
  if (this->jumpedBack(stamp, this->lastCheckTime))
    this->reset();

  // If we're processing the first message, record its stamp and say that dt == 0, so nothing will be added to bucket
  if (this->lastCheckTime == ros::Time(0))
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
  this->tokensAvailable += (dt / this->period);

  // Limit by bucket capacity
  this->tokensAvailable = (std::min)(this->tokensAvailable, ros::Duration(this->bucketCapacity, 0));

  // If there is at least one whole token in the bucket, allow publishing
  if (this->tokensAvailable >= ros::Duration(1, 0))
  {
    result = true;
    this->tokensAvailable -= ros::Duration(1, 0);
  }

  return result;
}

void TokenBucketLimiter::reset()
{
  this->lastCheckTime = {0, 0};
  this->tokensAvailable = ros::Duration(this->initialTokensAvailable);
}

}
