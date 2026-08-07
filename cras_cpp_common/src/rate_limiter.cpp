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

namespace cras {

RateLimiter::RateLimiter(const ::rclcpp::Rate& rate) : RateLimiter(rate.clock_, rate.period()) {
}

RateLimiter::RateLimiter(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& period)
    : rate_(period, clock), period_(period) {
  if (period_ < rclcpp::Duration(0, 0)) {
    throw std::invalid_argument("Negative rate is not supported.");
  }

  jump_handler_ = rate_.clock_->create_jump_callback(
    nullptr, std::bind_front(&RateLimiter::onJump, this), jump_back_tolerance_);
}

RateLimiter::~RateLimiter() = default;

void RateLimiter::reset() {
  last_jump_.reset();
}

void RateLimiter::setJumpBackTolerance(const rclcpp::Duration& tolerance) {
  if (tolerance < rclcpp::Duration(0, 0)) {
    throw std::invalid_argument("Jump back tolerance cannot be negative");
  }
  jump_back_tolerance_.min_backward.nanoseconds = -cras::convertDuration<rcl_duration_value_t>(tolerance);
  jump_handler_ = rate_.clock_->create_jump_callback(
    nullptr, std::bind_front(&RateLimiter::onJump, this), jump_back_tolerance_);
}

void RateLimiter::setJumpBackTolerance(const rcl_jump_threshold_t& tolerance) {
  jump_back_tolerance_ = tolerance;
  jump_handler_ = rate_.clock_->create_jump_callback(
    nullptr, std::bind_front(&RateLimiter::onJump, this), jump_back_tolerance_);
}

void RateLimiter::onJump(const rcl_time_jump_t& time_jump) {
  last_jump_ = std::make_tuple(rate_.clock_->now(), time_jump);
}

ThrottleLimiter::ThrottleLimiter(const rclcpp::Rate& rate) : RateLimiter(rate) {
  last_publish_time_.rcl_time_.clock_type = rate.get_type();
}

ThrottleLimiter::ThrottleLimiter(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& period)
    : RateLimiter(clock, period) {
  last_publish_time_.rcl_time_.clock_type = rate_.get_type();
}

bool ThrottleLimiter::shouldPublish(const rclcpp::Time& stamp) {
  // If time jumped back, always allow
  if (last_jump_.has_value()) {
    last_jump_.reset();
    last_publish_time_ = stamp;
    return true;
  }

  auto result {false};
  if (stamp >= (last_publish_time_ + period_)) {
    result = true;
    last_publish_time_ = stamp;
  }

  return result;
}

void ThrottleLimiter::reset() {
  RateLimiter::reset();
  last_publish_time_ = {0, 0, last_publish_time_.get_clock_type()};
}

TokenBucketLimiter::TokenBucketLimiter(
    const rclcpp::Rate& rate, const size_t bucket_capacity, const double initial_tokens_available)
    : RateLimiter(rate), tokens_available_(rclcpp::Duration::from_seconds(initial_tokens_available)) {
  bucket_capacity_ = bucket_capacity;
  initial_tokens_available_ = (std::min)(initial_tokens_available, static_cast<double>(bucket_capacity));
  tokens_available_ = rclcpp::Duration::from_seconds(initial_tokens_available_);
}

TokenBucketLimiter::TokenBucketLimiter(
    const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& period, const size_t bucket_capacity,
    const double initial_tokens_available)
    : RateLimiter(clock, period), tokens_available_(rclcpp::Duration::from_seconds(initial_tokens_available)) {
  bucket_capacity_ = bucket_capacity;
  initial_tokens_available_ = (std::min)(initial_tokens_available, static_cast<double>(bucket_capacity));
  tokens_available_ = rclcpp::Duration::from_seconds(initial_tokens_available_);
}

bool TokenBucketLimiter::shouldPublish(const rclcpp::Time& stamp) {
  // If time jumped back by a lot, reset
  if (last_jump_.has_value()) {
    reset();
  }

  // If we're processing the first message, record its stamp and say that dt == 0, so nothing will be added to bucket
  if (last_check_time_.nanoseconds() == 0) {
    last_check_time_ = stamp;
  }

  // Do not allow if time jumped back just a bit (large jumps are solved above)
  if (stamp < last_check_time_) {
    last_check_time_ = stamp;
    return false;
  }

  auto result {false};

  const auto dt = stamp - last_check_time_;
  last_check_time_ = stamp;

  // Refill rate is 1 token per every period
  tokens_available_ += dt / period_;

  // Limit by bucket capacity
  tokens_available_ = (std::min)(
    tokens_available_, rclcpp::Duration(std::chrono::nanoseconds(bucket_capacity_ * 1000000000)));

  // If there is at least one whole token in the bucket, allow publishing
  if (tokens_available_ >= rclcpp::Duration(1, 0)) {
    result = true;
    tokens_available_ -= rclcpp::Duration(1, 0);
  }

  return result;
}

void TokenBucketLimiter::reset() {
  RateLimiter::reset();
  last_check_time_ = {0, 0, last_check_time_.get_clock_type()};
  tokens_available_ = rclcpp::Duration::from_seconds(initial_tokens_available_);
}

}  // namespace cras
