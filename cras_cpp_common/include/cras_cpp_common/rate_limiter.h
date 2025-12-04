#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Various implementations of rate-limiting algorithms.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <optional>
#include <tuple>

#include <rcl/time.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

namespace cras
{

/**
 * \brief Generic rate-limiter interface.
 */
class RateLimiter
{
public:
  /**
   * \brief Create limiter with the given rate.
   * \param[in] rate The desired rate of messages.
   */
  explicit RateLimiter(const ::rclcpp::Rate& rate);

  /**
   * \brief Create limiter with rate corresponding to the given period.
   * \param[in] clock The clock to use.
   * \param[in] period Average delay between two desired output messages.
   */
  explicit RateLimiter(const ::rclcpp::Clock::SharedPtr& clock, const ::rclcpp::Duration& period);

  virtual ~RateLimiter();

  /**
   * \brief Call this function whenever a message is received. It tells whether the message has passed the rate-limiting
   *        and should be published, or whether it should be skipped.
   * \param[in] stamp Time when the message should be sent (usually not header.stamp!).
   * \return Whether to continue publishing the message.
   */
  virtual bool shouldPublish(const ::rclcpp::Time& stamp) = 0;

  /**
   * \brief Reset the rate-limiter as if it were newly created with the same parameters.
   */
  virtual void reset();

  /**
   * \brief Set the limit for recognizing backward time jumps.
   * \param[in] tolerance If a new message comes that is `tolerance` seconds older than the last seen message, it should
   *                      be treated as a jump.
   */
  void setJumpBackTolerance(const ::rclcpp::Duration& tolerance);

  /**
   * \brief Set the limit for recognizing time jumps.
   * \param[in] tolerance The tolerance parameters.
   */
  void setJumpBackTolerance(const ::rcl_jump_threshold_t& tolerance);

protected:
  /**
   * \brief Callback called when time jumped.
   * \param[in] timeJump Parameters of the time jump.
   */
  virtual void onJump(const rcl_time_jump_t& timeJump);

  //! \brief The desired rate (1/period).
  ::rclcpp::Rate rate;

  //! \brief The desired period between message (1/rate).
  ::rclcpp::Duration period;

  //! \brief Threshold for time jump detection.
  ::rcl_jump_threshold_t jumpBackTolerance {true, 0, -3000000000};

  //! \brief Handler of time jumps.
  ::rclcpp::JumpHandler::SharedPtr jumpHandler {nullptr};

  //! \brief Parameters of the last time jump.
  ::std::optional<::std::tuple<::rclcpp::Time, ::rcl_time_jump_t>> lastJump {};
};

/**
 * \brief The (not so clever) algorithm used by topic_tools/throttle node.
 * \note It is not very good at achieving the requested if it isn't orders of magnitude smaller than the incoming rate.
 */
class ThrottleLimiter : public ::cras::RateLimiter
{
public:
  explicit ThrottleLimiter(const ::rclcpp::Rate& rate);
  explicit ThrottleLimiter(const ::rclcpp::Clock::SharedPtr& clock, const ::rclcpp::Duration& period);

  bool shouldPublish(const ::rclcpp::Time& stamp) override;
  void reset() override;

protected:
  //! \brief Stamp of the last message for which `shouldPublish()` returned true.
  ::rclcpp::Time lastPublishTime {0, 0};
};

/**
 * \brief Token bucket rate limiting algorithm.
 * \note Generally, it should be quite good at achieving the desired rate.
 * \note The limiter has a bucket of a given capacity. It is refilled over time by a constant number of tokens per
 *       second. To publish a message, there has to be at least one token in the bucket. If more tokens should be put
 *       in the bucket than is its capacity, they are ignored.
 * \note The bucket capacity basically specifies the size of the burst that can happen after some period of inactivity
 *       when tokens are just collected and not consumed.
 */
class TokenBucketLimiter : public ::cras::RateLimiter
{
public:
  /**
   * \brief Create the rate-limiter limiting to the desired rate.
   * \param[in] rate Desired rate.
   * \param[in] bucketCapacity Capacity of the bucket (in tokens).
   * \param[in] initialTokensAvailable Number of tokens available in the bucket at the beginning. Set to 1 to always
   *                                   let the first packet through. This number should not be higher than
   *                                   `bucketCapacity`.
   */
  explicit TokenBucketLimiter(const ::rclcpp::Rate& rate, size_t bucketCapacity = 2,
    double initialTokensAvailable = 1.0);

  /**
   * \brief Create rate-limiter with rate corresponding to the given period.
   * \param[in] clock The clock to use.
   * \param[in] period Average delay between two desired output messages.
   * \param[in] bucketCapacity Capacity of the bucket (in tokens).
   * \param[in] initialTokensAvailable Number of tokens available in the bucket at the beginning. Set to 1 to always
   *                                   let the first packet through. This number should not be higher than
   *                                   `bucketCapacity`.
   */
  explicit TokenBucketLimiter(const ::rclcpp::Clock::SharedPtr& clock, const ::rclcpp::Duration& period,
    size_t bucketCapacity = 2, double initialTokensAvailable = 1.0);

  bool shouldPublish(const ::rclcpp::Time& stamp) override;
  void reset() override;

protected:
  //! \brief Stamp of the last incoming message. Zero at the beginning.
  ::rclcpp::Time lastCheckTime {0, 0};

  //! \brief Number of tokens that can fit into the bucket. This influences the maximum burst size.
  size_t bucketCapacity;

  //! \brief The number of currently available tokens. This units of this number are actually not seconds, but Duration
  //! is used here to achieve higher decimal point accuracy.
  ::rclcpp::Duration tokensAvailable;

  //! \brief The number of tokens that are initially in the buffer (and after reset).
  double initialTokensAvailable;
};

}
