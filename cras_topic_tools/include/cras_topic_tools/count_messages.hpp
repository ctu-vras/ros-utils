#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Count messages on a topic.
 * \author Martin Pecka
 */

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

namespace cras {

/**
 * \brief Component for counting messages and their size.
 *
 * ROS parameters:
 * - `~in_queue_size` (uint, default 1000): Queue size for the subscriber.
 * - `~qos_profile` (string, default empty string): If nonempty, specifies the QoS profile to use for subscription.
 * - `~count` (uint, output parameter): The number of messages received so far.
 * - `~bytes` (uint, output parameter): The size of messages received so far (in bytes).
 *
 * Subscribed topics:
 * - `~input` (any type): The input messages.
 * - `~reset` (any type): When a message is received on this topic, the counter is reset to zero.
 */
class CountMessagesComponent : public ::rclcpp::Node {
public:
  explicit CountMessagesComponent(const ::rclcpp::NodeOptions& options);

protected:
  /**
   * \brief Called when the counter should be reset. The incoming message can be of any type and should not be examined.
   */
  void resetCb(const std::shared_ptr<const rclcpp::SerializedMessage>&);

  /**
   * \brief Callback for counting the messages.
   * \param[in] message The message event.
   */
  void cb(const std::shared_ptr<const rclcpp::SerializedMessage>& message);

  /**
   * \brief Callback for periodic reports.
   */
  void reportCb();

  /**
   * \brief Check if the input topic is available. Subscribe to it when it is.
   */
  void discoverTopicAndSubscribe();

  /**
   * \brief Check if the reset topic is available. Subscribe to it when it is.
   */
  void discoverResetTopicAndSubscribe();

  //! \brief The message subscriber.
  ::rclcpp::GenericSubscription::SharedPtr sub_;

  //! \brief The reset message subscriber.
  ::rclcpp::GenericSubscription::SharedPtr reset_sub_;

  //! \brief Timer for input topic type discovery.
  ::rclcpp::TimerBase::SharedPtr discovery_timer_;

  //! \brief Timer for reset topic type discovery.
  ::rclcpp::TimerBase::SharedPtr reset_discovery_timer_;

  //! \brief Timer for printing reports.
  ::rclcpp::TimerBase::SharedPtr report_timer_;

  //! \brief The unresolved input topic name.
  std::string topic_ {"input"};

  //! \brief Resolved name of the input topic.
  ::std::string resolved_topic_;

  //! \brief Resolved name of the reset topic.
  ::std::string resolved_reset_topic_;

  //! \brief QoS for the input topic subscription.
  ::rclcpp::QoS qos_profile_ {1000};

  //! \brief Whether the current message and byte count should be set as parameters of the node.
  bool use_params_ {true};

  //! \brief Whether to publish topic statistics for the subscription.
  bool topic_stats_ {false};

  //! \brief Whether to enable intraprocess comms for the subscription.
  bool intraprocess_comms_ {true};

  //! \brief Byte size of the received messages.
  ::size_t bytes_ {0};

  //! \brief Number of received messages.
  ::size_t count_ {0};

  //! \brief Number of received messages since last report.
  ::size_t count_since_last_report_ {0};

  //! \brief Mutex protecting `count_` and `bytes_`.
  ::std::mutex mutex_;

  //! \brief Time when the message count was last reported.
  ::std::optional<::rclcpp::Time> last_report_stamp_;
};

}  // namespace cras
