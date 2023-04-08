#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Count messages on a topic.
 * \author Martin Pecka
 */

#include <mutex>

#include <ros/message_event.h>
#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>

namespace cras
{

/**
 * \brief Nodelet for counting messages and their size.
 *
 * ROS parameters:
 * - `~in_queue_size` (uint, default 1000): Queue size for the subscriber.
 * - `~count` (uint, output parameter): The number of messages received so far.
 * - `~bytes` (uint, output parameter): The size of messages received so far (in bytes).
 * - `~tcp_no_delay` (bool, default False): If True, the `TCP_NODELAY` flag is set for the subscriber. This should
 *                                         decrease the latency of small messages, but might give suboptimal
 *                                         transmission speed for large messages.
 *
 * Subscribed topics:
 * - `~input` (any type): The input messages.
 * - `~reset` (any type): When a message is received on this topic, the counter is reset to zero.
 */
class CountMessagesNodelet : public ::cras::Nodelet
{
  void onInit() override;

  /**
   * \brief Called when the counter should be reset. The incoming message can be of any type and should not be examined.
   */
  virtual void resetCb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>&);

  /**
   * \brief Callback for counting the messages.
   * \param[in] event The message event.
   */
  virtual void cb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);

  //! \brief The message subscriber.
  ::ros::Subscriber sub;

  //! \brief The reset message subscriber.
  ::ros::Subscriber resetSub;

  //! \brief Byte size of the received messages.
  size_t bytes {0};

  //! \brief Number of received messages.
  size_t count {0};

  //! \brief Mutex protecting `count` and `bytes`.
  ::std::mutex mutex;
};

}
