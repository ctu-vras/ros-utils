#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a nodelet that republishes heartbeat of a topic with header.
 * \note It can process the messages on a single topic in parallel allowing for maximum throughput.
 * \author Martin Pecka
 */

#include <memory>

#include <ros/message_event.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_msgs/Heartbeat.h>

#include <cras_topic_tools/lazy_subscriber.hpp>

namespace cras
{

/**
 * \brief Nodelet that republishes heartbeat of a topic with header.
 *
 * ROS parameters:
 * - `~queue_size` (uint, default 10): Queue size for the subscriber and publisher.
 * - `~lazy` (bool, default False): Whether to shut down the subscriber when the publisher has no subscribers.
 * - `~tcp_no_delay` (bool, default False): If True, the `TCP_NODELAY` flag is set for the subscriber. This should
 *                                         decrease the latency of small messages, but might give suboptimal
 *                                         transmission speed for large messages.
 *
 * Subscribed topics:
 * - `~input` (any type with header): The input messages. If `lazy` is true, it will only be subscribed when
 *                                    `~input/heartbeat` has some subscribers.
 *                                    If you subscribe to a message type without header, the node should not crash, but
 *                                    it will discard all received messages and print an error message.
 *
 * Published topics:
 * - `~input/heartbeat` (`cras_msgs/Heartbeat`): The heartbeat.
 *
 * Command-line arguments:
 * This nodelet (or node) can also be called in a way backwards compatible with topic_tools nodes. This means you can
 * pass CLI arguments specifying the topic to subscribe.
 * - `rosrun cras_topic_tools heartbeat [TOPIC_IN]`
 *   - `TOPIC_IN`: The topic to subscribe. It is resolved against parent namespace of the node(let) (as opposed to the
 *                 `~input` topic which is resolved against the private node(let) namespace). The heartbeat topic will
 *                 be `TOPIC_IN/heartbeat`.
 */
class HeartbeatNodelet : public ::cras::Nodelet
{
protected:
  //! \brief The lazy subscriber type.
  typedef LazySubscriber<cras_msgs::Heartbeat, const ros::MessageEvent<const topic_tools::ShapeShifter>&>
    SubscriberType;

  void onInit() override;

  /**
   * \brief Process the received message.
   * \param [in] event The message event containing the received message. 
   */
  virtual void processMessage(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event);

  //! \brief The publisher of heartbeat messages.
  ros::Publisher pub;

  //! \brief The lazy subscriber.
  std::unique_ptr<SubscriberType> sub;

  //! \brief Whether the subscribed topic has a header field. This is filled on receipt of the first message.
  cras::optional<bool> hasHeader {cras::nullopt};
};

}
