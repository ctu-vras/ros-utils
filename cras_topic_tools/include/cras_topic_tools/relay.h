#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a simple implementation of a relay nodelet.
 * \note It can process the messages on a single topic in parallel allowing for maximum throughput.
 * \author Martin Pecka
 */

#include <memory>

#include <ros/message_event.h>
#include <ros/publisher.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>

namespace cras
{

/**
 * \brief Nodelet for relaying messages on a different topic.
 *
 * ROS parameters:
 * - `~in_queue_size` (uint, default 10): Queue size for the subscriber.
 * - `~out_queue_size` (uint, default $in_queue_size): Queue size for the publisher.
 * - `~lazy` (bool, default False): Whether to shut down the subscriber when the publisher has no subscribers.
 *                                  The `~input` topic will be subscribed in the beginning, and will unsubscribe
 *                                  automatically after the first message is received (this is needed to determine the
 *                                  full type of the topic to publish).
 * - `~tcp_no_delay` (bool, default False): If True, the `TCP_NODELAY` flag is set for the subscriber. This should
 *                                         decrease the latency of small messages, but might give suboptimal
 *                                         transmission speed for large messages.
 *
 * Subscribed topics:
 * - `~input` (any type): The input messages. If `lazy` is true, it will only be subscribed when `~output` has some
 *                        subscribers.
 *
 * Published topics:
 * - `~output` (same type as `~input`): The relayed output messages.
 *
 * Command-line arguments:
 * This nodelet (or node) can also be called in a way backwards compatible with topic_tools/relay. This means you can
 * pass CLI arguments specifying the topics to subscribe/publish.
 * - `rosrun cras_topic_tools relay TOPIC_IN [TOPIC_OUT]`
 *   - `TOPIC_IN`: The topic to subscribe. It is resolved against parent namespace of the node(let) (as opposed to the
 *                 `~input` topic which is resolved against the private node(let) namespace).
 *   - `TOPIC_OUT`: The topic to publish. It is resolved against parent namespace of the node(let) (as opposed to the
 *                  `~output` topic which is resolved against the private node(let) namespace). If not specified, output
 *                  topic will be `${TOPIC_IN}_relay`.
 */
class RelayNodelet : public ::cras::Nodelet
{
protected:
  //! \brief The lazy pair of subscriber and publisher.
  ::std::unique_ptr<::cras::GenericLazyPubSub> pubSub;

  void onInit() override;

  /**
   * \brief Republish the received message.
   * \param[in] event The event with the incoming message.
   * \param[in] pub The publisher to use for publishing the output messages.
   */
  virtual void processMessage(
    const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event, ::ros::Publisher& pub);
};

}
