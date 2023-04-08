#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a relay nodelet that can modify headers.
 * \note It can process the messages on a single topic in parallel allowing for maximum throughput.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <utility>

#include <ros/duration.h>
#include <ros/message_event.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>

namespace cras
{

/**
 * \brief Nodelet for relaying messages and changing their header.
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
 * - `~frame_id_prefix` (string, no default): If set, frame_id will be prefixed with this string.
 * - `~frame_id_suffix` (string, no default): If set, frame_id will be suffixed with this string.
 * - `~frame_id` (string, no default): If set, frame_id will be replaced by this string.
 * - `~frame_id_replace_start` (string "from|to", no default): If set and frame_id starts with `from`, it will be
 *                                                             replaced with `to`.
 * - `~frame_id_replace_end` (string "from|to", no default): If set and frame_id ends with `from`, it will be replaced
 *                                                           with `to`.
 * - `~frame_id_replace` (string "from|to", no default): If set, all occurrences of `from` in frame_id will be replaced
 *                                                       with `to`.
 * - `~stamp_relative` (double, no default): If set, the given duration will be added to the message's stamp. If the
 *                                           stamp would under/overflow, ros::TIME_MIN or ros::TIME_MAX will be set.
 * - `~stamp` (double, no default): If set, the message's stamp will be set to this time (`stamp_relative` will not
 *                                  apply if set).
 * - `~stamp_ros_time (bool, default false): If true, message stamp will be changed to current ROS time.
 *                                           `stamp_relative` can be used in combination with this option.
 * - `~stamp_wall_time (bool, default false): If true, message stamp will be changed to current wall time.
 *                                            `stamp_relative` can be used in combination with this option.
 *
 * Subscribed topics:
 * - `~input` (any type with header): The input messages. If `lazy` is true, it will only be subscribed when `~output`
 *                                    has some subscribers.
 *                                    If you subscribe to a message type without header, the node should not crash, but
 *                                    it will discard all received messages and print an error message.
 *
 * Published topics:
 * - `~output` (same type as `~input`): The relayed output messages with changed header.
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
 *
 * \note A copy of the received message will be made in any case to allow modification of the header. If you set
 *       frame_id to a longer string, another copy of the message will be made to make space for the prolonged ID.
 */
class ChangeHeaderNodelet : public ::cras::Nodelet
{
protected:
  //! \brief The lazy pair of subscriber and publisher.
  ::std::unique_ptr<::cras::GenericLazyPubSub> pubSub;

  void onInit() override;

  /**
   * \brief Change the header of the incoming message and publish the result.
   * \param [in] event Event containing the received message.
   * \param [in] pub The publisher to use for publishing.
   */
  virtual void processMessage(
    const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event, ::ros::Publisher& pub);

  //! \brief Replace the whole frame_id with this value.
  ::cras::optional<::std::string> newFrameId;

  //! \brief Prefix frame_id with this value.
  ::cras::optional<::std::string> newFrameIdPrefix;

  //! \brief Suffix frame_id with this value.
  ::cras::optional<::std::string> newFrameIdSuffix;

  //! \brief Replace all occurrences of first string with the second one in frame_id.
  ::cras::optional<::std::pair<::std::string, ::std::string>> newFrameIdReplace;

  //! \brief If frame_id starts with the first string, replace it with the second one.
  ::cras::optional<::std::pair<::std::string, ::std::string>> newFrameIdReplaceStart;

  //! \brief If frame_id ends with the first string, replace it with the second one.
  ::cras::optional<::std::pair<::std::string, ::std::string>> newFrameIdReplaceEnd;

  //! \brief Replace stamp with the given value.
  ::cras::optional<::ros::Time> newStampAbs;

  //! \brief Add this value to stamp. In case of under/overflow, TIME_MIN or TIME_MAX are set.
  ::cras::optional<::ros::Duration> newStampRel;

  //! \brief Change stamp to current ROS time.
  bool newStampRosTime {false};

  //! \brief Change stamp to current wall time.
  bool newStampWallTime {false};

  //! \brief Whether the subscribed topic has a header field. This is filled on receipt of the first message.
  ::cras::optional<bool> hasHeader {::cras::nullopt};
};

}
