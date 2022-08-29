#pragma once

/**
 * \file
 * \brief This is a relay nodelet that can modify headers. It can process the messages on a single topic in parallel
 *        allowing for maximum throughput.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>
#include <utility>

#include <ros/duration.h>
#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

/**
 * \brief Parameters for header-changing nodelet.
 */
struct ChangeHeaderParams
{
  //! \brief Replace the whole frame_id with this value.
  ::cras::optional<::std::string> newFrameId;

  //! \brief Prefix frame_id with this value.
  ::cras::optional<::std::string> newFrameIdPrefix;

  //! \brief Suffix frame_id with this value.
  ::cras::optional<::std::string> newFrameIdSuffix;

  //! \brief Replace all occurences of first string with the second one in frame_id.
  ::cras::optional<::std::pair<::std::string, ::std::string>> newFrameIdReplace;

  //! \brief If frame_id starts with the first string, replace it with the second one.
  ::cras::optional<::std::pair<::std::string, ::std::string>> newFrameIdReplaceStart;

  //! \brief If frame_id ends with the first string, replace it with the second one.
  ::cras::optional<::std::pair<::std::string, ::std::string>> newFrameIdReplaceEnd;

  //! \brief Replace stamp with the given value.
  ::cras::optional<::ros::Time> newStampAbs;

  //! \brief Add this value to stamp. In case of under/overflow, TIME_MIN or TIME_MAX are set.
  ::cras::optional<::ros::Duration> newStampRel;
};

/**
 * \brief (Possibly lazy) publisher and subscriber pair that changes header of the received messages.
 * \tparam SubscriberType Type of the lazy-created subscriber.
 */
template <typename SubscriberType = ::ros::Subscriber>
class ChangeHeaderPubSub : public ::cras::GenericLazyPubSub<SubscriberType>
{
public:
  /**
   * \brief Create the lazy pub-sub object.
   * \param[in] params Parameters of the header changer.
   * \param[in] topicIn Input topic.
   * \param[in] topicOut Output topic
   * \param[in] nh Nodehandle used for subscriber and publisher creation.
   * \param[in] inQueueSize Queue size for the subscriber.
   * \param[in] outQueueSize Queue size for the publisher.
   * \param[in] logHelper Log helper.
   */
  ChangeHeaderPubSub(::cras::ChangeHeaderParams params,
    const ::std::string& topicIn, const ::std::string& topicOut, const ::ros::NodeHandle& nh = {},
    size_t inQueueSize = 10, size_t outQueueSize = 10,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>()) :
      GenericLazyPubSub<SubscriberType>(topicIn, topicOut, nh, inQueueSize, outQueueSize, logHelper),
        params(::std::move(params))
  {
  }

  ~ChangeHeaderPubSub() = default;

protected:
  virtual void processMessage(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event)
  {
    if (!this->advertiseOptions->has_header)
    {
      this->logHelper->logErrorOnce("Running change_header on message type %s which does not have a header! "
        "Ignoring the message.", this->advertiseOptions->datatype.c_str());
      return;
    }

    const auto& msg = event.getConstMessage();
    auto header = ::cras::getHeader(*msg);
    if (!header.has_value())
    {
      this->logHelper->logError("Change_header failed to extract a header from the message of type %s! "
        "Ignoring the message.", this->advertiseOptions->datatype.c_str());
      return;
    }
    const auto origHeader = header.value();

    if (this->params.newFrameIdReplaceStart.has_value())
    {
      const auto fromTo = this->params.newFrameIdReplaceStart.value();
      ::cras::replace(header->frame_id, fromTo.first, fromTo.second, ::cras::ReplacePosition::START);
    }

    if (this->params.newFrameIdReplaceEnd.has_value())
    {
      const auto fromTo = this->params.newFrameIdReplaceEnd.value();
      ::cras::replace(header->frame_id, fromTo.first, fromTo.second, ::cras::ReplacePosition::END);
    }

    if (this->params.newFrameIdReplace.has_value())
    {
      const auto fromTo = this->params.newFrameIdReplace.value();
      ::cras::replace(header->frame_id, fromTo.first, fromTo.second);
    }

    if (this->params.newFrameIdPrefix.has_value())
      header->frame_id = this->params.newFrameIdPrefix.value() + header->frame_id;

    if (this->params.newFrameIdSuffix.has_value())
      header->frame_id += this->params.newFrameIdSuffix.value();

    if (this->params.newFrameId.has_value())
      header->frame_id = this->params.newFrameId.value();

    if (this->params.newStampRel.has_value())
    {
      // Correctly handle overflow cases
      try
      {
        header->stamp += this->params.newStampRel.value();
      }
      catch (const ::std::runtime_error&)
      {
        header->stamp = this->params.newStampRel.value().toSec() > 0 ? ::ros::TIME_MAX : ::ros::TIME_MIN;
      }
    }

    if (this->params.newStampAbs.has_value())
      header->stamp = this->params.newStampAbs.value();

    if (header.value() == origHeader)
    {
      this->pub.template publish(msg);
      return;
    }

    ::topic_tools::ShapeShifter newMsg;
    // cannot use newMsg = *msg in Melodic, that would segfault
    ::cras::copyShapeShifter(*msg, newMsg);
    if (!::cras::setHeader(newMsg, header.value()))
    {
      this->logHelper->logError("Change_header failed to modify the header of the message of type %s! "
        "Ignoring the message.", this->advertiseOptions->datatype.c_str());
      return;
    }

    this->pub.template publish(newMsg);
  }

  //! \brief Parameters.
  ::cras::ChangeHeaderParams params;
};

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
 * - `~frame_id_prefix` (string, no default): If set, frame_id will be prefixed with this string.
 * - `~frame_id_suffix` (string, no default): If set, frame_id will be suffixed with this string.
 * - `~frame_id` (string, no default): If set, frame_id will be replaced by this string.
 * - `~frame_id_replace_start` (string "from|to", no default): If set and frame_id starts with `from`, it will be
 *                                                             replaced with `to`.
 * - `~frame_id_replace_end` (string "from|to", no default): If set and frame_id ends with `from`, it will be replaced
 *                                                           with `to`.
 * - `~frame_id_replace` (string "from|to", no default): If set, all occurences of `from` in frame_id will be replaced
 *                                                       with `to`.
 * - `~stamp_relative` (double, no default): If set, the given duration will be added to the message's stamp. If the
 *                                           stamp would under/overflow, ros::TIME_MIN or ros::TIME_MAX will be set.
 * - `~stamp` (double, no default): If set, the message's stamp will be set to this time.
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
  ::std::unique_ptr<::cras::ChangeHeaderPubSub<>> pubSub;

  void onInit() override;
};

}
