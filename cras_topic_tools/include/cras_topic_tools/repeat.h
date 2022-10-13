#pragma once

/**
 * \file
 * \brief This is a simple implementation of a repeater nodelet.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

/**
 * \brief Parameters of the repeater.
 */
struct RepeatMessagesParams
{
  //! \brief The desired output rate.
  ::ros::Rate rate;

  //! \brief Maximum number of repetitions of a single message (only limited when set).
  ::cras::optional<size_t> maxRepeats {::cras::nullopt};

  //! \brief Maximum age of a message to allow publishing it (the message has to have a Header).
  ::cras::optional<::ros::Duration> maxAge {::cras::nullopt};

  //! \brief Whether to discard an incoming message if its stamp is older than the previously accepted message.
  bool discardOlderMessages {false};

  //! \brief Whether to reset the publication timer when a new message arrives.
  bool resetOnMsg {true};

  //! \brief Whether to publish only on timer events, or also when an incoming message is received.
  bool publishOnlyOnTimer {false};
};

/**
 * \brief (Possibly lazy) publisher and subscriber pair that repeats received messages at the given rate. The last
 *        received message is repeated until a newer message arrives. If the input rate is higher than the desired,
 *        the messages are published on the input rate (depending on configuration).
 * \tparam SubscriberType Type of the lazy-created subscriber.
 */
template <typename SubscriberType = ::ros::Subscriber>
class RepeatMessagesPubSub : public ::cras::GenericLazyPubSub<SubscriberType>
{
public:
  /**
   * \brief Create the lazy pub-sub object.
   * \param[in] params Parameters of the repeater.
   * \param[in] topicIn Input topic.
   * \param[in] topicOut Output topic
   * \param[in] nh Nodehandle used for subscriber and publisher creation.
   * \param[in] inQueueSize Queue size for the subscriber.
   * \param[in] outQueueSize Queue size for the publisher.
   * \param[in] logHelper Log helper.
   */
  RepeatMessagesPubSub(::cras::RepeatMessagesParams params,
    const ::std::string& topicIn, const ::std::string& topicOut, const ::ros::NodeHandle& nh = {},
    size_t inQueueSize = 10, size_t outQueueSize = 10,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>()) :
      GenericLazyPubSub<SubscriberType>(topicIn, topicOut, nh, inQueueSize, outQueueSize, logHelper),
        params(::std::move(params))
  {
    this->timer = nh.createTimer(this->params.rate.expectedCycleTime(), &RepeatMessagesPubSub::everyPeriod, this);
  }

  ~RepeatMessagesPubSub() override = default;

  /**
   * \brief Reset the repeater, e.g. after a time jump.
   */
  void reset()
  {
    this->timer.setPeriod(this->params.rate.expectedCycleTime(), true);

    ::std::lock_guard<::std::mutex> lock(this->msgMutex);
    this->msg.reset();
    this->numRepeats = 0;
    this->lastMsgStamp.reset();
  }

protected:
  /**
   * \brief Record the incoming message if it passes validations, and publish it if `publishOnlyOnTimer` is false.
   * \param[in] event The incoming message event.
   * \note It is safe to call this function from multiple threads (access to the stored message is mutex-protected).
   */
  void processMessage(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event) override
  {
    ::cras::optional<::ros::Time> stamp;
    // If inspecting time stamps is required, deserialize the Header and validate the time stamp.
    if (this->inspectStamps())
    {
      // This is potentially unsafe if the subscribed message actually does not have a Header field at the beginning.
      const auto header = ::cras::getHeader(*event.getConstMessage());
      if (header.has_value())
      {
        stamp = header->stamp;
        if (this->params.maxAge.has_value() && (stamp.value() + this->params.maxAge.value()) < ::ros::Time::now())
        {
          this->logHelper->logInfoThrottle(5.0, "Received message too old (%.3g s > %.3g s) will be discarded.",
            (::ros::Time::now() - stamp.value()).toSec(), this->params.maxAge.value().toSec());
          return;
        }
        if (this->params.discardOlderMessages && this->lastMsgStamp.has_value()
          && stamp.value() < this->lastMsgStamp.value())
        {
          this->logHelper->logInfoThrottle(
            5.0, "Received message is %.3g s older than current message, it will be discarded.",
            (this->lastMsgStamp.value() - stamp.value()).toSec());
          return;
        }
      }
    }

    // Record the incoming message.
    {
      ::std::lock_guard<::std::mutex> lock(this->msgMutex);
      this->msg = event.getConstMessage();
      this->numRepeats = 0;
      this->lastMsgStamp = stamp;
    }

    // Republish the message right away if the configuration says so.
    if (!this->params.publishOnlyOnTimer)
      this->maybePublish();

    // If resetOnMsg, we reset the publication timer to start counting from zero.
    if (this->params.resetOnMsg)
      this->timer.setPeriod(this->params.rate.expectedCycleTime(), true);
  }

  /**
   * \brief Publish the last stored message (if it is eligible).
   */
  virtual void maybePublish()
  {
    if (!this->pub || this->msg == nullptr)
      return;

    ::std::lock_guard<::std::mutex> lock(this->msgMutex);

    if (this->params.maxRepeats.has_value() && this->numRepeats > this->params.maxRepeats.value())
    {
      this->logHelper->logWarnThrottle(5.0, "Message already republished %i times.", this->numRepeats);
      return;
    }

    if (this->inspectStamps() && this->params.maxAge.has_value() && this->lastMsgStamp.has_value() &&
        (this->lastMsgStamp.value() + this->params.maxAge.value()) < ::ros::Time::now())
    {
      this->logHelper->logWarnThrottle(5.0, "Message too old (%.3g s > %.3g s) will not be republished.",
        (::ros::Time::now() - this->lastMsgStamp.value()).toSec(), this->params.maxAge.value().toSec());
      return;
    }

    this->numRepeats += 1;
    this->pub.template publish(this->msg);
  }

  /**
   * \brief Whether Header should be extracted from the message and its time stamp should undergo inspection.
   * \return Whether to extract the stamp.
   */
  virtual bool inspectStamps() const
  {
    if (!this->advertiseOptions.has_value() || !this->advertiseOptions->has_header)
      return false;
    return this->params.maxAge.has_value() || this->params.discardOlderMessages;
  }

  /**
   * \brief Timer callback. Publish the last stored message if it is eligible.
   */
  void everyPeriod(const ::ros::TimerEvent&)
  {
    this->maybePublish();
  }

  //! \brief Parameters of the repeater.
  ::cras::RepeatMessagesParams params;

  //! \brief Mutex protecting msg, lastMsgStamp and numRepeats.
  ::std::mutex msgMutex;

  //! \brief The last stored message (null if no message has been received yet).
  ::topic_tools::ShapeShifter::ConstPtr msg;

  //! \brief Time stamp of the last stored message (only filled if `inspectStamps()` returns true.
  ::cras::optional<::ros::Time> lastMsgStamp;

  //! \brief Number of times the last stored message has been repeated.
  size_t numRepeats {0};

  //! \brief The timer periodically republishing the last stored message.
  ::ros::Timer timer;
};

/**
 * \brief Nodelet for repeating messages coming at a slower rate (or even just a single message).
 *
 * ROS parameters:
 * - `~in_queue_size` (uint, default 10): Queue size for the subscriber.
 * - `~out_queue_size` (uint, default $in_queue_size): Queue size for the publisher.
 * - `~rate` (Hz, positive double, default 1.0): The desired rate of the published messages.
 * - `~lazy` (bool, default False): Whether to shut down the subscriber when the publisher has no subscribers.
 *                                  The `~input` topic will be subscribed in the beginning, and will unsubscribe
 *                                  automatically after the first message is received (this is needed to determine the
 *                                  full type of the topic to publish). This can have unexpected consequences as the
 *                                  nodelet doesn't listen for new messages that came in when the publisher had no
 *                                  subscribers. So when the nodelet should start publishing again, it will repeat the
 *                                  last message it received before unsubscribing.
 * - `~max_age` (double, no default): If set, this parameter instructs the repeater to check how old the re-published
 *                                    message would be. If it is older than ros::Time::now(), it will no longer be
 *                                    published. This setting requires that the type of the message contains a Header
 *                                    field at the beginning. Setting it on header-less topics can cause unexpected
 *                                    behavior and random rejection of messages.
 * - `~max_repeats` (uint, no default): If set, this parameter tells how many times a single message can be repeated.
 * - `~discard_older_messages` (bool, default false): If true, incoming messages' time stamp will be compared to the
 *                                                    last stored message, and if the incoming message is older than the
 *                                                    stored one, it will be discarded. This setting requires that the
 *                                                    type of the message contains a Header field at the beginning.
 *                                                    Setting it on header-less topics can cause unexpected behavior
 *                                                    and random rejection of messages.
 * - `~reset_on_msg` (bool, default true): Whether the internal repeating timer should be reset to count from 0 when an
 *                                         incoming message is stored.
 * - `~publish_only_on_timer` (bool, default false): If true, messages are only published on timer events. If false,
 *                                                   they are also published right away as the come on the input topic.
 *
 * Subscribed topics:
 * - `~input` (any type): The input messages. If `lazy` is true, it will only be subscribed when `~output` has some
 *                        subscribers.
 * - `~reset` (any type): When a message is received on this topic, the repeater is reset into its initial state
 *                        (i.e. the last received message is discarded). This should happen every time the ROS time
 *                        jumps, e.g. when simulation is reset or a bag file starts playing again.
 *
 * Published topics:
 * - `~output` (same type as `~input`): The output messages.
 *
 * Command-line arguments:
 * This nodelet (or node) can also be called with some arguments passed on command line. This means you can
 * pass CLI arguments specifying the topics to subscribe/publish and the rate.
 * - `rosrun cras_topic_tools repeat TOPIC_IN RATE [TOPIC_OUT]`
 *   - `TOPIC_IN`: The topic to subscribe. It is resolved against parent namespace of the node(let) (as opposed to the
 *                 `~input` topic which is resolved against the private node(let) namespace).
 *   - `RATE`: The output rate. If both this argument and `~rate` parameter are specified, the ROS parameter wins.
 *   - `TOPIC_OUT`: The topic to publish. It is resolved against parent namespace of the node(let) (as opposed to the
 *                  `~output` topic which is resolved against the private node(let) namespace). If not specified, output
 *                  topic will be `${TOPIC_IN}_repeat`.
 *
 * \note If `reset_on_msg` and `publish_only_on_timer` are both true and the rate of the incoming messages is higher
 *       than `rate` parameter, no message will ever be published by this repeated (the timer will get reset earlier
 *       than it can ever fire).
 */
class RepeatMessagesNodelet : public ::cras::Nodelet
{
protected:
  //! \brief The lazy pair of subscriber and publisher.
  ::std::unique_ptr<::cras::RepeatMessagesPubSub<>> pubSub;

  //! \brief Subscriber to the reset topic.
  ::ros::Subscriber resetSub;

  void onInit() override;

  /**
   * \brief Called when the repeater should be reset. The incoming message can be of any type and should not be
   *        examined.
   */
  virtual void onReset(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&);
};

}
