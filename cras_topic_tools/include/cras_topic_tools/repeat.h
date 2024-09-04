#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a simple implementation of a repeater nodelet.
 * \author Martin Pecka
 */

#include <memory>
#include <mutex>
#include <string>

#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/message_event.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>

namespace cras
{

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
 * - `~tcp_no_delay` (bool, default False): If True, the `TCP_NODELAY` flag is set for the subscriber. This should
 *                                         decrease the latency of small messages, but might give suboptimal
 *                                         transmission speed for large messages.
 * - `~max_age` (double, no default): If set, this parameter instructs the repeater to check how old the re-published
 *                                    message would be. If it is older than `ros::Time::now() - max_age`, it will no
 *                                    longer be published. This setting requires that the type of the message contains a
 *                                    Header field at the beginning. Setting it on header-less topics can cause
 *                                    unexpected behavior and random rejection of messages.
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
 *                                                   they are also published right away as they come on the input topic.
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
 *       than `rate` parameter, no message will ever be published by this repeater (the timer will get reset earlier
 *       than it can ever fire).
 */
class RepeatMessagesNodelet : public ::cras::Nodelet
{
protected:
  //! \brief The lazy pair of subscriber and publisher.
  ::std::unique_ptr<::cras::GenericLazyPubSub> pubSub;

  //! \brief Subscriber to the reset topic.
  ::ros::Subscriber resetSub;

  void onInit() override;

  /**
   * \brief Called when the repeater should be reset. The incoming message can be of any type and should not be
   *        examined.
   */
  virtual void onReset(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&);

  /**
   * \brief Reset the repeater, e.g. after a time jump.
   */
  void reset() override;

  /**
   * \brief Record the incoming message if it passes validations, and publish it if `publishOnlyOnTimer` is false.
   * \param[in] event The incoming message event.
   * \param[in] pub The publisher to be used for publishing the repeated messages.
   * \note It is safe to call this function from multiple threads (access to the stored message is mutex-protected).
   */
  void processMessage(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event, ::ros::Publisher& pub);

  /**
   * \brief Publish the last stored message (if it is eligible).
   */
  virtual void maybePublish();

  /**
   * \brief Whether Header should be extracted from the message and its time stamp should undergo inspection.
   * \return Whether to extract the stamp.
   */
  virtual bool inspectStamps() const;

  /**
   * \brief Timer callback. Publish the last stored message if it is eligible.
   */
  void everyPeriod(const ::ros::TimerEvent&);

  //! \brief The desired output rate.
  ::std::unique_ptr<::ros::Rate> rate;

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

  //! \brief Mutex protecting msg, lastMsgStamp and numRepeats.
  ::std::mutex msgMutex;

  //! \brief The last stored message (null if no message has been received yet).
  ::topic_tools::ShapeShifter::ConstPtr msg;

  //! \brief Time stamp of the last stored message (only filled if `inspectStamps()` returns true.
  ::cras::optional<::ros::Time> lastMsgStamp;

  //! \brief Number of times the last stored message has been repeated.
  size_t numRepeats {0};

  ::cras::optional<bool> hasHeader {::cras::nullopt};

  //! \brief The timer periodically republishing the last stored message.
  ::ros::Timer timer;

  //! \brief The publisher of the repeated messages.
  ::ros::Publisher pub;
};

}
