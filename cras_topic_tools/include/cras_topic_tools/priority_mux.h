#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Priority-based muxer nodelet for topics.
 * \author Martin Pecka
 */

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <std_msgs/Bool.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_topic_tools/priority_mux_base.h>
#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

/**
 * \brief Priority-based muxer nodelet for topics.
 * 
 * It guards access to one or more output topics for which there are several input topics fighting. To determine which
 * input topic "wins" its desired output access, priorities are assigned to each input topic, and each input topic
 * specifies a validity period for which it is considered. This validity period should be renewed every time a message
 * is received on the topic. This means that when messages stop coming for a topic, its priority is given up. Each input
 * topic can also be explicitly released.
 * 
 * When more output topics are specified, the priority is still shared. This means that receiving a high-priority
 * message for one output topic also sets all other output topics to this higher priority. This effectively allows e.g.
 * managing access to robot motors in case there are several topics that can control them via different approaches.
 * 
 * The mux can also process binary lock messages. There are two types of them: permanent and time-based. Both act via
 * locking their assigned priority, which means no messages with lower priorities will qualify to get to their output
 * topics. Permanent locks just lock or unlock their priority. Time-based lock messages are expected to come regularly
 * and when they stop coming for some time, their priority is locked. They can also carry explicit locking commands.
 *
 * ROS parameters:
 * - `~queue_size` (uint, default 10): Queue size for all subscribers and publishers.
 * - `~tcp_no_delay` (bool, default False): If True, the `TCP_NODELAY` flag is set for all subscribers. This should
 *                                          decrease the latency of small messages, but might give suboptimal
 *                                          transmission speed for large messages.
 * - `~default_out_topic` (str, default "mux_out"): Default output topic to use when none is explicitly specified for a
 *                                                  topic configuration.
 * - `~none_topic` (str, default "__none"): Virtual name of a topic reported as selected when no priority is active.
 * - `~none_priority` (int, default 0): Priority level signalling that no priority is active.
 * - `~subscriber_connect_delay` (double, default 0.1 s): Time to wait before a newly created non-latched output topic
 *                                                        publisher will publish its first message.
 * - `~topics` (list or dict): Definition of subscribed topics and their priorities. If dict, only values are used.
 *   - `topic` (str): The input topic to subscribe.
 *   - `priority` (int): Priority of the input topic.
 *   - `out_topic` (str, default `default_out_topic`): The topic to which the input should be relayed if its priority
 *                                                     is active.
 *   - `timeout` (double): How long does a message on this topic hold its priority.
 *   - `name` (str, default `topic`): Human-readable description of the input topic.
 *   - `disable_topic` (str, default ""): If nonempty, this topic will be subscribed to allow disabling the input topic.
 *   - `disable_topic_inverted` (bool, default false): If true, `topic` will get disabled when false is received on
 *                                                     `disable_topic`. If false, `topic` will get disabled when true is
 *                                                     received on `disable_topic`. In case `disable_topic` is not of
 *                                                     `std_msgs/Bool` type, this parameter does nothing.
 *   - `before_disable_message` (str, optional): If set, this string should point to a bagfile containing a single
 *                                               message. This message will be published on this topic just before the
 *                                               topic becomes disabled.
 *   - `queue_size` (uint, default `~queue_size`): Queue size for the subscriber.
 * - `~locks` (list or dict): Definition of lock topics and their priorities. If dict, only values are used.
 *   - `topic` (str): The lock topic to subscribe.
 *   - `priority` (int): Priority of the lock topic.
 *   - `timeout` (double): If zero, the lock only uses data in the sent Bool messages to determine whether it is locked
 *                         or not. If non-zero, the lock will get locked when the last message on the lock topic is
 *                         older than the timeout.
 *   - `name` (str, default `topic`): Human-readable description of the lock topic.
 * - `~out_topics` (list or dict): Optional configuration of output topics. If dict, only values are used.
 *   - `topic` (str): The output topic.
 *   - `force_latch` (bool, optional): If set, forces the topic's latching status to the value. Otherwise, latching
 *                                     is configured based on latching of the first received message.
 *   - `subscriber_connect_delay` (double, default `~subscriber_connect_delay`): Time to wait before publishing the
 *                                                                               first message after the publisher is
 *                                                                               created. Only applies to non-latched
 *                                                                               topics.
 *   - `num_subscribers_to_wait` (uint, default 0): If non-zero, when the publisher is created, it will wait until it
 *                                                  has this number of subscribers before it publishes its first
 *                                                  message. This is more reliable than setting the delay if you know
 *                                                  the number of subscribers. The number is not the actual number of
 *                                                  subscriber objects, but rather the number of subscribing separate
 *                                                  nodes (so e.g. all subscriptions from a single nodelet manager count
 *                                                  as one).
 *   - `queue_size` (uint, default `~queue_size`): Queue size for the publisher.
 *
 * Subscribed topics:
 * - Topics specified in `~topics/topic` (any type): Input messages.
 * - Topics specified in `~topics/disable_topic` (any type): Disable messages. If the topic type is `std_msgs/Bool`,
 *                                                           the associated input topic is enabled/disabled according
 *                                                           to the value of the message (true means disable).
 *                                                           If the topic is of any other type, receiving a message on
 *                                                           this topic disables the associated input topic for the
 *                                                           `timeout` specified for the input topic.
 * - Topics specified in `~locks/topic` (`std_msgs/Bool`): Lock messages.
 * - `~reset` (any type): When a message is received on this topic, the mux is reset.
 * 
 * Published topics:
 * - Topics specified in `~topics/out_topic` (any type): The muxed output messages.
 * - `~active_priority` (`std_msgs/Int32`): The currently active priority.
 * - `~selected/${out_topic}` (`std_msgs/String`): For each output topic, this topic tells which input is currently
 *                                                 relayed to the output.
 */
class PriorityMuxNodelet : public ::cras::Nodelet
{
protected:
  void onInit() override;

  /**
   * \brief Callback method triggered when a message is received on the input topic.
   * \param[in] inTopic The topic the message was received on.
   * \param[in] event The received message ShapeShifter.
   */
  virtual void cb(const ::std::string& inTopic, const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);

  /**
   * \brief Callback method triggered when a lock message is received.
   * \param[in] topic The topic the lock message was received on.
   * \param[in] event The received lock message.
   */
  virtual void lockCb(const ::std::string& topic, const ::ros::MessageEvent<::std_msgs::Bool const>& event);

  /**
   * \brief Callback method triggered when a disable message is received.
   * \param[in] inTopic The input topic for which the disable message was received on.
   * \param[in] invert If true, the topic will get disabled when false is received on the disable topic. If false,
   *                   the topic will get disabled when true is received on the disable topic. In case the disable
   *                   topic is not of std_msgs/Bool type, this parameter does nothing.
   * \param[in] event The received message ShapeShifter. If the message is of `std_msgs::Bool` type, it gets special
   *                  handling (the `data` field of the message tells whether the topic should be disabled or enabled).
   */
  virtual void disableCb(const ::std::string& inTopic,
                         bool invert,
                         const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);

  /**
   * \brief Callback method triggered when a reset message is received.
   * \param[in] event The received message ShapeShifter. It can be anything, this method is only interested in the fact
   *                  that some message was received.
   */
  virtual void resetCb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);

  /**
   * \brief Check what changed from the last `publishChanges()` call and if there are changes, publish them (active
   *        priority, selected output topics etc.).
   */
  void publishChanges();

  /**
   * \brief Method triggered on timer timeout. This should update the mux and check for changes.
   * \param[in] name The name of the timer.
   * \param[in] event The timer event.
   */
  void onTimeout(const ::std::string& name, const ::ros::TimerEvent&);

  /**
   * \brief Set a timer that will call `onTimeout()` after the specified time.
   * \param[in] name The name of the timer.
   * \param[in] timeout The duration of the timer.
   */
  void setTimer(const ::std::string& name, const ::ros::Duration& timeout);

  /**
   * \brief Reset the mux to its initial state.
   */
  void reset();

  //! \brief Mux instance.
  ::std::unique_ptr<::cras::PriorityMux> mux;

  //! \brief List of subscribers. Just for keeping them alive.
  ::std::list<::ros::Subscriber> subscribers;

  //! \brief Map of output topic and the associated publisher.
  ::std::unordered_map<::std::string, ::ros::Publisher> publishers;

  //! \brief Publisher for active priority.
  ::ros::Publisher activePriorityPub;

  //! \brief Map of publishers of the topics announcing currently selected publishers.
  ::std::unordered_map<::std::string, ::ros::Publisher> selectedPublishers;

  //! \brief Subscriber for reset topic.
  ::ros::Subscriber resetSub;

  //! \brief Map of timer names and the timers.
  ::std::unordered_map<::std::string, ::ros::Timer> timers;

  //! \brief Configurations of input topics.
  ::std::unordered_map<::std::string, ::cras::priority_mux::TopicConfig> topicConfigs;

  //! \brief Configurations of lock topics.
  ::std::unordered_map<::std::string, ::cras::priority_mux::LockConfig> lockConfigs;

  //! \brief Configurations of output topics.
  std::unordered_map<std::string, ::cras::priority_mux::OutputTopicConfig> outTopicConfigs;

  //! \brief All output topic names.
  ::std::unordered_set<::std::string> outTopics;

  //! \brief The active priority after the last `publishChanges()` call.
  ::cras::optional<int> lastActivePriority;

  //! \brief The selected output topics after the last `publishChanges()` call.
  ::std::unordered_map<::std::string, ::std::string> lastSelectedTopics;

  //! \brief The messages to be injected into the mux right before a topic is disabled.
  ::std::unordered_map<::std::string, ::ros::MessageEvent<::cras::ShapeShifter>> beforeDisableMessages;

  //! \brief Cached list of "before disable" messages that have a Header.
  ::std::unordered_set<::std::string> beforeDisableMessagesWithHeader;

  //! \brief Queue size of all publishers and subscribers.
  size_t queueSize {10u};

  //! \brief Whether `tcpNoDelay()` flag should be set for all subscribers.
  bool tcpNoDelay {false};
};

}
