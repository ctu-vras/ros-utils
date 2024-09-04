#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Priority-based muxer for topics.
 * \author Martin Pecka
 */

#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/log_utils.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace cras
{

namespace priority_mux
{

/** 
 * \brief Configuration of an input to the mux.
 */
struct TopicConfig
{
  ::std::string name;  //!< The human-readable name of the config.
  ::std::string inTopic;  //!< The input topic.
  ::std::string outTopic;  //!< The output topic.
  int priority {0};  //!< The priority for the topic. Usually is a positive number.
  ::ros::Duration timeout;  //!< How long do messages on this topic keep the priority active.
  size_t queueSize;  //!< Queue size for the subscriber.
};

/** 
 * \brief Configuration of a lock topic for the mux.
 */
struct LockConfig
{
  ::std::string name;  //!< The human-readable name of the config.
  ::std::string topic;  //!< The topic associated with the lock.
  int priority {0};  //!< The priority for the lock. Usually is a positive number.
  ::ros::Duration timeout;  //!< If zero, the lock only locks/unlocks based on the `locked` parameter of the lock
                            //!< callback. If non-zero, it is the time after which the lock will get locked, unless a
                            //!< message comes to the lock topic with `locked` set to `false`.
};

/**
 * \brief Configuration of an output topic for the mux.
 */
struct OutputTopicConfig
{
  ::std::string topic;  //!< The output topic.
  ::cras::optional<bool> forceLatch;  //!< If set, forces the topic's latching status to the value. Otherwise, latching
                                      //!< is configured based on latching of the first received message.
  ::ros::WallDuration subscriberConnectDelay;  //!< Time to wait before publishing the first message after the publisher
                                               //!< is created. Only applies to non-latched topics.
  size_t numSubscribersToWait {0u};  //!< If non-zero, when the publisher is created, it will wait until it has this
                                     //!< number of subscribers before it publishes its first message. This is more
                                     //!< reliable than setting the delay if you know the number of subscribers. The
                                     //!< number is not the actual number of subscriber objects, but rather the number
                                     //!< of subscribing separate nodes (so e.g. all subscriptions from a single nodelet
                                     //!< manager count as one).
  size_t queueSize;  //!< Queue size for the publisher.
};
}

/**
 * \brief A class for priority-based muxing of topics.
 * 
 * This class provides the functionality for priority-based muxing of topics. It allows to control access to one or more
 * output topics which several input topics are aiming for. Priorities are assigned to each input topic to decide which
 * one gets access to its desired output. A validity period is also assigned to each input topic which needs to be
 * renewed whenever a message is received from the topic. An input topic can be disabled explicitly via a topic.
 */
class PriorityMux : public ::cras::HasLogger
{
public:
  /**
   * \brief The function to call when the mux needs to schedule a call to `update()` after some time.
   *
   * \param[in] name Name of the timer (subsequent calls with the same name will cancel the preceding requests).
   * \param[in] timeout The time after which `update()` should be called.
   */
  typedef ::std::function<void(const ::std::string& name, const ::ros::Duration& timeout)> SetTimerFn;

  /**
   * \brief Constructs a PriorityMux object.
   *
   * \param[in] topicConfigs Configurations of input topics.
   * \param[in] lockConfigs Configurations of lock topics.
   * \param[in] setTimerFn The function to call when the mux needs to schedule a call to `update()` after some time.
   * \param[in] now Current time.
   * \param[in] log The CRAS logger to use for printing log messages.
   * \param[in] noneTopic Virtual name of a topic reported as selected when no priority is active.
   * \param[in] nonePriority Priority level signalling that no priority is active.
   */
  PriorityMux(const ::std::unordered_map<::std::string, ::cras::priority_mux::TopicConfig>& topicConfigs,
              const ::std::unordered_map<::std::string, ::cras::priority_mux::LockConfig>& lockConfigs,
              const SetTimerFn& setTimerFn, const ::ros::Time& now, const ::cras::LogHelperPtr& log,
              const ::std::string& noneTopic = "__none", int nonePriority = 0);

  virtual ~PriorityMux();

  /**
   * \brief Callback function run when input topic message is received.
   *
   * \param[in] inTopic Name of the input topic.
   * \param[in] stamp ROS time stamp of the message.
   * \param[in] now Current ROS time.
   * \return True if the input message should be relayed to its output topic.
   */
  virtual bool cb(const ::std::string& inTopic, const ::ros::Time& stamp, const ::ros::Time& now);

  /**
   * \brief Callback function to disable/enable an input topic.
   *
   * \param[in] inTopic Name of the input topic to disable/enable.
   * \param[in] stamp ROS time stamp of the message.
   * \param[in] disable Whether to disable the input topic or enable it (`true` means disable).
   * \param[in] now Current ROS time.
   */
  virtual void disableCb(const ::std::string& inTopic, const ::ros::Time& stamp, bool disable, const ::ros::Time& now);

  /**
   * \brief Callback function run when a lock message is received.
   *
   * \param[in] topic Name of the lock topic.
   * \param[in] stamp ROS time stamp of the message.
   * \param[in] locked Whether to lock the lock or unlock it (`true` means lock).
   * \param[in] now Current ROS time.
   */
  virtual void lockCb(const ::std::string& topic, const ::ros::Time& stamp, bool locked, const ::ros::Time& now);

  /**
   * \brief Updates the mux after changes have been made to some of its internal state.
   *
   * \param[in] now Current ROS time.
   */
  virtual void update(const ::ros::Time& now);

  /**
   * \brief Resets the mux to its initial state.
   * 
   * \param[in] now Current time.
   */
  virtual void reset(const ::ros::Time& now);

  /**
   * \brief Return the last selected topic for each output topic.
   *
   * \return The last selected topic for each output topic.
   */
  const ::std::unordered_map<::std::string, ::std::string>& getLastSelectedTopics() const;

  /**
   * \brief Return the active priority level.
   *
   * \return The active priority level.
   */
  int getActivePriority() const;

  /**
   * Whether the given topic is currently disabled.
   * \param[in] inTopic The topic name.
   * \param[in] now Current time.
   * \return Whether the topic is disabled.
   */
  bool isDisabled(const ::std::string& inTopic, const ::ros::Time& now);

protected:
  /**
   * \brief Gets the highest locked priority.
   *
   * \param[in] now Current ROS time.
   * \return The highest locked priority.
   */
  int getHighestLockedPriority(const ::ros::Time& now);

  //! \brief Configurations of input topics.
  ::std::unordered_map<::std::string, ::cras::priority_mux::TopicConfig> topicConfigs;

  //! \brief Configurations of lock topics.
  ::std::unordered_map<::std::string, ::cras::priority_mux::LockConfig> lockConfigs;

  //! \brief The function to call when the mux needs to schedule a call to `update()` after some time.
  SetTimerFn setTimer;

  /*
   * Implementation note: we use the composite key <priority, topic> instead of just <topic> in the maps below.
   * The reasoning is that in the ordered maps, sorting first by priority and only then by string comparison should be
   * much faster than sorting just by the strings.
   */

  //! \brief The timestamps of the last received message for each input topic.
  ::std::map<::std::pair<int, ::std::string>, ::ros::Time> lastReceiveStamps;

  //! \brief The timestamps of the last received disable message for each input topic.
  ::std::map<::std::pair<int, ::std::string>, ::ros::Time> disabledStamps;

  //! \brief The timestamps of the last received lock message for each lock topic.
  ::std::map<::std::pair<int, ::std::string>, ::ros::Time> lastLockStamps;

  //! \brief Current lock status of each lock topic (whether the lock is explicitly locked or not).
  ::std::map<::std::pair<int, ::std::string>, bool> lockStates;

  //! \brief Virtual name of a topic reported as selected when no priority is active.
  ::std::string noneTopic;

  //! \brief Priority level signalling that no priority is active.
  int nonePriority;

  //! \brief The currently active priority level.
  int lastActivePriority;

  //! \brief The currently selected topic for each output topic.
  ::std::unordered_map<::std::string, ::std::string> lastSelectedTopics;

private:
  /**
   * \brief Reset the mux to its original state.
   * 
   * \param[in] now Current time.
   */
  void resetImpl(const ::ros::Time& now);
};

}
