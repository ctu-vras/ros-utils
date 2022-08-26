#pragma once

/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <std_msgs/Bool.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>

namespace cras
{

namespace priority_mux
{

static const ::std::string NONE_TOPIC = "__none";

struct TopicConfig
{
  ::std::string name;
  ::std::string inTopic;
  ::std::string outTopic;
  int priority{0};
  ::ros::Duration timeout;
  size_t queueSize{0u};
};

struct LockConfig
{
  ::std::string name;
  ::std::string topic;
  int priority{0};
  ::ros::Duration timeout;
  size_t queueSize{0u};
};

}

class PriorityMuxNodelet : public ::cras::Nodelet
{
protected:
  void onInit() override;

  virtual void cb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event, const ::std::string& inTopic);
  virtual void lockCb(const ::std::string& topic, const ::std_msgs::BoolConstPtr& msg);  
  virtual void updatePriorities(int newPriority, const ::std::string& newTopic);
  
  void publishPriorityChange(int newPriority, const ::ros::Duration& timeout);
  void publishSelectedTopicChange(const ::std::string& outTopic, const ::std::string& newTopic,
    const ::ros::Duration& timeout);
  void onPriorityTimeout(const ::ros::TimerEvent&);
  void onSelectedTopicTimeout(const ::std::string& outTopic, const ::ros::TimerEvent&);
  void onLockTimeout(const ::std::string& topic, const ::ros::TimerEvent&);
  
  ::std::list<::ros::Subscriber> subscribers;
  ::std::unordered_map<::std::string, ::ros::Publisher> publishers;
  ::ros::Publisher activePriorityPub;
  ::std::unordered_map<::std::string, ::ros::Publisher> selectedPublishers;
  ::std::unordered_map<::std::string, ::ros::Publisher> lockedPublishers;
  
  ::ros::Timer priorityBackToNoneTimer;
  ::std::unordered_map<::std::string, ::ros::Timer> selectedBackToNoneTimers;
  ::std::unordered_map<::std::string, ::ros::Timer> lockTimeoutTimers;
  
  ::std::unordered_map<::std::string, ::cras::priority_mux::TopicConfig> topicConfigs;
  ::std::unordered_map<::std::string, ::cras::priority_mux::LockConfig> lockConfigs;
  
  ::std::map<::std::pair<int, ::std::string>, ::ros::Time> lastReceiveStamps;
  ::std::map<::std::pair<int, ::std::string>, ::ros::Time> lastLockStamps;
  
  size_t outQueueSize {10u};
  int priorityNone {0};
  int lastActivePriority {0};
  ::std::unordered_map<::std::string, ::std::string> lastSelectedTopics;
};

}