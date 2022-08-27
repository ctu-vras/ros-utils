/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <cras_topic_tools/priority_mux.h>

namespace cras
{

constexpr auto DEFAULT_OUT_TOPIC = "mux_out";

void PriorityMuxNodelet::onInit()
{
  const auto params = this->privateParams();
  
  this->outQueueSize = params->getParam("out_queue_size", 10_sz, "messages");
  this->lastActivePriority = this->priorityNone = params->getParam("priority_none", 0);
  
  XmlRpc::XmlRpcValue topicParams;
  try
  {
    topicParams = params->getParam<XmlRpc::XmlRpcValue>("topics", cras::nullopt);
  }
  catch (const cras::GetParamException& e)
  {
    this->log->logError("Parameter ~topics is empty, priority mux will not do anything!");
    return;
  }
  
  std::list<std::pair<std::string, XmlRpc::XmlRpcValue>> topicItems;
  switch (topicParams.getType())
  {
    case XmlRpc::XmlRpcValue::TypeArray:
      for (size_t i = 0; i < topicParams.size(); ++i)
        topicItems.emplace_back("[" + cras::to_string(i) + "]", topicParams[i]);
      break;
    case XmlRpc::XmlRpcValue::TypeStruct:
      for (const auto& item : topicParams)
        topicItems.emplace_back("/" + item.first, item.second);
      break;
    default:
      this->log->logError("Parameter ~topics has to be either a list or a dict. Priority mux will not do anything!");
      return;
  }
  
  for (const auto& item : topicItems)
  {
    const auto& key = item.first;
    const auto& xmlConfig = item.second;
    if (xmlConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      this->log->logError("Item %s of ~topics has to be a dict, but %s was given.",
        key.c_str(), cras::to_string(xmlConfig.getType()).c_str());
      continue;
    }

    auto itemNamespace = this->getPrivateNodeHandle().resolveName("topics") + key;
    auto xmlParamAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(xmlConfig, itemNamespace);
    cras::BoundParamHelper xmlParams(this->log, xmlParamAdapter);
    cras::priority_mux::TopicConfig config;
    
    try
    {
      config.inTopic = xmlParams.getParam<std::string>("topic", cras::nullopt);
    }
    catch (const cras::GetParamException& e)
    {
      this->log->logError("Item %s of ~topics has to contain key 'topic'. Skipping the item.", key.c_str());
      continue;
    }

    config.name = xmlParams.getParam("name", config.inTopic);
    config.outTopic = xmlParams.getParam("out_topic", DEFAULT_OUT_TOPIC);
    config.priority = xmlParams.getParam("priority", 100, "", {true, true});
    config.timeout = xmlParams.getParam("timeout", ros::Duration{1, 0}, "s", {true, true});
    config.queueSize = xmlParams.getParam("queue_size", 10_sz, "messages");
    
    this->topicConfigs[config.inTopic] = config;
  }
  
  XmlRpc::XmlRpcValue lockParams;
  lockParams.begin();  // convert to struct
  
  lockParams = params->getParam("locks", lockParams);
  std::list<std::pair<std::string, XmlRpc::XmlRpcValue>> lockItems;
  switch (lockParams.getType())
  {
    case XmlRpc::XmlRpcValue::TypeArray:
      for (size_t i = 0; i < lockParams.size(); ++i)
        lockItems.emplace_back("[" + cras::to_string(i) + "]", lockParams[i]);
      break;
    case XmlRpc::XmlRpcValue::TypeStruct:
      for (const auto& item : lockParams)
        lockItems.emplace_back("/" + item.first, item.second);
      break;
    default:
      this->log->logError("Parameter ~locks has to be either a list or a dict. No locks will be set.");
      return;
  }
  
  if (lockItems.empty())
    this->log->logInfo("No locks were specified.");

  for (const auto& item : lockItems)
  {
    const auto& key = item.first;
    const auto& xmlConfig = item.second;
    if (xmlConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      this->log->logError("Item %s of ~locks has to be a dict, but %s was given.",
        key.c_str(), cras::to_string(xmlConfig.getType()).c_str());
      continue;
    }

    auto itemNamespace = this->getPrivateNodeHandle().resolveName("locks") + key;
    auto xmlParamAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(xmlConfig, itemNamespace);
    cras::BoundParamHelper xmlParams(this->log, xmlParamAdapter);
    cras::priority_mux::LockConfig config;
    
    try
    {
      config.topic = xmlParams.getParam<std::string>("topic", cras::nullopt);
    }
    catch (const cras::GetParamException& e)
    {
      this->log->logError("Item %s of ~locks has to contain key 'topic'. Skipping the item.", key.c_str());
      continue;
    }

    config.name = xmlParams.getParam("name", config.topic);
    config.priority = xmlParams.getParam("priority", 100, "", {true, true});
    config.timeout = xmlParams.getParam("timeout", ros::Duration{1, 0}, "s", {true, true});
    config.queueSize = xmlParams.getParam("queue_size", 10_sz, "messages");
    
    this->lockConfigs[config.topic] = config;
  }
  
  this->activePriorityPub = this->getPrivateNodeHandle().advertise<std_msgs::Int32>(
    "active_priority", this->outQueueSize, true);
  ros::WallDuration(0.1).sleep();
  
  std_msgs::Int32 msg;
  msg.data = this->priorityNone;
  this->activePriorityPub.publish(msg);
  this->log->logInfo("No priority active now.");
  
  for (const auto& config : this->topicConfigs)
  {
    const auto& topicConfig = config.second;
    
    if (!this->selectedPublishers[topicConfig.outTopic])
    {
      const auto selectedTopic = "selected/" + cras::stripLeading(topicConfig.outTopic, '/');
      this->selectedPublishers[topicConfig.outTopic] =
        this->getPrivateNodeHandle().advertise<std_msgs::String>(selectedTopic, this->outQueueSize, true);
      ros::WallDuration(0.1).sleep();
      
      std_msgs::String selectedMsg;
      selectedMsg.data = priority_mux::NONE_TOPIC;
      this->selectedPublishers[topicConfig.outTopic].publish(selectedMsg);
      this->log->logInfo("No topic is now selected for output topic %s.", topicConfig.outTopic.c_str());
    }
    
    const auto cb = boost::bind(&PriorityMuxNodelet::cb, this, boost::placeholders::_1, topicConfig.inTopic);
    const auto sub = this->getNodeHandle().subscribe<topic_tools::ShapeShifter>(
      topicConfig.inTopic, topicConfig.queueSize, cb);
    this->subscribers.push_back(sub);
  }
  
  for (const auto& config : this->lockConfigs)
  {
    const auto& lockConfig = config.second;
    
    const auto lockedTopic = "locked/" + cras::stripLeading(lockConfig.topic, '/');
    this->lockedPublishers[lockConfig.topic] =
      this->getPrivateNodeHandle().advertise<std_msgs::Bool>(lockedTopic, this->outQueueSize, true);
    ros::WallDuration(0.1).sleep();

    // Pretend that each lock has received a message when starting the mux
    this->lastLockStamps[std::make_pair(lockConfig.priority, lockConfig.topic)] = ros::Time::now();
    
    std_msgs::Bool lockedMsg;
    lockedMsg.data = false;
    this->lockedPublishers[lockConfig.topic].publish(lockedMsg);
    this->log->logInfo("Lock %s is not locked.", lockConfig.name.c_str());
    
    const auto cb = cras::bind_front(&PriorityMuxNodelet::lockCb, this, lockConfig.topic);
    const auto sub = this->getNodeHandle().subscribe<std_msgs::Bool>(lockConfig.topic, lockConfig.queueSize, cb);
    this->subscribers.push_back(sub);
  }
}

void PriorityMuxNodelet::cb(
  const ros::MessageEvent<const topic_tools::ShapeShifter>& event, const std::string& inTopic)
{
  const auto& topicConfig = this->topicConfigs[inTopic];
  const auto& priority = topicConfig.priority;
  const auto& outTopic = topicConfig.outTopic;
  this->lastReceiveStamps[std::make_pair(priority, inTopic)] = ros::Time::now();
  
  auto highestLockedPriority = std::numeric_limits<int>::min();
  for (auto it = this->lastLockStamps.crbegin(); it != this->lastLockStamps.crend(); ++it)
  {
    const auto& itPriority = it->first.first;
    const auto& itTopic = it->first.second;
    const auto& itStamp = it->second;
    const auto& itConfig = this->lockConfigs[itTopic];
    const auto& itTimeout = itConfig.timeout;
    if (itPriority > highestLockedPriority && (itStamp + itTimeout < ros::Time::now()))
      highestLockedPriority = itPriority;
  }

  for (auto it = this->lastReceiveStamps.crbegin(); it != this->lastReceiveStamps.crend(); ++it)
  {
    const auto& itPriority = it->first.first;

    if (itPriority <= priority)
    {
      this->updatePriorities(priority, inTopic);
      break;
    }
    
    if (itPriority < highestLockedPriority)
    {
      this->log->logDebug("Priority %i not active, as a lock with priority %i is locked.",
        priority, highestLockedPriority);
      return;
    }

    const auto& itTopic = it->first.second;
    const auto& itConfig = this->topicConfigs[itTopic];
    const auto& itStamp = it->second;
    if ((ros::Time::now() - itStamp) < itConfig.timeout)
    {
      this->log->logDebug("Priority %i not active. Found higher active priority %i.", priority, itPriority);
      return;
    }
  }
  
  auto& pub = this->publishers[topicConfig.outTopic];
  if (!pub)
  {
    auto latch = false;
    if (event.getConnectionHeaderPtr() != nullptr)
      latch = event.getConnectionHeader()["latching"] == "1";
    pub = event.getConstMessage()->advertise(this->getNodeHandle(), topicConfig.outTopic, this->outQueueSize, latch);
    ros::WallDuration(0.1).sleep();
  }
  
  if (this->priorityBackToNoneTimer.isValid())
    this->priorityBackToNoneTimer.setPeriod(topicConfig.timeout, true);
  if (this->selectedBackToNoneTimers[topicConfig.outTopic].isValid())
    this->selectedBackToNoneTimers[topicConfig.outTopic].setPeriod(topicConfig.timeout, true);
  pub.publish(event.getConstMessage());
}

void PriorityMuxNodelet::lockCb(const std::string& topic, const std_msgs::BoolConstPtr& msg)
{
  const auto& lockConfig = this->lockConfigs[topic];
  const auto& priority = lockConfig.priority;
  const auto& timeout = lockConfig.timeout;

  auto& stamp = this->lastLockStamps[std::make_pair(priority, topic)];

  // Check if the lock was locked before; if it was, announce that it is being unlocked.
  if (stamp + timeout < ros::Time::now())
  {
    std_msgs::Bool lockedMsg;
    lockedMsg.data = false;
    this->lockedPublishers[topic].publish(lockedMsg);
    this->log->logInfo("Lock %s is unlocked now.", lockConfig.name.c_str());
  }
  
  auto time = ros::Time::now();
  // It the message contains "true", set the timeout so that it is already expired
  if (msg->data)
  {
    if (ros::Time(0, 1) + timeout > time)
      time -= timeout + ros::Duration(0, 1);
    else
      time = ros::Time(0, 0); // TODO: this will prevent timeouting a lock at the start of a simulation
  }
  stamp = time;
  
  auto& timer = this->lockTimeoutTimers[topic];
  if (timer.isValid())
  {
    if (msg->data)
      timer.setPeriod({0, 0}, true);
    else
      timer.setPeriod(timeout, true);
  }
  else
  {
    auto cb = cras::bind_front(&PriorityMuxNodelet::onLockTimeout, this, topic);
    timer = this->getNodeHandle().createTimer(timeout, cb, true);
  }
}

void PriorityMuxNodelet::updatePriorities(const int newPriority, const ::std::string& newTopic)
{
  const auto& config = this->topicConfigs[newTopic];

  if (newPriority != this->lastActivePriority)
  {
    this->log->logInfo("Priority %i is now active.", newPriority);
    this->lastActivePriority = newPriority;
    this->publishPriorityChange(newPriority, config.timeout);
  }

  if (newTopic != this->lastSelectedTopics[config.outTopic])
  {
    this->log->logInfo("Source topic '%s' is now selected for output topic '%s'.",
      newTopic.c_str(), config.outTopic.c_str());
    this->lastSelectedTopics[config.outTopic] = newTopic;
    this->publishSelectedTopicChange(config.outTopic, newTopic, config.timeout);
  }
  
  for (const auto& topicConfig : this->topicConfigs)
  {
    const auto& itPriority = topicConfig.second.priority;
    if (itPriority >= newPriority)
      break;
    
    const auto& itTopic = topicConfig.second.inTopic;
    const auto outTopic = topicConfig.second.outTopic;
    if (this->lastSelectedTopics[outTopic] == itTopic && this->selectedBackToNoneTimers[outTopic].isValid())
      this->selectedBackToNoneTimers[outTopic].setPeriod(ros::Duration(0, 0), true);
  }
}

void PriorityMuxNodelet::publishPriorityChange(const int newPriority, const ::ros::Duration& timeout)
{
  std_msgs::Int32 msg;
  msg.data = newPriority;
  this->activePriorityPub.publish(msg);
  
  if (this->priorityBackToNoneTimer.isValid())
    this->priorityBackToNoneTimer.stop();
  this->priorityBackToNoneTimer = this->getNodeHandle().createTimer(
    timeout, &PriorityMuxNodelet::onPriorityTimeout, this, true);
}

void PriorityMuxNodelet::publishSelectedTopicChange(const ::std::string& outTopic, const std::string& newTopic,
  const ros::Duration& timeout)
{
  std_msgs::String msg;
  msg.data = newTopic;
  this->selectedPublishers[outTopic].publish(msg);

  if (this->selectedBackToNoneTimers[outTopic].isValid())
    this->selectedBackToNoneTimers[outTopic].stop();
  auto cb = ::cras::bind_front(&PriorityMuxNodelet::onSelectedTopicTimeout, this, outTopic);
  this->selectedBackToNoneTimers[outTopic] = this->getNodeHandle().createTimer(timeout, cb, true);
}

void PriorityMuxNodelet::onPriorityTimeout(const ros::TimerEvent&)
{
  std_msgs::Int32 msg;
  msg.data = this->priorityNone;
  this->activePriorityPub.publish(msg);
  this->lastActivePriority = this->priorityNone;
  this->log->logInfo("No priority is now active.");
}

void PriorityMuxNodelet::onSelectedTopicTimeout(const ::std::string& outTopic, const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = priority_mux::NONE_TOPIC;
  this->selectedPublishers[outTopic].publish(msg);
  this->lastSelectedTopics[outTopic] = priority_mux::NONE_TOPIC;
  this->log->logInfo("No topic is now selected for output topic %s.", outTopic.c_str());
}

void PriorityMuxNodelet::onLockTimeout(const std::string& topic, const ros::TimerEvent&)
{
  std_msgs::Bool msg;
  msg.data = true;
  this->lockedPublishers[topic].publish(msg);
  this->log->logInfo("Lock %s is locked now.", this->lockConfigs[topic].name.c_str());
}

}