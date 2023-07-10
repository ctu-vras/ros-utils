// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Priority-based muxer nodelet for topics.
 * \author Martin Pecka
 */

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <cras_topic_tools/priority_mux.h>
#include <cras_topic_tools/priority_mux_base.h>

namespace cras
{

constexpr auto DEFAULT_OUT_TOPIC = "mux_out";
constexpr auto DEFAULT_NONE_TOPIC = "__none";

void PriorityMuxNodelet::onInit()
{
  const auto params = this->privateParams();

  this->queueSize = params->getParam("queue_size", 10_sz, "messages");
  this->tcpNoDelay = params->getParam("tcp_no_delay", false);

  const auto defaultOutTopic = params->getParam("default_out_topic", std::string(DEFAULT_OUT_TOPIC));
  const auto noneTopic = params->getParam("none_topic", std::string(DEFAULT_NONE_TOPIC));
  const auto nonePriority = params->getParam("none_priority", 0);

  XmlRpc::XmlRpcValue topicParams;
  try
  {
    topicParams = params->getParam<XmlRpc::XmlRpcValue>("topics", cras::nullopt);
  }
  catch (const cras::GetParamException& e)
  {
    CRAS_ERROR("Parameter ~topics is empty, priority mux will not do anything!");
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
      CRAS_ERROR("Parameter ~topics has to be either a list or a dict. Priority mux will not do anything!");
      return;
  }

  std::unordered_map<std::string, std::string> disableTopics;

  for (const auto& item : topicItems)
  {
    const auto& key = item.first;
    const auto& xmlConfig = item.second;
    if (xmlConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      CRAS_ERROR("Item %s of ~topics has to be a dict, but %s was given.",
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
      CRAS_ERROR("Item %s of ~topics has to contain key 'topic'. Skipping the item.", key.c_str());
      continue;
    }

    config.name = xmlParams.getParam("name", config.inTopic);
    config.outTopic = xmlParams.getParam("out_topic", defaultOutTopic);
    try
    {
      config.priority = xmlParams.getParam<int>("priority", cras::nullopt);
    }
    catch (const cras::GetParamException& e)
    {
      CRAS_ERROR("Item %s of ~topics has to contain key 'priority'. Skipping the item.", key.c_str());
      continue;
    }
    try
    {
      config.timeout = xmlParams.getParam<ros::Duration>("timeout", cras::nullopt, "s");
    }
    catch (const cras::GetParamException& e)
    {
      CRAS_ERROR("Item %s of ~topics has to contain key 'timeout'. Skipping the item.", key.c_str());
      continue;
    }

    const auto disableTopic = xmlParams.getParam("disable_topic", std::string());
    if (!disableTopic.empty())
      disableTopics[config.inTopic] = disableTopic;

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
      CRAS_ERROR("Parameter ~locks has to be either a list or a dict. No locks will be set.");
      return;
  }

  if (lockItems.empty())
    CRAS_INFO("No locks were specified.");

  for (const auto& item : lockItems)
  {
    const auto& key = item.first;
    const auto& xmlConfig = item.second;
    if (xmlConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      CRAS_ERROR("Item %s of ~locks has to be a dict, but %s was given.",
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
      CRAS_ERROR("Item %s of ~locks has to contain key 'topic'. Skipping the item.", key.c_str());
      continue;
    }

    config.name = xmlParams.getParam("name", config.topic);
    try
    {
      config.priority = xmlParams.getParam<int>("priority", cras::nullopt);
    }
    catch (const cras::GetParamException& e)
    {
      CRAS_ERROR("Item %s of ~locks has to contain key 'priority'. Skipping the item.", key.c_str());
      continue;
    }
    try
    {
      config.timeout = xmlParams.getParam<ros::Duration>("timeout", cras::nullopt, "s");
    }
    catch (const cras::GetParamException& e)
    {
      CRAS_ERROR("Item %s of ~locks has to contain key 'timeout'. Skipping the item.", key.c_str());
      continue;
    }

    this->lockConfigs[config.topic] = config;
  }

  this->mux = std::make_unique<PriorityMux>(this->topicConfigs, this->lockConfigs,
    cras::bind_front(&PriorityMuxNodelet::setTimer, this), ros::Time::now(), this->getLogger(),
    noneTopic, nonePriority);

  this->activePriorityPub = this->getPrivateNodeHandle().advertise<std_msgs::Int32>("active_priority", 1, true);

  ros::TransportHints transportHints;
  transportHints.tcpNoDelay(this->tcpNoDelay);

  this->resetSub = this->getPrivateNodeHandle().subscribe(
    "reset", this->queueSize, &PriorityMuxNodelet::resetCb, this, transportHints);

  for (const auto& config : this->topicConfigs)
  {
    const auto& topicConfig = config.second;
    this->outTopics.insert(topicConfig.outTopic);

    const auto cb = cras::bind_front(&PriorityMuxNodelet::cb, this, topicConfig.inTopic);
    const auto sub = this->getNodeHandle().subscribe<topic_tools::ShapeShifter>(
      topicConfig.inTopic, this->queueSize, cb, nullptr, transportHints);
    this->subscribers.push_back(sub);
  }

  for (const auto& outTopic : this->outTopics)
  {
    const auto selectedTopic = "selected/" + cras::stripLeading(outTopic, '/');
    this->selectedPublishers[outTopic] =
      this->getPrivateNodeHandle().advertise<std_msgs::String>(selectedTopic, this->queueSize, true);
  }

  for (const auto& inTopicAndDisableTopic : disableTopics)
  {
    const auto& inTopic = inTopicAndDisableTopic.first;
    const auto& disableTopic = inTopicAndDisableTopic.second;

    const auto cb = cras::bind_front(&PriorityMuxNodelet::disableCb, this, inTopic);
    const auto sub = this->getNodeHandle().subscribe<topic_tools::ShapeShifter>(
      disableTopic, this->queueSize, cb, nullptr, transportHints);
    this->subscribers.push_back(sub);
  }

  for (const auto& config : this->lockConfigs)
  {
    const auto& lockConfig = config.second;
    const auto cb = cras::bind_front(&PriorityMuxNodelet::lockCb, this, lockConfig.topic);
    const auto sub = this->getNodeHandle().subscribe<std_msgs::Bool>(
      lockConfig.topic, this->queueSize, cb, nullptr, transportHints);
    this->subscribers.push_back(sub);
  }

  ros::WallDuration(0.1).sleep();  // Give publishers and subscribers time to wire up
  
  this->publishChanges();
}

void PriorityMuxNodelet::cb(
  const std::string& inTopic, const ros::MessageEvent<const topic_tools::ShapeShifter>& event)
{
  const auto& topicConfig = this->topicConfigs[inTopic];

  CRAS_DEBUG("Received message on topic %s with priority %i. Current priority %i.",
             inTopic.c_str(), topicConfig.priority, this->mux->getActivePriority());

  const auto shouldPublish = this->mux->cb(inTopic, event.getReceiptTime(), ros::Time::now());

  if (shouldPublish)
  {
    auto& pub = this->publishers[topicConfig.outTopic];
    if (!pub)
    {
      auto latch = false;
      if (event.getConnectionHeaderPtr() != nullptr)
        latch = event.getConnectionHeader()["latching"] == "1";
      pub = event.getConstMessage()->advertise(this->getNodeHandle(), topicConfig.outTopic, this->queueSize, latch);
      CRAS_DEBUG("Created publisher %s with type %s.", topicConfig.outTopic.c_str(),
                 event.getConstMessage()->getDataType().c_str());
      // Give the publisher some time to wire up so that we don't lose the first message.
      ros::WallDuration(0.1).sleep();
    }

    CRAS_DEBUG("Publishing message on topic %s.", inTopic.c_str());
    pub.publish(event.getConstMessage());
  }
  else
  {
    CRAS_DEBUG("Discarding message on topic %s.", inTopic.c_str());
  }

  this->publishChanges();
}

void PriorityMuxNodelet::lockCb(const std::string& topic, const ros::MessageEvent<std_msgs::Bool const>& event)
{
  this->mux->lockCb(topic, event.getReceiptTime(), event.getConstMessage()->data, ros::Time::now());

  this->publishChanges();
}

void PriorityMuxNodelet::disableCb(
  const std::string& inTopic, const ros::MessageEvent<const topic_tools::ShapeShifter>& event)
{
  bool disable = true;
  if (event.getConstMessage()->getDataType() == ros::message_traits::DataType<std_msgs::Bool>::value())
    disable = event.getConstMessage()->instantiate<std_msgs::Bool>()->data;

  this->mux->disableCb(inTopic, event.getReceiptTime(), disable, ros::Time::now());

  this->publishChanges();
}

void PriorityMuxNodelet::publishChanges()
{
  const auto newPriority = this->mux->getActivePriority();
  if (!this->lastActivePriority || newPriority != *this->lastActivePriority)
  {
    CRAS_INFO("Priority %i is now active.", newPriority);
    std_msgs::Int32 msg;
    msg.data = newPriority;
    this->activePriorityPub.publish(msg);
  }
  this->lastActivePriority = newPriority;

  const auto& newTopics = this->mux->getLastSelectedTopics();
  const auto& oldTopics = this->lastSelectedTopics;
  for (const auto& outTopic : this->outTopics)
  {
    const auto newIt = newTopics.find(outTopic);
    const auto oldIt = oldTopics.find(outTopic);
    if (newIt == newTopics.end() || oldIt == oldTopics.end() || newIt->second != oldIt->second)
    {
      CRAS_INFO("Source topic '%s' is now selected for output topic '%s'.", newIt->second.c_str(), outTopic.c_str());
      std_msgs::String msg;
      msg.data = newIt->second;
      this->selectedPublishers[outTopic].publish(msg);
    }
  }
  this->lastSelectedTopics = newTopics;
}

void PriorityMuxNodelet::onTimeout(const std::string&, const ros::TimerEvent& event)
{
  this->mux->update(event.current_real);

  this->publishChanges();
}

void PriorityMuxNodelet::setTimer(const std::string& name, const ros::Duration& timeout)
{
  if (this->timers.find(name) == this->timers.end())
  {
    const auto cb = cras::bind_front(&PriorityMuxNodelet::onTimeout, this, name);
    this->timers[name] = this->getNodeHandle().createTimer(timeout, cb, true, true);
  }
  else
  {
    this->timers[name].setPeriod(timeout);
  }
}

void PriorityMuxNodelet::reset()
{
  CRAS_WARN("Resetting mux.");

  for (auto& timer : this->timers)
    timer.second.stop();
  this->timers.clear();

  this->lastActivePriority.reset();
  this->lastSelectedTopics.clear();
  this->mux->reset(ros::Time::now());

  this->publishChanges();
}

void PriorityMuxNodelet::resetCb(const ros::MessageEvent<const ::topic_tools::ShapeShifter>&)
{
  this->reset();
}

}

PLUGINLIB_EXPORT_CLASS(cras::PriorityMuxNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
