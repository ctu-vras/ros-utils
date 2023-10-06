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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <cras_topic_tools/priority_mux.h>
#include <cras_topic_tools/priority_mux_base.h>
#include <cras_topic_tools/shape_shifter.h>

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
  const auto defaultSubscriberConnectDelay = params->getParam("subscriber_connect_delay", ros::WallDuration(0.1));

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
    config.queueSize = xmlParams.getParam("queue_size", this->queueSize);
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
    {
      disableTopics[config.inTopic] = disableTopic;
      // If there should be a message sent just before disabling this topic, read the message from a bag
      const auto beforeDisableMessageBag = xmlParams.getParam("before_disable_message", std::string());
      if (!beforeDisableMessageBag.empty())
      {
        try
        {
          rosbag::Bag bag(beforeDisableMessageBag);
          rosbag::View view(bag);
          if (view.size() == 0)
          {
            CRAS_ERROR("Bag file %s does not contain any messages.", beforeDisableMessageBag.c_str());
          }
          else
          {
            // Read the first message from the bag and copy it to a ShapeShifter
            const auto msg = *view.begin();
            auto shifter = boost::make_shared<cras::ShapeShifter>();
            cras::resizeBuffer(*shifter, msg.size());
            ros::serialization::OStream stream(cras::getBuffer(*shifter), msg.size());
            msg.write(stream);
            shifter->morph(msg.getMD5Sum(), msg.getDataType(), msg.getMessageDefinition(), msg.isLatching() ? "1" : "");
            // If the message has Header, figure out if we need to overwrite frame_id
            if (cras::hasHeader(*shifter))
            {
              // This set is read by disableCb().
              this->beforeDisableMessagesWithHeader.insert(config.inTopic);
              const auto frameId = xmlParams.getParam("before_disable_message_frame_id", std::string());
              if (!frameId.empty())
              {
                auto header = cras::getHeader(*shifter);
                if (header)
                {
                  header->frame_id = frameId;
                  cras::setHeader(*shifter, *header);
                }
              }
            }
            // Create a fake message event. Time does not matter as the code in disableCb() ignores it.
            ros::MessageEvent<const cras::ShapeShifter> event;
            event.init(shifter, msg.getConnectionHeader(), ros::Time(0), true,
                       ros::DefaultMessageCreator<cras::ShapeShifter>());
            this->beforeDisableMessages[config.inTopic] = event;
          }
        }
        catch (const rosbag::BagException& e)
        {
          CRAS_ERROR("%s", e.what());
        }
      }
    }

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

  ros::TransportHints transportHints;
  transportHints.tcpNoDelay(this->tcpNoDelay);

  for (const auto& config : this->topicConfigs)
  {
    const auto& topicConfig = config.second;
    this->outTopics.insert(topicConfig.outTopic);
    this->outTopicConfigs[topicConfig.outTopic].queueSize = this->queueSize;
    this->outTopicConfigs[topicConfig.outTopic].subscriberConnectDelay = defaultSubscriberConnectDelay;

    const auto cb = cras::bind_front(&PriorityMuxNodelet::cb, this, topicConfig.inTopic);
    const auto sub = this->getNodeHandle().subscribe<topic_tools::ShapeShifter>(
      topicConfig.inTopic, topicConfig.queueSize, cb, nullptr, transportHints);
    this->subscribers.push_back(sub);
  }

  std::list<std::pair<std::string, XmlRpc::XmlRpcValue>> outTopicItems;
  if (params->hasParam("out_topics"))
  {
    const auto outTopicParams = params->getParam<XmlRpc::XmlRpcValue>("out_topics", cras::nullopt);
    switch (outTopicParams.getType())
    {
      case XmlRpc::XmlRpcValue::TypeArray:
        for (size_t i = 0; i < outTopicParams.size(); ++i)
          outTopicItems.emplace_back("[" + cras::to_string(i) + "]", outTopicParams[i]);
        break;
      case XmlRpc::XmlRpcValue::TypeStruct:
        for (const auto& item : outTopicParams)
          outTopicItems.emplace_back("/" + item.first, item.second);
        break;
      default:
        CRAS_ERROR("Parameter ~out_topics has to be either a list or a dict. It will be ignored!");
    }
  }

  for (const auto& item : outTopicItems)
  {
    const auto& key = item.first;
    const auto& xmlConfig = item.second;
    if (xmlConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      CRAS_ERROR("Item %s of ~out_topics has to be a dict, but %s was given.",
        key.c_str(), cras::to_string(xmlConfig.getType()).c_str());
      continue;
    }

    auto itemNamespace = this->getPrivateNodeHandle().resolveName("out_topics") + key;
    auto xmlParamAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(xmlConfig, itemNamespace);
    cras::BoundParamHelper xmlParams(this->log, xmlParamAdapter);

    std::string topic;
    try
    {
      topic = xmlParams.getParam<std::string>("topic", cras::nullopt);
      if (topic.empty())
      {
        CRAS_ERROR("Item %s of ~out_topics has to contain key 'topic'. Skipping the item.", key.c_str());
        continue;
      }
      else
      {
        this->outTopicConfigs[topic].topic = topic;
      }
    }
    catch (const cras::GetParamException& e)
    {
      CRAS_ERROR("Item %s of ~out_topics has to contain key 'topic'. Skipping the item.", key.c_str());
      continue;
    }

    auto& config = this->outTopicConfigs[topic];

    config.queueSize = xmlParams.getParam("queue_size", config.queueSize);
    config.subscriberConnectDelay = xmlParams.getParam("subscriber_connect_delay", config.subscriberConnectDelay);
    config.numSubscribersToWait = xmlParams.getParam("num_subscribers_to_wait", config.numSubscribersToWait);
    if (xmlParams.hasParam("force_latch"))
      config.forceLatch = xmlParams.getParam<bool>("force_latch", cras::nullopt);
  }

  for (const auto& outTopic : this->outTopics)
  {
    const auto selectedTopic = "selected/" + cras::stripLeading(outTopic, '/');
    this->selectedPublishers[outTopic] =
      this->getPrivateNodeHandle().advertise<std_msgs::String>(selectedTopic, this->queueSize, true);
  }

  this->mux = std::make_unique<PriorityMux>(this->topicConfigs, this->lockConfigs,
    cras::bind_front(&PriorityMuxNodelet::setTimer, this), ros::Time::now(), this->getLogger(),
    noneTopic, nonePriority);

  this->activePriorityPub = this->getPrivateNodeHandle().advertise<std_msgs::Int32>(
    "active_priority", this->queueSize, true);

  this->resetSub = this->getPrivateNodeHandle().subscribe(
    "reset", this->queueSize, &PriorityMuxNodelet::resetCb, this, transportHints);

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
      const auto& outTopicConfig = this->outTopicConfigs[topicConfig.outTopic];

      // Figure out if the publisher should be latched
      auto latch = false;
      if (event.getConnectionHeaderPtr() != nullptr)
        latch = event.getConnectionHeader()["latching"] == "1";
      if (outTopicConfig.forceLatch.has_value())
        latch = *outTopicConfig.forceLatch;

      // Create the publisher
      pub = event.getConstMessage()->advertise(
        this->getNodeHandle(), topicConfig.outTopic, outTopicConfig.queueSize, latch);
      CRAS_DEBUG("Created publisher %s with type %s.", topicConfig.outTopic.c_str(),
                 event.getConstMessage()->getDataType().c_str());

      // Give the publisher some time to wire up so that we don't lose the first message.
      if (!latch)
      {
        const auto waitStart = ros::WallTime::now();
        while (ros::ok() && pub.getNumSubscribers() < outTopicConfig.numSubscribersToWait)
        {
          ros::WallDuration(0.01).sleep();
          if (ros::WallTime::now() - waitStart > ros::WallDuration(1, 0))
          {
            CRAS_WARN_THROTTLE(1.0, "Waiting for subscribers to topic %s (has %u/%zu).",
                               topicConfig.outTopic.c_str(), pub.getNumSubscribers(),
                               outTopicConfig.numSubscribersToWait);
          }
        }
        if (ros::WallTime::now() < (waitStart + outTopicConfig.subscriberConnectDelay))
          (outTopicConfig.subscriberConnectDelay - (ros::WallTime::now() - waitStart)).sleep();
        CRAS_DEBUG("Waited %f seconds before publishing.", (ros::WallTime::now() - waitStart).toSec());
      }
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

  // Publish the "before disabled" message if there is any and the topic gets disabled right now
  if (disable && !this->mux->isDisabled(inTopic, event.getReceiptTime()))
  {
    const auto& it = this->beforeDisableMessages.find(inTopic);
    if (it != this->beforeDisableMessages.end())
    {
      const auto& itEvent = it->second;
      auto message = itEvent.getConstMessage();
      auto nonconstWillCopy = itEvent.nonConstWillCopy();
      // If the message type has header, update stamp to current time
      if (this->beforeDisableMessagesWithHeader.find(inTopic) != this->beforeDisableMessagesWithHeader.end())
      {
        auto header = cras::getHeader(*message);
        if (header)
        {
          header->stamp = ros::Time::now();
          auto nonconstMessage = itEvent.getMessage();
          cras::setHeader(*nonconstMessage, *header);
          message = nonconstMessage;
          nonconstWillCopy = false;  // We already made a copy and nobody else will use it
        }
      }

      // Create a fake message event
      ros::MessageEvent<const topic_tools::ShapeShifter> beforeEvent;
      beforeEvent.init(message, itEvent.getConnectionHeaderPtr(), event.getReceiptTime(), nonconstWillCopy,
                       itEvent.getMessageFactory());

      // Inject the constructed message as if it were received on the inTopic.
      this->cb(inTopic, beforeEvent);
    }
  }

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
