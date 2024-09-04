// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a simple implementation of a repeater nodelet.
 * \author Martin Pecka
 */

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscribe_options.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/repeat.h>
#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

void RepeatMessagesNodelet::onInit()
{
  auto nh = this->getMTPrivateNodeHandle();
  std::string inTopic = "input";
  std::string outTopic = "output";
  ros::Rate defaultRate(1.0);

  // Allow basic configuration via command line args.
  if (!this->getMyArgv().empty())
  {
    if (this->getMyArgv().size() == 1)
      throw std::runtime_error("Not enough arguments.\nUsage: repeat IN_TOPIC RATE [OUT_TOPIC].");

    nh = this->getMTNodeHandle();
    inTopic = this->getMyArgv()[0];
    outTopic = (this->getMyArgv().size() >= 3 ? this->getMyArgv()[2] : (inTopic + "_repeat"));
    try
    {
      defaultRate = ros::Rate(cras::parseDouble(this->getMyArgv()[1]));
    }
    catch (const std::invalid_argument& e)
    {
      CRAS_WARN("Could not parse the given repeat rate: %s", e.what());
    }
  }

  auto params = this->privateParams();
  const auto inQueueSize = params->getParam("in_queue_size", 10_sz, "messages");
  const auto outQueueSize = params->getParam("out_queue_size", inQueueSize, "messages");
  const auto lazy = params->getParam("lazy", false);
  const auto tcpNoDelay = params->getParam("tcp_no_delay", false);

  const auto rate = params->getParam("rate", defaultRate, "Hz");
  this->rate = std::make_unique<ros::Rate>(rate);
  if (params->hasParam("max_age"))
    this->maxAge = params->getParam("max_age", ros::Duration(1), "s");
  if (params->hasParam("max_repeats"))
    this->maxRepeats = params->getParam("max_repeats", 10_sz);
  this->discardOlderMessages = params->getParam("discard_older_messages", false);
  this->resetOnMsg = params->getParam("reset_on_msg", true);
  this->publishOnlyOnTimer = params->getParam("publish_only_on_timer", false);

  ros::SubscribeOptions opts;
  opts.transport_hints.tcpNoDelay(tcpNoDelay);
  this->pubSub = std::make_unique<cras::GenericLazyPubSub>(
    nh, inTopic, outTopic, inQueueSize, outQueueSize, cras::bind_front(&RepeatMessagesNodelet::processMessage, this),
    opts, this->log);

  this->timer = nh.createTimer(this->rate->expectedCycleTime(), &RepeatMessagesNodelet::everyPeriod, this);

  if (!lazy)
    this->pubSub->setLazy(false);

  opts.transport_hints.tcpNoDelay(true);
  auto cb = boost::bind(&RepeatMessagesNodelet::onReset, this, boost::placeholders::_1);
  opts.initByFullCallbackType<const ros::MessageEvent<const topic_tools::ShapeShifter>&>("reset", 1, cb);
  this->resetSub = this->getMTPrivateNodeHandle().subscribe(opts);

  CRAS_INFO("Created%s repeater from %s to %s at rate %s Hz.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str(),
    cras::to_string(rate).c_str());
}

void RepeatMessagesNodelet::onReset(const ros::MessageEvent<const topic_tools::ShapeShifter>&)
{
  this->reset();
}

void RepeatMessagesNodelet::reset()
{
  NodeletWithSharedTfBuffer::reset();

  this->timer.setPeriod(this->rate->expectedCycleTime(), true);

  std::lock_guard<std::mutex> lock(this->msgMutex);
  this->msg.reset();
  this->numRepeats = 0;
  this->lastMsgStamp.reset();
}

void RepeatMessagesNodelet::processMessage(
  const ros::MessageEvent<const topic_tools::ShapeShifter>& event, ros::Publisher& pub)
{
  if (!this->pub)
    this->pub = pub;

  const auto& msg = event.getConstMessage();

  if (!this->hasHeader.has_value() && this->inspectStamps())
  {
    this->hasHeader = cras::hasHeader(*msg);

    if (!this->hasHeader.value())
    {
      CRAS_ERROR("Running repeat with timestamp conditions on message type %s which does not have a header! "
                 "Ignoring all messages.", event.getConnectionHeader()["type"].c_str());
    }
  }

  if (this->inspectStamps() && !this->hasHeader.value())
    return;

  cras::optional<ros::Time> stamp;
  // If inspecting time stamps is required, deserialize the Header and validate the time stamp.
  if (this->inspectStamps())
  {
    // This is potentially unsafe if the subscribed message actually does not have a Header field at the beginning.
    const auto header = cras::getHeader(*msg);
    if (header.has_value())
    {
      stamp = header->stamp;
      if (this->maxAge.has_value() && (stamp.value() + this->maxAge.value()) < ros::Time::now())
      {
        CRAS_INFO_THROTTLE(5.0, "Received message too old (%.3g s > %.3g s) will be discarded.",
                           (ros::Time::now() - stamp.value()).toSec(), this->maxAge.value().toSec());
        return;
      }
      if (this->discardOlderMessages && this->lastMsgStamp.has_value()
        && stamp.value() < this->lastMsgStamp.value())
      {
        CRAS_INFO_THROTTLE(
          5.0, "Received message is %.3g s older than current message, it will be discarded.",
          (this->lastMsgStamp.value() - stamp.value()).toSec());
        return;
      }
    }
  }

  // Record the incoming message.
  {
    std::lock_guard<std::mutex> lock(this->msgMutex);
    this->msg = msg;
    this->numRepeats = 0;
    this->lastMsgStamp = stamp;
  }

  // Republish the message right away if the configuration says so.
  if (!this->publishOnlyOnTimer)
    this->maybePublish();

  // If resetOnMsg, we reset the publication timer to start counting from zero.
  if (this->resetOnMsg)
    this->timer.setPeriod(this->rate->expectedCycleTime(), true);
}

void RepeatMessagesNodelet::maybePublish()
{
  if (!this->pub || this->msg == nullptr)
    return;

  std::lock_guard<std::mutex> lock(this->msgMutex);

  if (this->maxRepeats.has_value() && this->numRepeats > this->maxRepeats.value())
  {
    CRAS_WARN_THROTTLE(5.0, "Message already republished %zu times.", this->numRepeats);
    return;
  }

  if (this->inspectStamps() && this->maxAge.has_value() && this->lastMsgStamp.has_value() &&
    (this->lastMsgStamp.value() + this->maxAge.value()) < ros::Time::now())
  {
    CRAS_WARN_THROTTLE(5.0, "Message too old (%.3g s > %.3g s) will not be republished.",
                       (ros::Time::now() - this->lastMsgStamp.value()).toSec(), this->maxAge.value().toSec());
    return;
  }

  this->numRepeats += 1;
  this->pub.template publish(this->msg);
}

bool RepeatMessagesNodelet::inspectStamps() const
{
  return this->maxAge.has_value() || this->discardOlderMessages;
}

void RepeatMessagesNodelet::everyPeriod(const ros::TimerEvent&)
{
  this->maybePublish();
}

}

PLUGINLIB_EXPORT_CLASS(cras::RepeatMessagesNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
