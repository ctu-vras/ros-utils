// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers. Version for unknown message
 *        type of both the subscriber and the publisher (implementation details, do not include directly).
 * \author Martin Pecka
 */

#include <mutex>
#include <string>
#include <utility>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <ros/advertise_options.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/log_utils.h>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/lazy_subscriber.hpp>
#include <cras_topic_tools/shape_shifter.h>

cras::GenericLazyPubSub::GenericLazyPubSub(
  const ros::NodeHandle& nhIn, const std::string& topicIn, const ros::NodeHandle& nhOut, const std::string& topicOut,
  const size_t inQueueSize, const size_t outQueueSize, CallbackType callback,
  ros::SubscribeOptions subscribeOptions, const cras::LogHelperPtr& logHelper) :
    cras::ConditionalSubscriber(cras::bind_front(&cras::GenericLazyPubSub::subscribe, this), logHelper),
    topicIn(topicIn), topicOut(topicOut), callback(std::move(callback)), subscribeOptions(std::move(subscribeOptions)),
    inQueueSize(inQueueSize), outQueueSize(outQueueSize), nhIn(nhIn), nhOut(nhOut)
{
  // We have to connect at the beginning so that we can create the publisher from a subscribed message.
  std::lock_guard<std::mutex> lock(this->connectMutex);
  this->connectNoLock();
}

void cras::GenericLazyPubSub::subscribe(ros::Subscriber& sub)
{
  ros::SubscribeOptions opts = this->subscribeOptions;
  opts.template initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
    this->topicIn, this->inQueueSize, cras::bind_front(&cras::GenericLazyPubSub::cb, this));
  sub = this->nhIn.subscribe(opts);
}

bool cras::GenericLazyPubSub::shouldBeSubscribed() const
{
  if (!this->pub)
    return true;
  return this->pub.getNumSubscribers() > 0;
}

void cras::GenericLazyPubSub::connectCb(const ros::SingleSubscriberPublisher&)
{
  if (!this->lazy)
    return;
  this->updateSubscription();
}

void cras::GenericLazyPubSub::cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
  const auto& msg = event.getConstMessage();
  if (!this->pub)
  {
    std::lock_guard<std::mutex> pubLock(this->pubCreateMutex);
    if (!this->pub)  // the first check is outside mutex, this one is inside
    {
      this->advertiseOptions = this->createAdvertiseOptions(event);  // cannot be const - advertise requires non-const
      CRAS_INFO("Creating%s publisher on %s with type %s.",
        (this->advertiseOptions->latch ? " latched" : ""),
        this->nhOut.resolveName(this->topicOut).c_str(),
        msg->getDataType().c_str());

      {  // Advertise the publisher
        std::lock_guard<std::mutex> lock(this->connectMutex);
        this->pub = this->nhOut.advertise(this->advertiseOptions.value());
      }
    }

    for (size_t i = 0; i < 100 && ros::ok() && this->pub.getNumSubscribers() == 0; ++i)
      ros::WallDuration(0.001).sleep();
    this->updateSubscription();
  }

  this->callback(event, this->pub);
}

ros::AdvertiseOptions cras::GenericLazyPubSub::createAdvertiseOptions(
  const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
  const auto& msg = event.getConstMessage();
  auto cb = boost::bind(&cras::GenericLazyPubSub::connectCb, this, boost::placeholders::_1);
  ros::AdvertiseOptions opts(this->topicOut, this->outQueueSize, msg->getMD5Sum(), msg->getDataType(),
    msg->getMessageDefinition(), cb, cb);
  opts.has_header = cras::hasHeader(*msg);
  if (event.getConnectionHeaderPtr() != nullptr)
  {
    const auto header = event.getConnectionHeader();
    opts.latch = header.find("latching") != header.end() && event.getConnectionHeader()["latching"] == "1";
  }
  return opts;
}

cras::GenericLazyPubSub::GenericLazyPubSub(const ros::NodeHandle& nhIn, const std::string& topicIn,
  const ros::NodeHandle& nhOut, const std::string& topicOut, size_t queueSize,
  cras::GenericLazyPubSub::CallbackType callback, ros::SubscribeOptions subscribeOptions,
  const cras::LogHelperPtr& logHelper) :
    GenericLazyPubSub(
      nhIn, topicIn, nhOut, topicOut, queueSize, queueSize, std::move(callback), std::move(subscribeOptions), logHelper)
{
}

cras::GenericLazyPubSub::GenericLazyPubSub(const ros::NodeHandle& nhIn, const std::string& topicIn,
  const ros::NodeHandle& nhOut, const std::string& topicOut, cras::GenericLazyPubSub::CallbackType callback,
  ros::SubscribeOptions subscribeOptions, const cras::LogHelperPtr& logHelper) :
    GenericLazyPubSub(nhIn, topicIn, nhOut, topicOut, 10, std::move(callback), std::move(subscribeOptions), logHelper)
{
}

cras::GenericLazyPubSub::GenericLazyPubSub(const ros::NodeHandle& nh, const std::string& topicIn,
  const std::string& topicOut, size_t inQueueSize, size_t outQueueSize, cras::GenericLazyPubSub::CallbackType callback,
  ros::SubscribeOptions subscribeOptions, const cras::LogHelperPtr& logHelper) :
    GenericLazyPubSub(
      nh, topicIn, nh, topicOut, inQueueSize, outQueueSize, std::move(callback), std::move(subscribeOptions), logHelper)
{
}

cras::GenericLazyPubSub::GenericLazyPubSub(const ros::NodeHandle& nh, const std::string& topicIn,
  const std::string& topicOut, size_t queueSize, cras::GenericLazyPubSub::CallbackType callback,
  ros::SubscribeOptions subscribeOptions, const cras::LogHelperPtr& logHelper) :
    GenericLazyPubSub(
      nh, topicIn, topicOut, queueSize, queueSize, std::move(callback), std::move(subscribeOptions), logHelper)
{
}

cras::GenericLazyPubSub::GenericLazyPubSub(const ros::NodeHandle& nh, const std::string& topicIn,
  const std::string& topicOut, cras::GenericLazyPubSub::CallbackType callback,
  ros::SubscribeOptions subscribeOptions, const cras::LogHelperPtr& logHelper) :
    GenericLazyPubSub(nh, topicIn, topicOut, 10, std::move(callback), std::move(subscribeOptions), logHelper)
{
}

