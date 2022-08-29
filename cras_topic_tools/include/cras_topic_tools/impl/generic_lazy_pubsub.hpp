#pragma once

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers. Version for unknown message
 *        type of both the subscriber and the publisher (implementation details, do not include directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <ros/advertise_options.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/message_event.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/log_utils.h>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/lazy_subscriber.hpp>

template<typename SubscriberType>
::cras::GenericLazyPubSub<SubscriberType>::GenericLazyPubSub(
  const ::std::string& topicIn, const ::std::string& topicOut, const ::ros::NodeHandle& nh,
  const size_t inQueueSize, const size_t outQueueSize, ::cras::LogHelperPtr logHelper) :
    ::cras::LazySubscriberBase<SubscriberType>(
      ::cras::bind_front(&::cras::GenericLazyPubSub<SubscriberType>::subscribe, this),
      [](SubscriberType& sub) { sub.shutdown(); },
      logHelper),
    topicIn(topicIn), topicOut(topicOut), inQueueSize(inQueueSize), outQueueSize(outQueueSize), nh(nh)
{
  // We have to connect at the beginning so that we can create the publisher from a subscribed message.
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);
  this->connectNoLock();
}

template<typename SubscriberType>
void ::cras::GenericLazyPubSub<SubscriberType>::subscribe(SubscriberType& sub)
{
  ::ros::SubscribeOptions opts;
  opts.template initByFullCallbackType<const ::ros::MessageEvent<::topic_tools::ShapeShifter const>&>(
    this->topicIn, this->inQueueSize, ::boost::bind(&::cras::GenericLazyPubSub<SubscriberType>::cb, this, _1));
  sub = this->nh.subscribe(opts);
}

template<typename SubscriberType>
bool ::cras::GenericLazyPubSub<SubscriberType>::shouldBeSubscribed() const
{
  if (!this->pub)
    return true;
  return this->pub.getNumSubscribers() > 0;
}

template<typename SubscriberType>
void ::cras::GenericLazyPubSub<SubscriberType>::connectCb(const ::ros::SingleSubscriberPublisher&)
{
  if (!this->lazy)
    return;
  this->updateSubscription();
}

template<typename SubscriberType>
void ::cras::GenericLazyPubSub<SubscriberType>::cb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event)
{
  const auto& msg = event.getConstMessage();
  if (!this->pub)
  {
    ::std::lock_guard<::std::mutex> pubLock(this->pubCreateMutex);
    if (!this->pub)  // the first check is outside mutex, this one is inside
    {
      this->advertiseOptions = this->createAdvertiseOptions(event);  // cannot be const - advertise requires non-const
      this->logHelper->logInfo("Creating%s publisher on %s with type %s.",
        (this->advertiseOptions->latch ? " latched" : ""),
        ::ros::names::resolve(this->nh.getNamespace(), this->topicOut).c_str(),
        msg->getDataType().c_str());

      {  // Advertise the publisher
        ::std::lock_guard<::std::mutex> lock(this->connectMutex);
        this->pub = this->nh.advertise(this->advertiseOptions.value());
      }
    }

    for (size_t i = 0; i < 100 && ::ros::ok() && this->pub.getNumSubscribers() == 0; ++i)
      ::ros::WallDuration(0.001).sleep();
    this->updateSubscription();
  }

  this->processMessage(event);
}

template<typename SubscriberType>
::ros::AdvertiseOptions cras::GenericLazyPubSub<SubscriberType>::createAdvertiseOptions(
  const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event)
{
  const auto& msg = event.getConstMessage();
  auto cb = ::boost::bind(&::cras::GenericLazyPubSub<SubscriberType>::connectCb, this, ::boost::placeholders::_1);
  ::ros::AdvertiseOptions opts(this->topicOut, this->outQueueSize, msg->getMD5Sum(), msg->getDataType(),
    msg->getMessageDefinition(), cb, cb);
  // This is a bit simplified but there is no really good way to detect it (cannot use rosmsg_cpp as the embedded Python
  // cannot be run in multiple nodelets; cannot use ros_msg_parse as it has not yet been released).
  // TODO: Consider using ros_msg_parse when it is released
  opts.has_header = ::cras::contains(msg->getMessageDefinition(), "Header header");
  if (event.getConnectionHeaderPtr() != nullptr)
  {
    const auto header = event.getConnectionHeader();
    opts.latch = header.find("latching") != header.end() && event.getConnectionHeader()["latching"] == "1";
  }
  return opts;
}

template<typename SubscriberType>
void ::cras::GenericLazyPubSub<SubscriberType>::processMessage(
  const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event)
{
  this->pub.publish(event.getConstMessage());
}
