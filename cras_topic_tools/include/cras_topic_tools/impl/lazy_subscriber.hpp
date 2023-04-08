#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers (implementation details, do not
 *        include directly).
 * \author Martin Pecka
 */

#include <mutex>
#include <string>
#include <utility>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/functional.hpp>

#include <ros/advertise_options.h>
#include <ros/node_handle.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>

#include <cras_cpp_common/log_utils.h>

#include <cras_topic_tools/lazy_subscriber.hpp>

template<typename PublisherMsgType>
::cras::LazySubscriberBase<PublisherMsgType>::LazySubscriberBase(
  ::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
  typename ::cras::ConditionalSubscriber::ConnectFn connectFn,
  typename ::cras::ConditionalSubscriber::DisconnectFn disconnectFn,
  const ::cras::LogHelperPtr& logHelper) :
    ConditionalSubscriber(::std::move(connectFn), ::std::move(disconnectFn), logHelper)
{
  ::ros::AdvertiseOptions opts;
  auto cb = ::boost::bind(&LazySubscriberBase<PublisherMsgType>::connectCb, this, ::boost::placeholders::_1);
  opts.template init<PublisherMsgType>(publisherTopic, 1, cb, cb);

  // Need to create the publisher with connection mutex - connectCB can be called before the publisher is created
  // in nodelet, which means no topics will connect.
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);
  this->pub = publisherNh.advertise(opts);
}

template<typename PublisherMsgType>
::cras::LazySubscriberBase<PublisherMsgType>::LazySubscriberBase(
  ::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
  typename ::cras::ConditionalSubscriber::ConnectFn connectFn, const ::cras::LogHelperPtr& logHelper) :
    LazySubscriberBase(::std::move(publisherNh), publisherTopic, ::std::move(connectFn),
      [](::ros::Subscriber& sub) { sub.shutdown(); }, logHelper)
{
}

template<typename PublisherMsgType>
void ::cras::LazySubscriberBase<PublisherMsgType>::connectCb(const ::ros::SingleSubscriberPublisher&)
{
  if (!this->lazy)
    return;

  this->updateSubscription();
}

template<typename PublisherMsgType>
bool ::cras::LazySubscriberBase<PublisherMsgType>::shouldBeSubscribed() const
{
  return this->pub && this->pub.getNumSubscribers() > 0;
}

template<typename PublisherMsgType, typename CallbackType>
::cras::LazySubscriber<PublisherMsgType, CallbackType>::LazySubscriber(
  ::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
  ::ros::NodeHandle subscriberNh, const ::std::string& subscriberTopic, const size_t subscriberQueueSize,
  ::boost::function<void(CallbackType)> subscriberCallback, ::ros::SubscribeOptions subscribeOptions,
  const ::cras::LogHelperPtr& logHelper) :
  LazySubscriberBase<PublisherMsgType>(::std::move(publisherNh), publisherTopic,
    [=](::ros::Subscriber& sub) mutable
    {
      subscribeOptions.template initByFullCallbackType<CallbackType>(
        subscriberTopic, subscriberQueueSize, ::std::move(subscriberCallback));
      sub = subscriberNh.subscribe(subscribeOptions);
    },
    logHelper)
{
}

template<typename PublisherMsgType, typename CallbackType>
::cras::LazySubscriber<PublisherMsgType, CallbackType>::LazySubscriber(
  ::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
  ::ros::NodeHandle subscriberNh, const ::std::string& subscriberTopic, const size_t subscriberQueueSize,
  ::boost::function<void(CallbackType)> subscriberCallback, const ::cras::LogHelperPtr& logHelper) :
    LazySubscriber<PublisherMsgType, CallbackType>(::std::move(publisherNh), publisherTopic, ::std::move(subscriberNh),
      subscriberTopic, subscriberQueueSize, subscriberCallback, {}, logHelper)
{
}
