#pragma once

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers (implementation details, do not
 *        include directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <ros/advertise_options.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>

#include <cras_cpp_common/log_utils.h>

#include <cras_topic_tools/lazy_subscriber.hpp>

template<typename SubscriberType>
::cras::LazySubscriberBase<SubscriberType>::LazySubscriberBase(
  const ConnectFn& connectFn, const DisconnectFn& disconnectFn, ::cras::LogHelperPtr logHelper) :
    connectFn(connectFn), disconnectFn(disconnectFn), logHelper(::std::move(logHelper))
{
}

template<typename SubscriberType>
::cras::LazySubscriberBase<SubscriberType>::~LazySubscriberBase()
{
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);
  if (this->subscribed)
    this->disconnectNoLock();
}

template<typename SubscriberType>
bool ::cras::LazySubscriberBase<SubscriberType>::isLazy() const
{
  return this->lazy;
}

template<typename SubscriberType>
void ::cras::LazySubscriberBase<SubscriberType>::setLazy(const bool lazy)
{
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);

  if (lazy == this->lazy)
    return;

  this->lazy = lazy;

  if (lazy)
    CRAS_LOG_DEBUG(this->logHelper, "Switching to lazy subscription mode");
  else
    CRAS_LOG_DEBUG(this->logHelper, "Switching to non-lazy subscription mode");

  this->updateSubscriptionNoLock();
}

template<typename SubscriberType>
bool ::cras::LazySubscriberBase<SubscriberType>::isSubscribed() const
{
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);
  return this->subscribed;
}

template<typename SubscriberType>
void ::cras::LazySubscriberBase<SubscriberType>::updateSubscription()
{
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);
  this->updateSubscriptionNoLock();
}

template<typename SubscriberType>
void ::cras::LazySubscriberBase<SubscriberType>::updateSubscriptionNoLock()
{
  const auto shouldBeSubscribed = !this->lazy || this->shouldBeSubscribed();
  if (this->subscribed && !shouldBeSubscribed)
  {
    this->disconnectNoLock();
  }
  else if (!this->subscribed && shouldBeSubscribed)
  {
    this->connectNoLock();
  }
}

template<typename SubscriberType>
void ::cras::LazySubscriberBase<SubscriberType>::connectNoLock()
{
  this->connectFn(this->sub);
  this->subscribed = true;
  this->logHelper->logDebug("Connected to topic " + this->sub.getTopic());
}

template<typename SubscriberType>
void ::cras::LazySubscriberBase<SubscriberType>::disconnectNoLock()
{
  this->logHelper->logDebug("Disconnecting from topic " + this->sub.getTopic());
  this->disconnectFn(this->sub);
  this->subscribed = false;
}

template<typename M, typename SubscriberType>
::cras::LazySubscriber<M, SubscriberType>::LazySubscriber(const ::ros::Publisher& pub,
  const typename ::cras::LazySubscriberBase<SubscriberType>::ConnectFn& connectFn,
  const typename ::cras::LazySubscriberBase<SubscriberType>::DisconnectFn& disconnectFn,
  ::cras::LogHelperPtr logHelper) :
    LazySubscriberBase<SubscriberType>(connectFn, disconnectFn, ::std::move(logHelper))
{
  ::ros::AdvertiseOptions opts;
  auto cb = ::boost::bind(&LazySubscriber<M>::connectCb, this, ::boost::placeholders::_1);
  opts.template init<M>(pub.getTopic(), 10, cb, cb);

  const auto ns = ::ros::names::parentNamespace(pub.getTopic());
  ::ros::NodeHandle nh(ns);

  // Need to create the publisher with connection mutex - connectCB can be called before the publisher is created
  // in nodelet, which means no topics will connect.
  ::std::lock_guard<::std::mutex> lock(this->connectMutex);
  this->pub = nh.advertise(opts);
}

template<typename M, typename SubscriberType>
void ::cras::LazySubscriber<M, SubscriberType>::connectCb(const ::ros::SingleSubscriberPublisher&)
{
  if (!this->lazy)
    return;

  this->updateSubscription();
}

template<typename M, typename SubscriberType>
bool ::cras::LazySubscriber<M, SubscriberType>::shouldBeSubscribed() const
{
  return this->pub && this->pub.getNumSubscribers() > 0;
}
