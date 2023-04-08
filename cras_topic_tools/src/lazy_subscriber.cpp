// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers (implementation details, do not
 *        include directly).
 * \author Martin Pecka
 */

#include <mutex>
#include <utility>

#include <ros/subscriber.h>

#include <cras_cpp_common/log_utils.h>

#include <cras_topic_tools/lazy_subscriber.hpp>

cras::ConditionalSubscriber::ConditionalSubscriber(
  ConnectFn connectFn, DisconnectFn disconnectFn, const cras::LogHelperPtr& logHelper) :
    cras::HasLogger(logHelper), connectFn(std::move(connectFn)), disconnectFn(std::move(disconnectFn))
{
}

cras::ConditionalSubscriber::ConditionalSubscriber(
  ConnectFn connectFn, const cras::LogHelperPtr& logHelper) :
    ConditionalSubscriber(std::move(connectFn), [](ros::Subscriber& sub) { sub.shutdown(); }, logHelper)
{
}

cras::ConditionalSubscriber::~ConditionalSubscriber()
{
  std::lock_guard<std::mutex> lock(this->connectMutex);
  if (this->subscribed)
    this->disconnectNoLock();
}

bool cras::ConditionalSubscriber::isLazy() const
{
  return this->lazy;
}

void cras::ConditionalSubscriber::setLazy(const bool lazy)
{
  std::lock_guard<std::mutex> lock(this->connectMutex);

  if (lazy == this->lazy)
    return;

  this->lazy = lazy;

  if (lazy)
    CRAS_DEBUG("Switching to lazy subscription mode");
  else
    CRAS_DEBUG("Switching to non-lazy subscription mode");

  this->updateSubscriptionNoLock();
}

bool cras::ConditionalSubscriber::isSubscribed() const
{
  std::lock_guard<std::mutex> lock(this->connectMutex);
  return this->subscribed;
}

void cras::ConditionalSubscriber::updateSubscription()
{
  std::lock_guard<std::mutex> lock(this->connectMutex);
  this->updateSubscriptionNoLock();
}

void cras::ConditionalSubscriber::updateSubscriptionNoLock()
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

void cras::ConditionalSubscriber::connectNoLock()
{
  this->connectFn(this->sub);
  this->subscribed = true;
  CRAS_DEBUG("Connected to topic " + this->sub.getTopic());
}

void cras::ConditionalSubscriber::disconnectNoLock()
{
  CRAS_DEBUG("Disconnecting from topic " + this->sub.getTopic());
  this->disconnectFn(this->sub);
  this->subscribed = false;
}
