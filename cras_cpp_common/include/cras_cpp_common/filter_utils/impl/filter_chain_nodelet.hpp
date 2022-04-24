#pragma once

/**
 * \file
 * \brief A versatile nodelet that can load and run a filter chain.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/thread/recursive_mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <ros/advertise_options.h>
#include <ros/duration.h>
#include <ros/message_traits.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscribe_options.h>
#include <ros/time.h>

#include <cras_cpp_common/FilterChainConfig.h>

#include <cras_cpp_common/filter_utils/filter_chain_nodelet.hpp>
#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

template <class F>
FilterChainNodelet<F>::FilterChainNodelet(const ::std::string& dataType,
  const ::std::string& topicIn, const ::std::string& topicFiltered, const ::std::string& configNamespace) :
    filterChain(dataType,
      [this](const F& data, const size_t filterNum, const ::std::string& name, const ::std::string& type)
        { this->filterCallback(data, filterNum, name, type); }),
    topicIn(topicIn), topicFiltered(topicFiltered), configNamespace(configNamespace)
{
}

template <class F>
FilterChainNodelet<F>::FilterChainNodelet(const ::std::string& topicIn, const ::std::string& topicFiltered,
  const ::std::string& configNamespace) :
    FilterChainNodelet<F>(::cras::replace(MsgDataType::value(), "/", "::"), topicIn, topicFiltered, configNamespace)
{
}

template <class F>
FilterChainNodelet<F>::FilterChainNodelet(const ::std::string& configNamespace) :
  FilterChainNodelet<F>("in", "out", configNamespace)
{
}

template <class F>
FilterChainNodelet<F>::~FilterChainNodelet()
{
}

template <class F>
void FilterChainNodelet<F>::enableFilterPublishers()
{
  ::boost::lock_guard<::boost::recursive_mutex> lock(this->configMutex);
  const auto changed = !this->publishFilters;
  this->publishFilters = true;
  if (changed)
    this->updateDynamicParams();
}

template <class F>
void FilterChainNodelet<F>::disableFilterPublishers()
{
  if (!this->publishFilters)
    return;

  ::boost::lock_guard<::boost::recursive_mutex> lock(this->configMutex);
  {
    ::std::lock_guard<::std::mutex> l(this->filterPublishersMutex);

    this->publishFilters = false;
    for (auto& pub: this->filterPublishers)
    {
      pub.second.shutdown();
    }

    this->filterPublishers.clear();
  }
  this->updateDynamicParams();
}

template <class F>
void FilterChainNodelet<F>::setDisabledFilters(::std::unordered_set<::std::string> filters)
{
  ::boost::lock_guard<::boost::recursive_mutex> lock(this->configMutex);
  this->filterChain.setDisabledFilters(::std::move(filters));
  this->updateDynamicParams();
}

template <class F>
void FilterChainNodelet<F>::setMaxAge(const ::ros::Duration& maxAge)
{
  ::boost::lock_guard<::boost::recursive_mutex> lock(this->configMutex);
  this->maxAge = maxAge;
  this->updateDynamicParams();
}

template <class F>
void FilterChainNodelet<F>::dynreconfCallback(::cras_cpp_common::FilterChainConfig& config, const uint32_t)
{
  if (config.publish_each_filter)
    this->enableFilterPublishers();
  else
    this->disableFilterPublishers();

  this->maxAge = ::ros::Duration(config.max_age);

  ::std::vector<::std::string> disabledFilters;
  if (!config.disabled_filters.empty())
    disabledFilters = ::cras::split(config.disabled_filters, ",");
  this->filterChain.setDisabledFilters({disabledFilters.begin(), disabledFilters.end()});
}

template <class F>
void FilterChainNodelet<F>::updateDynamicParams() const
{
  ::boost::lock_guard<::boost::recursive_mutex> lock(this->configMutex);

  ::cras_cpp_common::FilterChainConfig config;
  config.publish_each_filter = this->publishFilters;
  config.max_age = this->maxAge.toSec();
  auto disabledFilters = this->filterChain.getDisabledFilters();
  config.disabled_filters = "";
  
  size_t i = 0;
  for (const auto& filter : disabledFilters)
  {
    config.disabled_filters += filter;
    if (i != disabledFilters.size() - 1)
      config.disabled_filters += ",";
    i++;
  }

  this->dynreconfServer->updateConfig(config);
}

template <class F>
void FilterChainNodelet<F>::onInit()
{
  this->nodeHandle = this->getNodeHandle();
  this->privateNodeHandle = this->getPrivateNodeHandle();

  this->dynreconfServer = ::std::make_unique<::dynamic_reconfigure::Server<::cras_cpp_common::FilterChainConfig>>(
    this->configMutex, this->privateNodeHandle);
  this->dynreconfServer->setCallback([this](auto& config, auto level){ this->dynreconfCallback(config, level); });

  const auto privateParams = this->privateParams();
  this->lazySubscription = privateParams->getParam("lazy_subscription", true);
  this->subscriberQueueSize = privateParams->getParam("subscriber_queue_size", this->subscriberQueueSize);
  this->publisherQueueSize = privateParams->getParam("publisher_queue_size", this->publisherQueueSize);
  this->publishDiagnostics = privateParams->getParam("publish_diagnostics", this->publishDiagnostics);

  // setup filters
  this->filterChain.configure(this->configNamespace, this->privateNodeHandle);
  this->filterChain.setNodelet(this);

  // filtered data publisher
  ::ros::AdvertiseOptions opts;
  opts.template init<F>(this->topicFiltered, this->publisherQueueSize,
    [this](const auto& pub){ this->connectCb(pub); }, [this](const auto& pub){ this->disconnectCb(pub); });
  
  if (this->publishDiagnostics)
  {
    this->publisherDiag = this->template advertiseDiagnosed<F>(opts);
    this->filteredPublisher = this->publisherDiag->getPublisher();
  }
  else
  {
    this->filteredPublisher = this->nodeHandle.advertise(opts); 
  }

  if (!this->lazySubscription)
    this->subscribe();

  if (this->publishDiagnostics)
    this->startDiagTimer();
}

template <class F>
void FilterChainNodelet<F>::subscribe()
{
  NODELET_DEBUG("Connecting to input topic.");
  ::ros::SubscribeOptions opts;
  opts.template init<F>(this->topicIn, this->subscriberQueueSize, [this](const auto& data){ dataCallback(data); });
  
  if (this->publishDiagnostics)
  {
    this->subscriberDiag = this->template subscribeDiagnosed<F>(opts);
    this->subscriber = this->subscriberDiag->getSubscriber();
  }
  else
  {
    this->subscriber = this->nodeHandle.subscribe(opts);
  }
}

template <class F>
void FilterChainNodelet<F>::connectCb(const ::ros::SingleSubscriberPublisher&)
{
  if (!this->lazySubscription)
    return;

  ::std::lock_guard<::std::mutex> l(this->connectMutex);
  if (!this->subscriber && this->filteredPublisher.getNumSubscribers() > 0)
    this->subscribe();
}

template <class F>
void FilterChainNodelet<F>::disconnectCb(const ::ros::SingleSubscriberPublisher&)
{
  if (!this->lazySubscription)
    return;

  ::std::lock_guard<::std::mutex> l(this->connectMutex);
  if (this->filteredPublisher.getNumSubscribers() == 0)
  {
    NODELET_DEBUG("Unsubscribing from input topic.");
    if (this->subscriberDiag != nullptr)
    {
      this->getDiagUpdater().removeByName(this->subscriberDiag->getDiagnosticTask()->getName());
      this->subscriberDiag.reset();
    }
    this->subscriber.shutdown();
  }
}

template <class F>
template <typename T, ::std::enable_if_t<::ros::message_traits::HasHeader<T>::value, bool>>
::ros::Time FilterChainNodelet<F>::getStamp(const typename T::ConstPtr& data)
{
  return data->header.stamp;
}

template <class F>
template <typename T, ::std::enable_if_t<!::ros::message_traits::HasHeader<T>::value, bool>>
::ros::Time FilterChainNodelet<F>::getStamp(const typename T::ConstPtr&)
{
  return ::ros::Time::now();
}

template <class F>
void FilterChainNodelet<F>::dataCallback(const typename F::ConstPtr& data)
{
  const auto stamp = this->template getStamp<F>(data);
  const auto age = ::ros::Time::now() - stamp;
  if (age > this->maxAge)
  {
    NODELET_WARN("Throwing away old data from time %s (age %s s, limit is %s s)",
                 ::cras::to_string(stamp).c_str(),
                 ::cras::to_string(age).c_str(),
                 ::cras::to_string(this->maxAge).c_str());
    return;
  }
  
  const auto stopwatchOverall = ::ros::WallTime::now();

  this->updateThreadName();

  typename F::Ptr dataFiltered(new F);

  // apply the filter chain
  if (this->filterChain.update(*data, *dataFiltered))
  {
    // publish the filtered result
    if (this->publisherDiag != nullptr)
      this->publisherDiag->publish(dataFiltered);
    else
      this->filteredPublisher.publish(dataFiltered);
  }
  else
  {
    NODELET_ERROR_DELAYED_THROTTLE(3, "Filtering data from time %s failed.", ::cras::to_string(stamp).c_str());
  }

  NODELET_DEBUG("Data filtered in %.5f secs.", (::ros::WallTime::now() - stopwatchOverall).toSec());
}

template <class F>
void FilterChainNodelet<F>::filterCallback(const F& data, const size_t filterNum,
  const ::std::string& name, const ::std::string& type)
{
  if (!this->publishFilters)
    return;
  
  ::std::lock_guard<::std::mutex> l(this->filterPublishersMutex);
  if (this->filterPublishers.find(name) == this->filterPublishers.end())
  {
    const auto topicName = ::cras::format("filter%i/%s", filterNum, name.c_str());
    this->filterPublishers[name] = this->privateNodeHandle.template advertise<F>(topicName, this->publisherQueueSize);
    ::ros::WallDuration(0.001).sleep();
  }

  this->filterPublishers[name].publish(data);
}

}
