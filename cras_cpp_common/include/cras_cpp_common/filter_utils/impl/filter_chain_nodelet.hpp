#pragma once

/**
 * \file
 * \brief A versatile nodelet that can load and run a filter chain  (implementation details, do not include directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/thread/recursive_mutex.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <dynamic_reconfigure/server.h>
#include <ros/advertise_options.h>
#include <ros/duration.h>
#include <ros/message_traits.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscribe_options.h>
#include <ros/time.h>

#include <cras_cpp_common/FilterChainConfig.h>

#include <cras_cpp_common/diag_utils/duration_status.h>
#include <cras_cpp_common/diag_utils/duration_status_param.h>
#include <cras_cpp_common/filter_utils/filter_chain.hpp>
#include <cras_cpp_common/filter_utils/filter_chain_nodelet.hpp>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

template <class F>
FilterChainNodelet<F>::FilterChainNodelet(const ::std::string& dataType,
  const ::std::string& topicIn, const ::std::string& topicFiltered, const ::std::string& configNamespace) :
    filterChain(dataType, ::cras::bind_front(&FilterChainNodelet::filterFinishedCallback, this),
      ::cras::bind_front(&FilterChainNodelet::filterStartCallback, this), this->log),
    topicIn(topicIn), topicFiltered(topicFiltered), configNamespace(configNamespace)
{
}

template <class F>
FilterChainNodelet<F>::FilterChainNodelet(const ::std::string& topicIn, const ::std::string& topicFiltered,
  const ::std::string& configNamespace) :
    FilterChainNodelet(::cras::replace(MsgDataType::value(), "/", "::"), topicIn, topicFiltered, configNamespace)
{
}

template <class F>
FilterChainNodelet<F>::FilterChainNodelet(const ::std::string& configNamespace) :
  FilterChainNodelet("in", "out", configNamespace)
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
  this->dynreconfServer->setCallback(::cras::bind_front(&FilterChainNodelet::dynreconfCallback, this));

  const auto privateParams = this->privateParams();
  this->lazySubscription = privateParams->getParam("lazy_subscription", true);
  this->subscriberQueueSize = privateParams->getParam("subscriber_queue_size", this->subscriberQueueSize);
  this->publisherQueueSize = privateParams->getParam("publisher_queue_size", this->publisherQueueSize);
  const auto publishDiagnostics = privateParams->getParam("publish_diagnostics", false);
  this->publishTopicDiagnostics = privateParams->getParam("publish_topic_diagnostics", publishDiagnostics);
  this->publishDurationDiagnostics = privateParams->getParam("publish_duration_diagnostics", publishDiagnostics);
  this->publishChainDiagnostics = privateParams->getParam("publish_chain_diagnostics", publishDiagnostics);

  // setup filters
  this->filterChain.configure(this->configNamespace, this->privateNodeHandle);
  this->filterChain.setNodelet(this);

  // filtered data publisher
  ::ros::AdvertiseOptions opts;
  opts.template init<F>(this->topicFiltered, this->publisherQueueSize,
    ::cras::bind_front(&FilterChainNodelet::connectCb, this),
    ::cras::bind_front(&FilterChainNodelet::disconnectCb, this));
  
  if (this->publishTopicDiagnostics)
  {
    this->publisherDiag = this->template advertiseDiagnosed<F>(opts);
    this->filteredPublisher = this->publisherDiag->getPublisher();
  }
  else
  {
    this->filteredPublisher = this->nodeHandle.advertise(opts); 
  }

  if (this->publishDurationDiagnostics)
  {
    this->callbackDurationDiag = ::std::make_unique<::cras::DurationStatus>("All filters callback duration",
      privateParams->paramsInNamespace("update_duration"), ::cras::SimpleDurationStatusParam());
    this->getDiagUpdater().add(*this->callbackDurationDiag);
    
    const auto& constFilterChain = this->filterChain;
    for (const auto& filter : constFilterChain.getFilters())
    {
      const auto& name = filter->getName();
      this->filterCallbackDurationDiags[name] = ::std::make_unique<::cras::DurationStatus>(name + " callback duration",
        privateParams->paramsInNamespace(name + "/update_duration"), ::cras::SimpleDurationStatusParam());
      this->getDiagUpdater().add(*this->filterCallbackDurationDiags[name]);
    }
  }
  
  if (this->publishChainDiagnostics)
  {
    this->chainDiag = ::std::make_unique<::cras::FilterChainDiagnostics<F>>(
      "Filter chain performance", this->filterChain);
    this->getDiagUpdater().add(*this->chainDiag);
  }

  if (!this->lazySubscription)
    this->subscribe();

  if (this->publishTopicDiagnostics || this->publishDurationDiagnostics || this->publishChainDiagnostics)
    this->startDiagTimer();
}

template <class F>
void FilterChainNodelet<F>::subscribe()
{
  NODELET_DEBUG("Connecting to input topic.");
  ::ros::SubscribeOptions opts;
  opts.template init<F>(this->topicIn, this->subscriberQueueSize,
    ::cras::bind_front(&FilterChainNodelet::dataCallback, this));
  
  if (this->publishTopicDiagnostics)
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
  
  if (this->publishDurationDiagnostics)
    this->callbackDurationDiag->start(::ros::WallTime::now());

  // apply the filter chain
  if (this->filterChain.update(*data, *dataFiltered))
  {
    if (this->publishDurationDiagnostics)
      this->callbackDurationDiag->stop(::ros::WallTime::now());
    
    // publish the filtered result
    if (this->publisherDiag != nullptr)
      this->publisherDiag->publish(dataFiltered);
    else
      this->filteredPublisher.publish(dataFiltered);
  }
  else
  {
    if (this->publishDurationDiagnostics)
      this->callbackDurationDiag->stop(::ros::WallTime::now());
    NODELET_ERROR_DELAYED_THROTTLE(3, "Filtering data from time %s failed.", ::cras::to_string(stamp).c_str());
  }

  NODELET_DEBUG("Data filtered in %.5f secs.", (::ros::WallTime::now() - stopwatchOverall).toSec());
}

template <class F>
void FilterChainNodelet<F>::filterFinishedCallback(const F& data, const size_t filterNum,
  const ::std::string& name, const ::std::string& type, const bool success)
{
  if (this->publishDurationDiagnostics)
  {
    if (this->filterCallbackDurationDiags.find(name) != this->filterCallbackDurationDiags.end())
    {
      this->filterCallbackDurationDiags[name]->stop(::ros::WallTime::now());
    }
  }
  
  if (this->publishChainDiagnostics)
    this->chainDiag->addReport(name, success);

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

template <class F>
void FilterChainNodelet<F>::filterStartCallback(const F& data, const size_t filterNum,
  const ::std::string& name, const ::std::string& type)
{
  if (!this->publishDurationDiagnostics)
    return;

  if (this->filterCallbackDurationDiags.find(name) != this->filterCallbackDurationDiags.end())
  {
    this->filterCallbackDurationDiags[name]->start(::ros::WallTime::now());
  }
}

template<typename F>
FilterChainDiagnostics<F>::FilterChainDiagnostics(const ::std::string& name, const ::cras::FilterChain<F>& chain) :
  ::diagnostic_updater::DiagnosticTask(name), chain(chain)
{
}

template<typename F>
void FilterChainDiagnostics<F>::run(::diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  ::std::lock_guard<::std::mutex> lock(this->mutex);

  size_t totalCallbacks = 0;
  size_t totalFailures = 0;

  for (const auto& nameAndVal : this->numCallbacks)
    totalCallbacks += nameAndVal.second;

  for (const auto& nameAndVal : this->numFailures)
    totalFailures += nameAndVal.second;

  if (totalFailures == 0)
  {
    stat.level = ::diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = ::cras::format("Filters running ok (processed %lu messages since last update)", totalCallbacks);
  }
  else
  {
    stat.level = ::diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = ::cras::format("Some filters are failing (processed %lu messages since last update, %ul errors "
                                  "encountered)", totalCallbacks, totalFailures);
  }

  auto allFilters = this->chain.getFilters();
  auto disabledFilters = this->chain.getDisabledFilters();
  for (const auto& filter : allFilters)
  {
    const auto& name = filter->getName();
    if (disabledFilters.find(name) == disabledFilters.end())
    {
      stat.addf("Filter " + name, "Active, processed %lu callbacks, %lu successful, %lu failures.",
                this->numCallbacks[name], this->numSuccesses[name], this->numFailures[name]);
    }
    else
    {
      stat.add("Filter " + name, "Disabled");
    }
  }

  this->numCallbacks.clear();
  this->numSuccesses.clear();
  this->numFailures.clear();
}

template<typename F>
void FilterChainDiagnostics<F>::addReport(const ::std::string& filterName, const bool success)
{
  ::std::lock_guard<::std::mutex> lock(this->mutex);
  this->numCallbacks[filterName]++;
  if (success)
    this->numSuccesses[filterName]++;
  else
    this->numFailures[filterName]++;
}

}
