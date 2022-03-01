#pragma once

#include <functional>
#include <mutex>

#include <ros/ros.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/filter_utils/filter_chain.hpp>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <nodelet/nodelet.h>
#include <utility>

#include <cras_cpp_common/FilterChainConfig.h>
#include <dynamic_reconfigure/server.h>

namespace cras {

template<class F>
class FilterChainNodelet : public cras::Nodelet
{

public:
  //! Read ROS parameters, initialize publishers/subscribers, initialize other class members. ROS::init() is assumed to have been called before.
  FilterChainNodelet(const std::string& dataType, std::string topicIn,
      std::string topicFiltered, std::string configNamespace)
      : filterChain(dataType, [this](const F& data, const size_t filterNum, const std::string& name, const std::string& type) {this->filterCallback(data, filterNum, name, type);}),
      topicIn(std::move(topicIn)), topicFiltered(std::move(topicFiltered)),
      configNamespace(std::move(configNamespace)) {
  }

  ~FilterChainNodelet() override {};

  void enableFilterPublishers()
  {
    this->publishFilters = true;
  }

  void disableFilterPublishers()
  {
    if (!this->publishFilters) {
      return;
    } else {
      std::lock_guard<std::mutex>(this->filterPublishersMutex);

      this->publishFilters = false;
      for (auto &pub : this->filterPublishers)
        pub.second.shutdown();

      this->filterPublishers.clear();
    }
  }

protected:

  //! public NodeHandle
  ros::NodeHandle nodeHandle;

  //! private NodeHandle
  ros::NodeHandle privateNodeHandle;

  //! Subscriber to data
  ros::Subscriber subscriber;
  std::shared_ptr<cras::TopicStatus<F>> subscriberDiag;

  //! The chain of filters to apply to the incoming scans.
  cras::FilterChain<F> filterChain;

  //! Publisher for the filtered data
  ros::Publisher filteredPublisher;
  std::unique_ptr<cras::DiagnosedPublisher<F>> publisherDiag;

  std::string topicIn, topicFiltered;
  std::string configNamespace;

  bool publishFilters = false;
  std::map<std::string, ros::Publisher> filterPublishers;
  std::mutex filterPublishersMutex;
  std::mutex connectMutex;

  bool lazySubscription = true;
  size_t subscriberQueueSize = 5_sz;
  bool publishDiagnostics = false;

  ros::Duration maxAge;

  std::unique_ptr<dynamic_reconfigure::Server<cras_cpp_common::FilterChainConfig> > dynreconfServer;

  void dynreconfCallback(cras_cpp_common::FilterChainConfig& config, uint32_t level)
  {
    if (config.publish_each_filter)
      this->enableFilterPublishers();
    else
      this->disableFilterPublishers();

    this->maxAge = ros::Duration().fromSec(config.max_age);

    std::unordered_set<std::string> disabledFilters;
    if (!config.disabled_filters.empty())
    {
      boost::split(disabledFilters, config.disabled_filters, boost::is_any_of(","));
    }
    this->filterChain.setDisabledFilters(disabledFilters);
  }

  void onInit() override
  {
    this->nodeHandle = this->getNodeHandle();
    this->privateNodeHandle = this->getPrivateNodeHandle();

    this->dynreconfServer = std::make_unique<dynamic_reconfigure::Server<cras_cpp_common::FilterChainConfig> >(this->privateNodeHandle);
    this->dynreconfServer->setCallback([this](auto& config, auto level){ this->dynreconfCallback(config, level); });

    const auto privateParams = this->paramsForNodeHandle(this->privateNodeHandle);
    this->lazySubscription = privateParams->getParam("lazy_subscription", true);
    this->subscriberQueueSize = privateParams->getParam("subscriber_queue_size", this->subscriberQueueSize);
    const auto publisherQueueSize = privateParams->getParam("publisher_queue_size", 15_sz);
    this->publishDiagnostics = privateParams->getParam("publish_diagnostics", this->publishDiagnostics);

    // setup filters
    this->filterChain.configure(this->configNamespace, this->privateNodeHandle);
    this->filterChain.setNodelet(this);

    // filtered data publisher
    this->filteredPublisher = nodeHandle.template advertise<F>(
        this->topicFiltered, publisherQueueSize,
        [this](const auto& pub){ this->connectCb(pub); },
        [this](const auto& pub){ this->disconnectCb(pub); });

    if (this->publishDiagnostics)
    {
      const auto publisherParams = privateParams->paramsInNamespace("publisher");
      this->publisherDiag = std::make_unique<cras::DiagnosedPublisher<F>>(
          this->filteredPublisher, this->getDiagUpdater(), publisherParams);

      const auto subscriberParams = privateParams->paramsInNamespace("subscriber");
      auto sub = cras::DiagnosedPubSub<F>(subscriberParams, {});
      sub.attach(this->getDiagUpdater());
      this->subscriberDiag = sub.getDiagnosticTask();
    }

    if (!this->lazySubscription) {
      this->subscribe();
    } else if (this->subscriberDiag != nullptr) {
      this->getDiagUpdater().removeByName(this->subscriberDiag->getName());
    }

    if (this->publishDiagnostics)
      this->startDiagTimer(this->privateNodeHandle);
  }

  void subscribe()
  {
    NODELET_DEBUG("Connecting to input topic.");
    this->subscriber = this->nodeHandle.template subscribe<F>(this->topicIn, this->subscriberQueueSize,
        [this](const auto& data){ dataCallback(data); });
  }

  virtual void connectCb(const ros::SingleSubscriberPublisher& pub)
  {
    if (this->lazySubscription)
    {
      std::lock_guard<std::mutex>(this->connectMutex);
      if (!this->subscriber && this->filteredPublisher.getNumSubscribers() > 0)
      {
        this->subscribe();
        if (this->subscriberDiag != nullptr)
          this->getDiagUpdater().add(*this->subscriberDiag);
      }
    }
  }

  virtual void disconnectCb(const ros::SingleSubscriberPublisher& pub)
  {
    if (this->lazySubscription)
    {
      std::lock_guard<std::mutex>(this->connectMutex);
      if (this->filteredPublisher.getNumSubscribers() == 0)
      {
        NODELET_DEBUG("Unsubscribing from input topic.");
        if (this->subscriberDiag != nullptr)
          this->getDiagUpdater().removeByName(this->subscriberDiag->getName());
        this->subscriber.shutdown();
      }
    }
  }

  //! Data callback function.
  virtual void dataCallback(const typename F::ConstPtr& data)
  {
    if (this->subscriberDiag != nullptr)
      this->subscriberDiag->tick(data);

    const auto age = ros::Time::now() - data->header.stamp;
    if (age > this->maxAge) {
      NODELET_WARN("Throwing away old scan from time %s (age %s s)",
          cras::to_string(data->header.stamp).c_str(),
          cras::to_string(age).c_str());
      return;
    }

    const clock_t stopwatchOverall = clock();

    this->updateThreadName();

    typename F::Ptr dataFiltered(new F);

    // apply the filter chain
    if (this->filterChain.update(*data, *dataFiltered)) {
      // publish the filtered result
      if (this->publisherDiag != nullptr)
        this->publisherDiag->publish(dataFiltered);
      else
        this->filteredPublisher.publish(dataFiltered);
    } else {
      NODELET_ERROR_DELAYED_THROTTLE(3, "Filtering data from time %s failed.", cras::to_string(data->header.stamp).c_str());
    }

    NODELET_DEBUG("Data filtered in %.5f secs.", double(clock()-stopwatchOverall) / CLOCKS_PER_SEC);
  }

  virtual void filterCallback(const F& data, const size_t filterNum, const std::string& name, const std::string& type)
  {
    if (!this->publishFilters) {
      return;
    } else {
      std::lock_guard<std::mutex>(this->filterPublishersMutex);
      const std::string topicName = "filter" + std::to_string(filterNum) + "/" + name;
      if (this->filterPublishers.find(name) == this->filterPublishers.end()) {
        this->filterPublishers[name] = this->privateNodeHandle.template advertise<F>(topicName, 50);
        ros::Duration(0.001).sleep();
      }

      this->filterPublishers[name].publish(data);
    }
  }
};
}
