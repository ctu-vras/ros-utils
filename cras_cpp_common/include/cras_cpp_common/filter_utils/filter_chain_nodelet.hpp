#pragma once

/**
 * \file
 * \brief A versatile nodelet that can load and run a filter chain, run its diagnostics, and enable/disable single
 *        filters using dynamic reconfigure.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#include <boost/thread/recursive_mutex.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <dynamic_reconfigure/server.h>
#include <ros/duration.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <cras_cpp_common/FilterChainConfig.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/diag_utils/duration_status.h>
#include <cras_cpp_common/filter_utils/filter_chain.hpp>
#include <cras_cpp_common/nodelet_utils.hpp>

namespace cras
{

template <typename F>
class FilterChainDiagnostics;

/**
 * \brief A versatile nodelet that can load and run a filter chain.
 * \tparam F Type of the filtered data.
 * 
 * The following ROS parameters are read:
 * - ~lazy_subscription (bool): If true, the nodelet will only subscribe the input topic when there is at least one
 *                              subscriber for the output (filtered) topic. Default is true.
 * - ~subscriber_queue_size (uint): Queue size for the input topic subscriber. Default is 15.
 * - ~publisher_queue_size (uint): Queue size for the output topic publisher. Default is 15.
 * - ~publish_diagnostics (bool): Whether to publish all the following diagnostics (single diagnostics can be turned off
 *                                by the respective config if this is set to true). Default is false.
 * - ~publish_topic_diagnostics (bool): Whether to create and publish topic diagnostics.
 *                                      Default is {publish_diagnostics}.
 * - ~publish_duration_diagnostics (bool): Whether to diagnose callback duration. Default is {publish_diagnostics}.
 * - ~publish_chain_diagnostics (bool): Whether to diagnose overall chain performance. Default is {publish_diagnostics}.
 * - ~{TOPIC_IN} (struct): Configuration of the topic diagnostics of the input topic. {TOPIC_IN} is the name passed as
 *                         `topicIn` parameter in the constructor (by default `in`). This name is not affected by
 *                         remapping the input topic to another name. This parameter is only read when
 *                         `publish_diagnostics` is true.
 * - ~{TOPIC_FILTERED} (struct): Configuration of the topic diagnostics of the filtered topic. {TOPIC_FILTERED} is the
 *                               name passed as `topicFiltered` parameter in the constructor (by default `out`).
 *                               This name is not affected by remapping the filtered topic to another name.
 *                               This parameter is only read when `publish_diagnostics` is true.
 * - ~update_duration (struct): Configuration of duration diagnostics for the overall callback. See DurationStatus task.
 * - ~{FILTER_NAME}/update_duration (struct): Configuration of callback duration diagnostics for filter {FILTER_NAME}.
 * 
 * Subscribed topics:
 * - {TOPIC_IN} (F): The input data. {TOPIC_IN} is the name passed as `topicIn` parameter in the constructor
 *                   (by default `in`).
 *                   
 * Published topics:
 * - {TOPIC_FILTERED} (F): The filtered data. {TOPIC_FILTERED} is the name passed as `topicFiltered` parameter in the
 *                         constructor (by default `out`).
 * - ~filter{I}/{FILTER_NAME} (F): Output of {I}-th filter with name {FILTER_NAME} (if `publish_each_filter` is true).
 * - /diagnostics: Standard diagnostics output (if any diagnostics are enabled).
 * 
 * The following options can be set at runtime via dynamic reconfigure:
 * - publish_each_filter (bool): If true, an additional publisher will be created for each filter in the chain and it
 *                               will publish the message as it looked directly after the filter was applied. Name of
 *                               the topic is ~filter{I}/{FILTER_NAME}, where `I` is the order (index) of the applied
 *                               filter in the chain and `FILTER_NAME` is the name of the applied filter.
 *                               Default is false.
 * - max_age (float): Maximum age (in seconds) of messages that are processed by the filter chain. Age of a message is
 *                    computed as the difference between `ros::Time::now()` and the message timestamp (it can be
 *                    negative if the message is future-stamped). Messages without header are stamped by
 *                    `ros::Time::now()` on receipt, so they can never be considered old. Default is 3600 seconds.
 * - disabled_filters (string): A comma-separated list of names of filters that are disabled. A disabled filter is
 *                              skipped by the chain, which means that after applying the preceding filter, this filter
 *                              is skipped and the next non-disabled filter will process the message afterwards. Setting
 *                              this to empty string will enable all filters. Default is empty string (all filters
 *                              allowed).
 */
template<class F>
class FilterChainNodelet : public ::cras::Nodelet
{

public:
  //! \brief Type of the filtered data.
  typedef F DataType;
  
  //! \brief Accessor to DataType field of the filtered message type.
  typedef ::ros::message_traits::DataType<F> MsgDataType;
  
  /**
   * \brief Read ROS parameters, initialize publishers/subscribers, initialize other class members.
   *        ROS::init() is assumed to have been called before.
   * \param[in] dataType Type of the filtered messages.
   * \param[in] topicIn Input topic.
   * \param[in] topicFiltered Output topic.
   * \param[in] configNamespace ROS parameter namespace from which configuration should be read.
   */
  FilterChainNodelet(const ::std::string& dataType, const ::std::string& topicIn, const ::std::string& topicFiltered,
    const ::std::string& configNamespace);
  
  /**
   * \brief Read ROS parameters, initialize publishers/subscribers, initialize other class members.
   *        ROS::init() is assumed to have been called before.
   * \param[in] topicIn Input topic.
   * \param[in] topicFiltered Output topic.
   * \param[in] configNamespace ROS parameter namespace from which configuration should be read.
   */
  FilterChainNodelet(const ::std::string& topicIn, const ::std::string& topicFiltered,
    const ::std::string& configNamespace);
  
  /**
   * \brief Read ROS parameters, initialize publishers/subscribers, initialize other class members.
   *        ROS::init() is assumed to have been called before. Topic "in" is subscribed and "out" is published.
   * \param[in] configNamespace ROS parameter namespace from which configuration should be read.
   */
  explicit FilterChainNodelet(const ::std::string& configNamespace);

  ~FilterChainNodelet() override;

  /**
   * \brief Enable publishers of single filter results.
   */
  void enableFilterPublishers();

  /**
   * \brief Disable publishers of single filter results.
   */
  void disableFilterPublishers();

  /**
   * \brief Set which filters are temporarily disabled.
   * \param[in] filters The filters to disable. 
   */
  void setDisabledFilters(::std::unordered_set<::std::string> filters);

  /**
   * \brief Set the maximum age of messages that are accepted and filtered.
   * \param[in] maxAge The maximum age.
   */
  void setMaxAge(const ::ros::Duration& maxAge);

protected:

  /**
   * \brief Dynamic reconfiguration of parameters.
   * \param[in,out] config The configuration to apply.
   */
  void dynreconfCallback(::cras_cpp_common::FilterChainConfig& config, uint32_t);
  
  /**
   * \brief Update dynamic parameters on the server to correspond to current settings of this nodelet.
   */
  void updateDynamicParams() const;

  void onInit() override;

  /**
   * \brief Subscribe to the input topic.
   * \note This method is called either directly from onInit() (if non-lazy subscription is set), or dynamically on
   *       output topic (un)subscription (when lazy subscription is set).
   */
  void subscribe();

  /**
   * \brief Callback called when a new subscriber of the output topic appears.
   */
  virtual void connectCb(const ::ros::SingleSubscriberPublisher&);

  /**
   * \brief Callback called when a subscriber of the output topic disappears.
   */
  virtual void disconnectCb(const ::ros::SingleSubscriberPublisher&);
  
  /**
   * \brief Get timestamp from a message with header.
   * \tparam T Type of the message.
   * \param[in] data The input message. 
   * \return message.header.stamp field.
   */
  template <typename T, ::std::enable_if_t<::ros::message_traits::HasHeader<T>::value, bool> = true>
  ::ros::Time getStamp(const typename T::ConstPtr& data);

  /**
   * \brief Get current time as timestamp of the given message without header.
   * \tparam T Type of the message.
   * \param[in] data The input message.
   * \return ros::Time::now() .
   */
  template <typename T, ::std::enable_if_t<!::ros::message_traits::HasHeader<T>::value, bool> = true>
  ::ros::Time getStamp(const typename T::ConstPtr& data);

  /**
   * \brief Callback called when a new message arrives on the input topic.
   * \param[in] data The received message.
   * \note This is the function that calls FilterChain.update(). If a child class overrides this method, it is
   *       responsible for calling the update function.
   */
  virtual void dataCallback(const typename F::ConstPtr& data);

  /**
   * \brief Callback called before each filter starts working on a message.
   * \param[in] data The filtered message (before filter application).
   * \param[in] filterNum Order of the applied filter in the chain.
   * \param[in] name Name of the applied filter.
   * \param[in] type Type of the applied filter.
   */
  virtual void filterStartCallback(const F& data, size_t filterNum, const ::std::string& name,
    const ::std::string& type);

  /**
   * \brief Callback called after each filter finishes working on a message.
   * \param[in] data The filtered message (after filter application).
   * \param[in] filterNum Order of the applied filter in the chain.
   * \param[in] name Name of the applied filter.
   * \param[in] type Type of the applied filter.
   * \param[in] success Whether the filter succeeded (its update() function returned true).
   */
  virtual void filterFinishedCallback(const F& data, size_t filterNum, const ::std::string& name,
    const ::std::string& type, bool success);
  
  //! \brief Public NodeHandle.
  ::ros::NodeHandle nodeHandle;

  //! \brief Private NodeHandle.
  ::ros::NodeHandle privateNodeHandle;

  //! \brief Subscriber to data.
  ::ros::Subscriber subscriber;
  
  //! \brief Subscriber diagnostics.
  ::std::unique_ptr<::cras::DiagnosedSubscriber<F>> subscriberDiag;

  //! \brief The chain of filters to apply to the incoming messages.
  ::cras::FilterChain<F> filterChain;

  //! \brief Publisher for the filtered data.
  ::ros::Publisher filteredPublisher;
  
  //! \brief Publisher diagnostics.
  ::std::unique_ptr<::cras::DiagnosedPublisher<F>> publisherDiag;

  //! \brief Topic for incoming messages.
  ::std::string topicIn;
  
  //! \brief Topic for outgoing (filtered) messages.
  ::std::string topicFiltered;
  
  //! \brief ROS parameter namespace from which filter configuration will be read.
  ::std::string configNamespace;

  //! \brief Whether to create a publisher for each filter. Can be changed during runtime via `enableFilterPublishers()`
  bool publishFilters {false};

  //! \brief Queue size for publishers.
  size_t publisherQueueSize {15_sz};
  
  //! \brief Publishers for individual filter results.
  ::std::unordered_map<::std::string, ::ros::Publisher> filterPublishers;
  
  //! \brief Mutex that protects filterPublishers.
  ::std::mutex filterPublishersMutex;
  
  //! \brief Mutex that protects subscriber, subscriberDiag. 
  ::std::mutex connectMutex;

  //! \brief Whether to use the lazy subscription model (subscribe to input only when someone subscribes to output).
  bool lazySubscription {true};
  
  //! \brief Queue size for subscriber.
  size_t subscriberQueueSize {15_sz};
  
  //! \brief Whether topic frequency/delay statistics should be published to diagnostics.
  bool publishTopicDiagnostics {false};
  
  //! \brief Whether callback duration statistics should be published to diagnostics.
  bool publishDurationDiagnostics {false};
  
  //! \brief Whether overall chain diagnostics should be published to diagnostics.
  bool publishChainDiagnostics {false};

  //! \brief Maximum age of a message for it to be considered. Older messages are thrown away when received.
  ::ros::Duration maxAge;
  
  //! \brief Mutex protecting the dynamic reconfigure options, maxAge and publishFilters.
  mutable ::boost::recursive_mutex configMutex;

  //! \brief Dynamic reconfigure server.
  ::std::unique_ptr<::dynamic_reconfigure::Server<::cras_cpp_common::FilterChainConfig>> dynreconfServer;
  
  //! \brief Diagnostic task for overall callback duration.
  ::std::unique_ptr<::cras::DurationStatus> callbackDurationDiag;
  
  //! \brief Diagnostic tasks for duration of each filter callback.
  ::std::unordered_map<::std::string, ::std::unique_ptr<::cras::DurationStatus>> filterCallbackDurationDiags;

  //! \brief Diagnostic task for overall chain performance.
  ::std::unique_ptr<::cras::FilterChainDiagnostics<F>> chainDiag;
};

/**
 * \brief Diagnostics of performance of a filter chain. 
 * \tparam F Type of filtered data.
 */
template <typename F>
class FilterChainDiagnostics : public ::diagnostic_updater::DiagnosticTask
{
public:
  /**
   * \param[in] name Name of the diagnostic task.
   * \param[in] chain The diagnosed chain.
   */
  FilterChainDiagnostics(const ::std::string& name, const ::cras::FilterChain<F>& chain);
  
  /**
   * \brief Call this function every time a filter finished callback is called.
   * \param[in] filterName Name of the filter.
   * \param[in] success Whether the filter has succeeded.
   */
  void addReport(const ::std::string& filterName, bool success);

  void run(::diagnostic_updater::DiagnosticStatusWrapper& stat) override;
  
protected:
  //! \brief The diagnosed chain.
  const ::cras::FilterChain<F>& chain;
  
  //! \brief Mutex protecting numCallbacks, numSuccesses and numFailures.
  ::std::mutex mutex;
  
  //! \brief The overall number of callbacks since last update.
  ::std::unordered_map<::std::string, size_t> numCallbacks;
  
  //! \brief The number of successful filter runs since last update.
  ::std::unordered_map<::std::string, size_t> numSuccesses;
  
  //! \brief The number of failed filter runs since last update.
  ::std::unordered_map<::std::string, size_t> numFailures;
};

}

#include "impl/filter_chain_nodelet.hpp"