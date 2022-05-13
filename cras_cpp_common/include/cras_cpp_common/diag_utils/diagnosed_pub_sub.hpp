#pragma once

/**
 * \file
 * \brief ROS message publisher and subscriber with automatic rate and delay diagnostics.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/forwards.h>
#include <ros/message_event.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/subscribe_options.h>

#include <cras_cpp_common/diag_utils/topic_status.hpp>
#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/message_utils.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{

/**
 * \brief Base for ROS publisher and subscriber with automatic message rate and delay diagnostics.
 * 
 * This class offers an option to configure the diagnostic task from a set of ROS parameters. The following parameters
 * are read:
 * 
 * - `rate/desired` (double, Hz, no default): If set, this value will be used as the default of min/max rates.
 * - `rate/min` (double, Hz, default 0.0 or desired): Minimum acceptable rate.
 * - `rate/max` (double, Hz, default +inf or desired): Maximum acceptable rate.
 * - `rate/tolerance` (double, default 0.1): Tolerance of the min/max rate (0.0 means that the min/max rate limits
 *                                           are strict).
 * - `rate/window_size` (uint, default 5): Over how many diagnostics updates should the rate be computed.
 * - `delay/min` (double, s, default -1.0): Minimum acceptable delay (only computed for messages with a header).
 * - `delay/max` (double, s, default 5.0): Maximum acceptable delay (only computed for messages with a header).
 * 
 * \tparam Message Type of the published/subscribed message.
 * \tparam Enable SFINAE only. Do not explicitly set.
 */
template<class Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
class DiagnosedPubSub
{
public:
  /**
   * \brief Use the given diagnostic task to diagnose the publisher/subscriber.
   * \param[in] diag 
   */
  explicit DiagnosedPubSub(const ::std::shared_ptr<::cras::TopicStatus<Message>>& diag);
  
  /**
   * \brief Create the diagnostic task from the given params.
   * \param[in] name Name of the task.
   * \param[in] diagParams Parameters of the task.
   */
  explicit DiagnosedPubSub(const ::std::string& name, const ::cras::TopicStatusParam<Message>& diagParams);
  
  /**
   * \brief Configure the diagnostic task from the given ROS parameter helper.
   * \param[in] params The ROS parameters to load configuration from.
   * \param[in] defaultParams Defaults used in case a ROS parameter doesn' exist.
   */
  DiagnosedPubSub(const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<Message>& defaultParams);

  /**
   * \brief Configure the diagnostic task from the given ROS parameter helper.
   * \param[in] params The ROS parameters to load configuration from.
   */
  explicit DiagnosedPubSub(const ::cras::BoundParamHelperPtr& params);
  
  virtual ~DiagnosedPubSub() = default;
  
  /**
   * \brief Add the diagnostic task to the given updater.
   * \param[in] updater The updater to add to.
   */
  virtual void attach(::diagnostic_updater::Updater& updater);
  
  /**
   * \brief Get the topic diagnostic task.
   * \return The task.
   */
  virtual ::std::shared_ptr<::cras::TopicStatus<Message>> getDiagnosticTask() const;

protected:
  /**
   * \brief When configuring from ROS params, load the delay configuration.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] topicStatusParam Some pre-loaded param struct.
   * \param[in] params Parameters to load from?
   */
  template <class M = Message, ::std::enable_if_t<::ros::message_traits::HasHeader<M>::value, bool> EnableM = true>
  void addDelayParams(::cras::SimpleTopicStatusParam<Message>& topicStatusParam,
    const ::cras::BoundParamHelperPtr& params);

  /**
   * \brief When configuring from ROS params for a headerless message type, this function does nothing.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] topicStatusParam Some pre-loaded param struct.
   * \param[in] params Parameters to load from?
   */
  template <class M = Message, ::std::enable_if_t<!::ros::message_traits::HasHeader<M>::value, bool> EnableM = true>
  void addDelayParams(::cras::SimpleTopicStatusParam<Message>& topicStatusParam,
    const ::cras::BoundParamHelperPtr& params);

  //! \brief The diagnostic task.
  ::std::shared_ptr<::cras::TopicStatus<Message>> diag {nullptr};
};

/**
 * \brief Wrapper for ROS publisher that automatically diagnoses the rate and delay of published messages.
 * \tparam Message Type of the published message.
 * \tparam Enable SFINAE only. Do not set explicitly.
 * \note For the automatic diagnostics to work, you have to call the `publish()` method from this class, not the one
 * from the ROS publisher.
 */
template<class Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
class DiagnosedPublisher : public ::cras::DiagnosedPubSub<Message>
{
public:
  /**
   * \brief Add the given diagnostic task to the publisher.
   * \param[in] pub The ROS publisher.
   * \param[in] diag The topic diagnostics task.
   */
  DiagnosedPublisher(const ::ros::Publisher &pub, const ::std::shared_ptr<::cras::TopicStatus<Message>>& diag);
  
  /**
   * \brief Add a topic diagnostic task to the publisher.
   * \param[in] pub The ROS publisher.
   * \param[in] name Name of the diagnostic task
   * \param[in] diag Parameters of the topic diagnostics task.
   */
  DiagnosedPublisher(const ::ros::Publisher &pub, const ::std::string& name,
    const ::cras::TopicStatusParam<Message>& diagParams);
  
  /**
   * \brief Add a topic diagnostic task to the publisher.
   * \param[in] pub The ROS publisher.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \sa DiagnosedPubSub
   */
  DiagnosedPublisher(const ::ros::Publisher &pub, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<Message>& defaultParams);
  
  /**
   * \brief Add a topic diagnostic task to the publisher.
   * \param[in] pub The ROS publisher.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \sa DiagnosedPubSub
   */
  DiagnosedPublisher(const ::ros::Publisher &pub, const ::cras::BoundParamHelperPtr& params);

  [[deprecated]]
  DiagnosedPublisher(const ::ros::Publisher &pub, ::diagnostic_updater::Updater& updater,
    const ::cras::BoundParamHelperPtr& params);

  [[deprecated]]
  DiagnosedPublisher(const ::ros::Publisher &pub, ::diagnostic_updater::Updater& updater,
    const ::cras::BoundParamHelperPtr& params, const ::ros::Rate& defaultRate);

  [[deprecated]]
  DiagnosedPublisher(const ::ros::Publisher &pub, ::diagnostic_updater::Updater& updater,
    const ::cras::BoundParamHelperPtr& params, const ::ros::Rate& defaultRate,
    const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate);
  
  [[deprecated]]
  ::ros::Rate getDesiredRate() const;
  
  /**
   * \brief Publish a message.
   * \param[in] message The message to publish.
   */
  virtual void publish(const typename Message::Ptr& message);

  /**
   * \brief Publish a message.
   * \param[in] message The message to publish.
   */
  virtual void publish(const Message& message);

  /**
   * \brief Get the ROS publisher.
   * \return The ROS publisher.
   */
  const ::ros::Publisher& getPublisher() const;

  /**
   * \brief Get the ROS publisher.
   * \return The ROS publisher.
   */
  ::ros::Publisher& getPublisher();

  /**
   * \brief Change the ROS publisher.
   * \param[in] pub The new ROS publisher.
   */
  void setPublisher(const ::ros::Publisher& pub);
  
protected:
  //! \brief The ROS publisher.
  ::ros::Publisher publisher;
};

/**
 * \brief Wrapper for ROS subscriber that automatically diagnoses the rate and delay of received messages.
 * \tparam Message Type of the subscribed message.
 * \tparam Enable SFINAE only. Do not set explicitly.
 */
template<class Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
class DiagnosedSubscriber : public ::cras::DiagnosedPubSub<Message>
{
public:
  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] hints Connection hints.
   */
  template <typename M, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
    const ::std::string& topic, uint32_t queue_size, void(*cb)(M), ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] hints Connection hints.
   */
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<Message>& defaultParams,
    const ::std::string& topic, uint32_t queue_size,
    const ::boost::function<void (const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \tparam C SFINAE only. Do not set explicitly.
   * \tparam EnableC SFINAE only. Do not set explicitly.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] obj Tracked object for the subscriber.
   * \param[in] hints Connection hints.
   */
  template <typename C, typename EnableC = ::std::enable_if_t<::cras::IsMessageParam<C>::value>>
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>& defaultParams,
    const ::std::string& topic, uint32_t queue_size,
    const ::boost::function<void (C)>& cb, ::ros::VoidConstPtr obj = {}, ::ros::TransportHints hints = {});
  
  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \param[in] hints Connection hints.
   */
  template<typename M, class T, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), T* obj, ::ros::TransportHints hints = {});
  
  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \param[in] hints Connection hints.
   */
  template<typename M, class T, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, T* obj, ::ros::TransportHints hints = {});
  
  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \param[in] hints Connection hints.
   */
  template<typename M, class T, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), const ::boost::shared_ptr<T>& obj,
    ::ros::TransportHints hints = {});
  
  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] topic Name of the topic to subscribe.
   * \param[in] queue_size Queue size of the subscriber.
   * \param[in] cb The message callback.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \param[in] hints Connection hints.
   */
  template<typename M, class T, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj,
    ::ros::TransportHints hints = {});
  
  /**
   * \brief Subscribe to the given topic reading diagnostic task configuration from ROS parameters.
   * \param[in] nh Node handle used for subscribing.
   * \param[in] params ROS parameters from which the configuration of the diagnostic task is taken.
   * \param[in] defaultParams Default parameters used when no ROS parameter exists.
   * \param[in] options Subscribe options.
   * \note This constructor will "wrap" the callback specified in `options` to automatically trigger the diagnostic task
   * when a message is received.
   */
  DiagnosedSubscriber(::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
    const ::cras::SimpleTopicStatusParam<Message>& defaultParams, ::ros::SubscribeOptions& options);
  
  /**
   * \brief Get the created ROS subscriber.
   * \return The ROS subscriber.
   */
  const ::ros::Subscriber& getSubscriber() const;

  /**
   * \brief Get the created ROS subscriber.
   * \return The ROS subscriber.
   */
  ::ros::Subscriber& getSubscriber();

protected:
  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \param[in] fn The function to wrap.
   * \return The wrapping function.
   */
  ::boost::function<void(const Message&)> addTick(const ::std::function<void(const Message&)>& fn);

  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \param[in] fn The function to wrap.
   * \return The wrapping function.
   */
  ::boost::function<void(const typename Message::Ptr&)> addTick(
    const ::std::function<void(const typename Message::Ptr&)>& fn);

  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \param[in] fn The function to wrap.
   * \return The wrapping function.
   */
  ::boost::function<void(const ::ros::MessageEvent<Message>&)> addTick(
    const ::std::function<void(const ::ros::MessageEvent<Message>&)>& fn);
  
  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \tparam T Ignored.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] fn The function to wrap.
   * \return The wrapping function.
   */
  template <typename T, typename M, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::boost::function<void(M)> addTick(const ::std::function<void(M)>& fn, T);

  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \tparam T Ignored.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] fn The function to wrap.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \return The wrapping function.
   */
  template <typename T, typename M, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::boost::function<void(M)> addTick(void(T::*fn)(M), T* obj);

  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \tparam T Ignored.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] fn The function to wrap.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \return The wrapping function.
   */
  template <typename T, typename M, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::boost::function<void(M)> addTick(void(T::*fn)(M), const ::boost::shared_ptr<T>& obj);

  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \tparam T Ignored.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] fn The function to wrap.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \return The wrapping function.
   */
  template <typename T, typename M, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::boost::function<void(M)> addTick(void(T::*fn)(M) const, T* obj);

  /**
   * \brief Wrap `fn` in a function that first calls `tick()` on the diagnostic task and then calls `fn()`.
   * \tparam T Ignored.
   * \tparam M SFINAE only. Do not set explicitly.
   * \tparam EnableM SFINAE only. Do not set explicitly.
   * \param[in] fn The function to wrap.
   * \param[in] obj Tracked object for the subscriber and on which the callback will be called.
   * \return The wrapping function.
   */
  template <typename T, typename M, typename EnableM = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::boost::function<void(M)> addTick(void(T::*fn)(M) const, const ::boost::shared_ptr<T>& obj);
  
  //! \brief The ROS subscriber.
  ::ros::Subscriber subscriber;
};

// Class template deduction guides allow leaving out the type parameter when declaring a DiagnosedSubscriber. C++17 only
#if __cpp_deduction_guides

template <typename M, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
DiagnosedSubscriber(::ros::NodeHandle, const ::cras::BoundParamHelperPtr&,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>&, const ::std::string&, uint32_t, void(*)(M),
  ::ros::TransportHints = {})
    -> DiagnosedSubscriber<::cras::BaseMessage<M>>;

template <typename C, typename = ::std::enable_if_t<::cras::IsMessageParam<C>::value>>
DiagnosedSubscriber(::ros::NodeHandle, const ::cras::BoundParamHelperPtr&,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>&, const ::std::string&, uint32_t,
  const ::boost::function<void (C)>&, ::ros::VoidConstPtr = {}, ::ros::TransportHints = {})
    -> DiagnosedSubscriber<::cras::BaseMessage<C>>;

template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
DiagnosedSubscriber(::ros::NodeHandle, const ::cras::BoundParamHelperPtr&,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>&, const ::std::string&, uint32_t,
  void(T::*)(M), T*, ::ros::TransportHints = {})
    -> DiagnosedSubscriber<::cras::BaseMessage<M>>;

template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
DiagnosedSubscriber(::ros::NodeHandle, const ::cras::BoundParamHelperPtr&,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>&, const ::std::string&, uint32_t,
  void(T::*)(M) const, T*, ::ros::TransportHints = {})
    -> DiagnosedSubscriber<::cras::BaseMessage<M>>;

template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
DiagnosedSubscriber(::ros::NodeHandle, const ::cras::BoundParamHelperPtr&,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>&, const ::std::string&, uint32_t,
  void(T::*)(M), const ::boost::shared_ptr<T>&, ::ros::TransportHints = {})
    -> DiagnosedSubscriber<::cras::BaseMessage<M>>;

template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
DiagnosedSubscriber(::ros::NodeHandle, const ::cras::BoundParamHelperPtr&,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>&, const ::std::string&, uint32_t,
  void(T::*)(M) const, const ::boost::shared_ptr<T>&, ::ros::TransportHints = {})
    -> DiagnosedSubscriber<::cras::BaseMessage<M>>;

#endif
}

#include <cras_cpp_common/diag_utils/impl/diagnosed_pub_sub.hpp>