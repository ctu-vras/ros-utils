#pragma once

/**
 * \file
 * \brief ROS message publisher and subscriber with automatic rate and delay diagnostics (implementation details, do not
 * include directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscription_callback_helper.h>
#include <ros/subscriber.h>
#include <ros/subscribe_options.h>

#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/diag_utils/topic_status.hpp>
#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/time_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{

template<class Message, typename Enable>
DiagnosedPubSub<Message, Enable>::DiagnosedPubSub(const ::std::shared_ptr<::cras::TopicStatus<Message>>& diag) :
  diag(diag)
{
}

template<class Message, typename Enable>
DiagnosedPubSub<Message, Enable>::DiagnosedPubSub(const ::std::string& name,
  const ::cras::TopicStatusParam<Message>& diagParams) :
    DiagnosedPubSub(::std::make_shared<::cras::TopicStatus<Message>>(name, diagParams))
{
}

template<class Message, typename Enable>
DiagnosedPubSub<Message, Enable>::DiagnosedPubSub(const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<Message>& defaultParams)
{
  auto topicStatusParam = defaultParams;

  if (params->hasParam("rate/desired"))
  {
    const auto desiredRate = params->getParam("rate/desired", 10.0, "Hz");
    topicStatusParam.minRate = params->getParam("rate/min", desiredRate, "Hz");
    topicStatusParam.maxRate = params->getParam("rate/max", desiredRate, "Hz");
  }
  else
  {
    topicStatusParam.minRate = params->getParam("rate/min", topicStatusParam.minRate, "Hz");
    topicStatusParam.maxRate = params->getParam("rate/max", topicStatusParam.maxRate, "Hz");
  }

  topicStatusParam.rateTolerance = params->getParam("rate/tolerance", topicStatusParam.rateTolerance);
  topicStatusParam.rateWindowSize = params->getParam("rate/window_size", topicStatusParam.rateWindowSize, "updates");
  
  this->template addDelayParams(topicStatusParam, params);
  
  this->diag = ::std::make_shared<::cras::TopicStatus<Message>>(params->getNamespace(), topicStatusParam);
}

template<class Message, typename Enable>
DiagnosedPubSub<Message, Enable>::DiagnosedPubSub(const ::cras::BoundParamHelperPtr& params) :
  DiagnosedPubSub(params, {})
{
}

template<class Message, typename Enable>
void DiagnosedPubSub<Message, Enable>::attach(::diagnostic_updater::Updater& updater)
{
  updater.add(*this->diag);
}

template<class Message, typename Enable>
::std::shared_ptr<::cras::TopicStatus<Message>> DiagnosedPubSub<Message, Enable>::getDiagnosticTask() const
{
  return this->diag;
}

template<class Message, typename Enable>
template<class M, ::std::enable_if_t<::ros::message_traits::HasHeader<M>::value, bool> EnableM>
void DiagnosedPubSub<Message, Enable>::addDelayParams(::cras::SimpleTopicStatusParam<Message>& topicStatusParam,
  const ::cras::BoundParamHelperPtr& params)
{
  topicStatusParam.minDelay = params->getParam("delay/min", topicStatusParam.minDelay, "s");
  topicStatusParam.maxDelay = params->getParam("delay/max", topicStatusParam.maxDelay, "s");
}

template<class Message, typename Enable>
template<class M, ::std::enable_if_t<!::ros::message_traits::HasHeader<M>::value, bool> EnableM>
void DiagnosedPubSub<Message, Enable>::addDelayParams(::cras::SimpleTopicStatusParam<Message>& topicStatusParam,
  const ::cras::BoundParamHelperPtr& params)
{
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher& pub,
  const ::std::shared_ptr<::cras::TopicStatus<Message>>& diag) : ::cras::DiagnosedPubSub<Message>(diag), publisher(pub)
{
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher &pub, const ::std::string& name,
  const ::cras::TopicStatusParam<Message>& diagParams) :
    DiagnosedPublisher(pub, ::std::make_shared<::cras::TopicStatus<Message>>(name, diagParams))
{
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher &pub,
  const ::cras::BoundParamHelperPtr& params, const ::cras::SimpleTopicStatusParam<Message>& defaultParams) :
    ::cras::DiagnosedPubSub<Message>(params, defaultParams), publisher(pub)
{
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher& pub,
  const ::cras::BoundParamHelperPtr& params) : DiagnosedPublisher(pub, params, {})
{
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher& pub,
  ::diagnostic_updater::Updater& updater, const ::cras::BoundParamHelperPtr& params) :
    DiagnosedPublisher(pub, params, {})
{
  updater.add(*this->diag);
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher& pub,
  ::diagnostic_updater::Updater& updater, const ::cras::BoundParamHelperPtr& params, const ::ros::Rate& defaultRate) :
    DiagnosedPublisher(pub, params, {::cras::frequency(defaultRate), ::cras::frequency(defaultRate)})
{
  updater.add(*this->diag);
}

template<class Message, typename Enable>
DiagnosedPublisher<Message, Enable>::DiagnosedPublisher(const ::ros::Publisher& pub,
  ::diagnostic_updater::Updater& updater, const ::cras::BoundParamHelperPtr& params,
  const ::ros::Rate& defaultRate, const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate) :
  DiagnosedPublisher(pub, params, {::cras::frequency(defaultMinRate), ::cras::frequency(defaultMaxRate)})
{
  updater.add(*this->diag);
}

template<class Message, typename Enable>
::ros::Rate DiagnosedPublisher<Message, Enable>::getDesiredRate() const
{
  return this->diag->getExpectedRate();
}

template<class Message, typename Enable>
void DiagnosedPublisher<Message, Enable>::publish(const typename Message::Ptr& message)
{
  this->diag->tick(message);
  this->publisher.publish(message);
}

template<class Message, typename Enable>
void DiagnosedPublisher<Message, Enable>::publish(const Message& message)
{
  this->diag->tick(message);
  this->publisher.publish(message);
}

template<class Message, typename Enable>
const ::ros::Publisher& DiagnosedPublisher<Message, Enable>::getPublisher() const
{
  return this->publisher;
}

template<class Message, typename Enable>
::ros::Publisher& DiagnosedPublisher<Message, Enable>::getPublisher()
{
  return this->publisher;
}

template<class Message, typename Enable>
void DiagnosedPublisher<Message, Enable>::setPublisher(const ::ros::Publisher& pub)
{
  this->publisher = pub;
}

namespace impl
{

/**
 * \brief This SubscriptionCallbackHelper "prepends" a call to diag->tick() to every message callback.
 * \tparam Message Type of the message.
 */
template <typename Message>
class AddTickSubscriptionCallbackHelper : public ::ros::SubscriptionCallbackHelper
{
public:
  /**
   * \brief Create the callback helper.
   * \param[in] orig The original subscription callback helper. Most methods are just relayed on it.
   * \param[in] log Log helper.
   * \param[in] diag The topic diagnostic task which should receive the added tick() call.
   * \param[in] topic Topic on which the subscription happens.
   */
  AddTickSubscriptionCallbackHelper(const ::ros::SubscriptionCallbackHelperPtr& orig,
    const ::cras::LogHelperPtr& log, const ::cras::TopicStatusPtr<Message>& diag, const ::std::string& topic) :
      orig(orig), log(log), diag(diag), topic(topic)
  {
  }

  ::ros::VoidConstPtr deserialize(const ::ros::SubscriptionCallbackHelperDeserializeParams& _params) override
  {
    return this->orig->deserialize(_params);
  }

  void call(::ros::SubscriptionCallbackHelperCallParams& params) override
  {
    // params.event is MessageEvent<const void> (ROS applies type erasure to get rid of templates)
    // This cast comes from the following MessageEvent constructor from ros/message_event.h:
    // MessageEvent(const MessageEvent<void const>& rhs, const CreateFunction& create)
    const auto msg = ::boost::static_pointer_cast<Message const>(params.event.getConstMessage());
    if (msg != nullptr)
      this->diag->tick(msg);
    else
      this->log->logError("Could not read header of %s from publisher %s on topic %s",
        ::cras::getTypeName(this->getTypeInfo()).c_str(), params.event.getPublisherName().c_str(),
        this->topic.c_str());

    this->orig->call(params);
  }

  const ::std::type_info& getTypeInfo() override
  {
    return this->orig->getTypeInfo();
  }

  bool isConst() override
  {
    return this->orig->isConst();
  }

  bool hasHeader() override
  {
    return this->orig->hasHeader();
  }

private:
  //! \brief The original subscription callback helper. Most methods are just relayed on it.
  ::ros::SubscriptionCallbackHelperPtr orig;
  
  //! \brief Log helper.
  ::cras::LogHelperPtr log;
  
  //! \brief The topic diagnostic task which should receive the added tick() call.
  ::cras::TopicStatusPtr<Message> diag;
  
  //! \brief Topic on which the subscription happens.
  ::std::string topic;
};

}

template<class Message, typename Enable>
template<typename M, typename EnableM>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size, void (*cb)(M), ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<::cras::BaseMessage<M>>(params, defaultParams),
    subscriber(nh.template subscribe<::cras::BaseMessage<M>>(
      topic, queue_size, this->addTick(cb), ::ros::VoidConstPtr(), hints))
{
}

template<class Message, typename Enable>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<Message>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size,
  const ::boost::function<void (const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<Message>(params, defaultParams),
    subscriber(nh.template subscribe<Message>(topic, queue_size, this->addTick(cb), ::ros::VoidConstPtr(), hints))
{
}

template<class Message, typename Enable>
template<typename C, typename EnableC>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size, const ::boost::function<void(C)>& cb, ::ros::VoidConstPtr obj,
  ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<::cras::BaseMessage<C>>(params, defaultParams),
    subscriber(nh.template subscribe<::cras::BaseMessage<C>>(topic, queue_size, this->addTick(cb), obj, hints))
{
}

template<class Message, typename Enable>
template<typename M, class T, typename EnableM>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size, void (T::*cb)(M), T* obj, ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<::cras::BaseMessage<M>>(params, defaultParams),
    subscriber(nh.template subscribe<::cras::BaseMessage<M>>(
      topic, queue_size, this->addTick(cb, obj), ::ros::VoidConstPtr(), hints))
{
}

template<class Message, typename Enable>
template<typename M, class T, typename EnableM>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size, void (T::*cb)(M) const, T* obj, ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<::cras::BaseMessage<M>>(params, defaultParams),
    subscriber(nh.template subscribe<::cras::BaseMessage<M>>(
      topic, queue_size, this->addTick(cb, obj), ::ros::VoidConstPtr(), hints))
{
}

template<class Message, typename Enable>
template<typename M, class T, typename EnableM>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size, void (T::*cb)(M), const ::boost::shared_ptr<T>& obj,
  ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<::cras::BaseMessage<M>>(params, defaultParams),
    subscriber(nh.template subscribe<::cras::BaseMessage<M>>(
      topic, queue_size, this->addTick(cb, obj), ::ros::VoidConstPtr(), hints))
{
}

template<class Message, typename Enable>
template<typename M, class T, typename EnableM>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultParams,
  const ::std::string& topic, const uint32_t queue_size, void (T::*cb)(M) const, const ::boost::shared_ptr<T>& obj,
  ::ros::TransportHints hints) :
    ::cras::DiagnosedPubSub<::cras::BaseMessage<M>>(params, defaultParams),
    subscriber(nh.template subscribe<::cras::BaseMessage<M>>(
      topic, queue_size, this->addTick(cb, obj), ::ros::VoidConstPtr(), hints))
{
}

template<class Message, typename Enable>
DiagnosedSubscriber<Message, Enable>::DiagnosedSubscriber(
  ::ros::NodeHandle nh, const ::cras::BoundParamHelperPtr& params,
  const ::cras::SimpleTopicStatusParam<Message>& defaultParams, ::ros::SubscribeOptions& options) :
    ::cras::DiagnosedPubSub<Message>(params, defaultParams), subscriber({})
{
  auto origHelper = options.helper;
  options.helper.reset(new ::cras::impl::AddTickSubscriptionCallbackHelper<Message>(
    origHelper, params->getLogger(), this->diag, options.topic));
  this->subscriber = nh.subscribe(options);
}

template<class Message, typename Enable>
const ::ros::Subscriber& DiagnosedSubscriber<Message, Enable>::getSubscriber() const
{
  return this->subscriber;
}

template<class Message, typename Enable>
::ros::Subscriber& DiagnosedSubscriber<Message, Enable>::getSubscriber()
{
  return this->subscriber;
}

template<class Message, typename Enable>
::boost::function<void(const Message&)> DiagnosedSubscriber<Message, Enable>::addTick(
  const ::std::function<void(const Message&)>& fn)
{
  return [this, fn](const Message& m) {this->diag->tick(m); fn(m);};
}

template<class Message, typename Enable>
::boost::function<void(const typename Message::Ptr&)> DiagnosedSubscriber<Message, Enable>::addTick(
  const ::std::function<void(const typename Message::Ptr&)>& fn)
{
  return [this, fn](const typename Message::Ptr& m) {this->diag->tick(m); fn(m);};
}

template<class Message, typename Enable>
::boost::function<void(const ::ros::MessageEvent<Message>&)> DiagnosedSubscriber<Message, Enable>::addTick(
  const ::std::function<void(const ::ros::MessageEvent<Message>&)>& fn)
{
  return [this, fn](const ::ros::MessageEvent<Message>& m) {this->diag->tick(m); fn(m);};
}

template<class Message, typename Enable>
template<typename T, typename M, typename EnableM>
::boost::function<void(M)> DiagnosedSubscriber<Message, Enable>::addTick(const ::std::function<void(M)>& fn, T)
{
  return [this, fn](M m) {this->diag->tick(m); fn(m);};
}

template<class Message, typename Enable>
template<typename T, typename M, typename EnableM>
::boost::function<void(M)> DiagnosedSubscriber<Message, Enable>::addTick(void (T::*fn)(M), T* obj)
{
  return [this, fn, obj](M m) {this->diag->tick(m); (obj->*fn)(m);};
}

template<class Message, typename Enable>
template<typename T, typename M, typename EnableM>
::boost::function<void(M)> DiagnosedSubscriber<Message, Enable>::addTick(
  void (T::*fn)(M), const ::boost::shared_ptr<T>& obj)
{
  return [this, fn, obj](M m) {this->diag->tick(m); ((*obj).*fn)(m);};
}

template<class Message, typename Enable>
template<typename T, typename M, typename EnableM>
::boost::function<void(M)> DiagnosedSubscriber<Message, Enable>::addTick(void (T::*fn)(M) const, T* obj)
{
  return [this, fn, obj](M m) {this->diag->tick(m); (obj->*fn)(m);};
}

template<class Message, typename Enable>
template<typename T, typename M, typename EnableM>
::boost::function<void(M)> DiagnosedSubscriber<Message, Enable>::addTick(
  void (T::*fn)(M) const, const ::boost::shared_ptr<T>& obj)
{
  return [this, fn, obj](M m) {this->diag->tick(m); ((*obj).*fn)(m);};
}

}