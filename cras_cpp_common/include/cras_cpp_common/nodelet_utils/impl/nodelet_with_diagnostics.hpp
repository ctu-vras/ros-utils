#pragma once

/**
 * \file
 * \brief Helpers for setting up diagnostics for nodelets (private implementation details, do not include this
 * directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "../nodelet_with_diagnostics.hpp"

#include <memory>
#include <string>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <nodelet/nodelet.h>
#include <ros/advertise_options.h>
#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <ros/timer.h>

#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/diag_utils/updater.h>
#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/nodelet_utils/param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <cras_cpp_common/log_utils/nodelet.h>

namespace cras
{

namespace impl
{

/**
 * \brief Private implementation (PIMPL) class for NodeletWithDiagnostics.
 */
struct NodeletWithDiagnosticsPrivate
{
  //! \brief The diagnostics updater used for the nodelet.
  ::std::shared_ptr<::cras::DiagnosticUpdater> updater;
  
  //! \brief Log helper.
  ::cras::LogHelperPtr log;

  //! \brief The diagnostics updater timer.
  ::ros::Timer timer;
};

}

template <typename NodeletType>
NodeletWithDiagnostics<NodeletType>::NodeletWithDiagnostics() : data(new ::cras::impl::NodeletWithDiagnosticsPrivate)
{
}

template <typename NodeletType>
NodeletWithDiagnostics<NodeletType>::~NodeletWithDiagnostics()
{
}

template <typename NodeletType>
::cras::DiagnosticUpdater& NodeletWithDiagnostics<NodeletType>::getDiagUpdater(const bool forceNew) const
{
  if (this->data->updater == nullptr || forceNew)
  {
    const auto* nodelet = dynamic_cast<const ::nodelet::Nodelet*>(this);
    if (nodelet != nullptr)
    {
      this->data->updater = ::std::make_shared<::cras::DiagnosticUpdater>(
        // TODO if NodeletType::getNodeHandle() is used as first argument, we get a segfault on nodelet unload
        ::ros::NodeHandle(), NodeletType::getPrivateNodeHandle(), NodeletType::getName());
    }
    else
    {
      this->data->updater = ::std::make_shared<::cras::DiagnosticUpdater>();
    }
  }
  return *this->data->updater;
}

template <typename NodeletType>
void NodeletWithDiagnostics<NodeletType>::startDiagTimer() const
{
  this->startDiagTimer(this->getNodeHandle());
}

template <typename NodeletType>
void NodeletWithDiagnostics<NodeletType>::startDiagTimer(const ::ros::NodeHandle& nh) const
{
  this->data->timer = nh.createTimer(::ros::Duration(1.0),
    [this](const ::ros::TimerEvent&) { this->getDiagUpdater().update(); });
}

template <typename NodeletType>
void NodeletWithDiagnostics<NodeletType>::stopDiagTimer() const
{
  this->data->timer.stop();
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  auto pub = publisherNh.template advertise<Message>(topic, queueSize, latch);
  auto params = this->getDiagParams(diagNh, diagNamespace, topic);
  auto result = ::std::make_unique<::cras::DiagnosedPublisher<Message>>(pub, params, defaultDiagParams);
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  ::ros::AdvertiseOptions& options)
{
  auto topic = options.topic;
  auto pub = publisherNh.advertise(options);
  auto params = this->getDiagParams(diagNh, diagNamespace, topic);
  auto result = ::std::make_unique<::cras::DiagnosedPublisher<Message>>(pub, params, defaultDiagParams);
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh,
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, this->getDefaultDiagNh(publisherNh, diagNamespace), defaultDiagParams, diagNamespace,
    topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh,
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  ::ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, this->getDefaultDiagNh(publisherNh, diagNamespace), defaultDiagParams, diagNamespace, options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    this->getNodeHandle(), defaultDiagParams, diagNamespace, topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  ::ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    this->getNodeHandle(), defaultDiagParams, diagNamespace, options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh, const ::std::string& diagNamespace,
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, diagNh, {}, diagNamespace, topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh, const ::std::string& diagNamespace,
  ::ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(publisherNh, diagNh, {}, diagNamespace, options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, const ::std::string& diagNamespace,
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, this->getDefaultDiagNh(publisherNh, diagNamespace), diagNamespace, topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, const ::std::string& diagNamespace, ::ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, this->getDefaultDiagNh(publisherNh, diagNamespace), diagNamespace, options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  const ::std::string& diagNamespace, const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    this->getNodeHandle(), diagNamespace, topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  const ::std::string& diagNamespace, ::ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(this->getNodeHandle(), diagNamespace, options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh,
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, diagNh, "", topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh, ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(publisherNh, diagNh, "", options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, this->getDefaultDiagNh(publisherNh, ""), topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ::ros::NodeHandle publisherNh, ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    publisherNh, this->getDefaultDiagNh(publisherNh, ""), options));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  const ::std::string& topic, size_t queueSize, const bool latch)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(
    this->getNodeHandle(), topic, queueSize, latch));
}

template <typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedPublisher<Message>> NodeletWithDiagnostics<NodeletType>::advertiseDiagnosed(
  ros::AdvertiseOptions& options)
{
  return ::std::move(this->template advertiseDiagnosed<Message>(this->getNodeHandle(), options));
}

template <typename NodeletType>
template<typename T>
::std::unique_ptr<::cras::DiagnosedPublisher<T>> NodeletWithDiagnostics<NodeletType>::createDiagnosedPublisher(
  ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
  const ::ros::Rate& defaultRate, const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate)
{
  return ::std::move(this->template advertiseDiagnosed<T>(
    nh, this->getPrivateNodeHandle(), {defaultMinRate, defaultMaxRate}, paramNamespace, topic, queueSize, false));
}

template <typename NodeletType>
template<typename T>
::std::unique_ptr<::cras::DiagnosedPublisher<T>> NodeletWithDiagnostics<NodeletType>::createDiagnosedPublisher(
  ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
  const ::ros::Rate& defaultRate)
{
  return ::std::move(this->template advertiseDiagnosed<T>(
    nh, this->getPrivateNodeHandle(), {defaultRate, defaultRate}, paramNamespace, topic, queueSize, false));
}

template <typename NodeletType>
template<typename T>
::std::unique_ptr<::cras::DiagnosedPublisher<T>> NodeletWithDiagnostics<NodeletType>::createDiagnosedPublisher(
  ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace)
{
  return ::std::move(this->template advertiseDiagnosed<T>(
    nh, this->getPrivateNodeHandle(), {}, paramNamespace, topic, queueSize, false));
}


template<typename NodeletType>
template<typename M, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(
  ::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size, void (* cb)(M), ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
    new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
      subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
      topic, queue_size, cb, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template <typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size,
  const ::boost::function<void (const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>(new ::cras::DiagnosedSubscriber<Message>(
    subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
    topic, queue_size, cb, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template <typename C, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size,
  const ::boost::function<void (C)>& cb, ::ros::VoidConstPtr obj, ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>(
    new ::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>(
      subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
      topic, queue_size, cb, obj, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template<typename M, class T, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), T* obj, ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
    new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
      subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
      topic, queue_size, cb, obj, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template<typename M, class T, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, T* obj, ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
    new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
      subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
      topic, queue_size, cb, obj, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template<typename M, class T, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size,
  void(T::*cb)(M), const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
    new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
      subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
      topic, queue_size, cb, obj, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template<typename M, class T, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
  const ::std::string& topic, uint32_t queue_size,
  void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
    new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
      subscriberNh, this->getDiagParams(diagNh, diagNamespace, topic), defaultDiagParams,
      topic, queue_size, cb, obj, hints));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
template<typename Message, typename Enable>
::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
NodeletWithDiagnostics<NodeletType>::subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
  const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
  ::ros::SubscribeOptions& options)
{
  auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>(new ::cras::DiagnosedSubscriber<Message>(
    subscriberNh, this->getDiagParams(diagNh, diagNamespace, options.topic), defaultDiagParams, options));
  result->attach(this->getDiagUpdater());
  return ::std::move(result);
}

template<typename NodeletType>
::ros::NodeHandle NodeletWithDiagnostics<NodeletType>::getDefaultDiagNh(
  const ::ros::NodeHandle& pubSubNh, const ::std::string& diagNamespace)
{
  if (diagNamespace.empty() || diagNamespace[0] == '~')
    return this->getPrivateNodeHandle();
  return pubSubNh;
}

template <typename NodeletType>
::cras::BoundParamHelperPtr NodeletWithDiagnostics<NodeletType>::getDiagParams(
  const ::ros::NodeHandle& nh, const ::std::string& diagNamespace, const ::std::string& topic)
{
  if (this->data->log == nullptr)
  {
    // if param helper is set up for this nodelet, use it to get the better logging using NODELET_ macros
    auto* params = dynamic_cast<::cras::NodeletParamHelper<NodeletType>*>(this);
    if (params != nullptr)
      this->data->log = params->getLogger();
    else
      this->data->log = ::std::make_shared<::cras::NodeletLogHelper>(
        ::std::bind(&NodeletWithDiagnostics<NodeletType>::getName, this));
  }

  auto paramAdapter = ::std::make_shared<::cras::NodeHandleGetParamAdapter>(nh);
  auto params = ::std::make_shared<::cras::BoundParamHelper>(this->data->log, paramAdapter);
  
  if (!diagNamespace.empty())
    return params->paramsInNamespace(diagNamespace[0] == '~' ? diagNamespace.substr(1) : diagNamespace);

  ::std::shared_ptr<::cras::GetParamAdapter> adapter;
  // If the topic is absolute, we want to take it as is, so we construct the XmlRpcValueGetParamAdapter for all
  // parameters. This may be a little costly, but there's probably no better way.
  if (topic[0] == '/')
  {
    ::XmlRpc::XmlRpcValue xml;
    xml.begin();  // morph xml into a struct
    xml = params->getParam("/", xml, "", {false});
    adapter = ::std::make_shared<::cras::XmlRpcValueGetParamAdapter>(xml, "/");
  }
  else
  {
    // Find a suitable parent namespace. Names are remapped until this namespace, and the rest (topic name) is not
    // remapped.
    const auto resolvedParentNs = ::ros::names::resolve(nh.resolveName(""));
    ::XmlRpc::XmlRpcValue xml;
    xml.begin();  // morph xml into a struct
    xml = params->getParam(resolvedParentNs, xml, "", {false});
    adapter = ::std::make_shared<::cras::XmlRpcValueGetParamAdapter>(xml, resolvedParentNs);
  }
  
  // Construct the final param adapter for the topic namespace, falling back to an empty adapter if the namespace does
  // not exist.
  const auto cleanTopic = ::cras::stripLeadingSlash(topic);
  ::std::shared_ptr<::cras::GetParamAdapter> topicAdapter;
  if (adapter->hasParam(cleanTopic))
  {
    topicAdapter = adapter->getNamespaced(cleanTopic);
  }
  else
  {
    ::XmlRpc::XmlRpcValue xml;
    xml.begin();  // morph xml into a struct
    auto topicNs = ::ros::names::append(adapter->getNamespace(), cleanTopic);
    topicAdapter = ::std::make_shared<::cras::XmlRpcValueGetParamAdapter>(xml, topicNs);
  }

  return ::std::make_shared<::cras::BoundParamHelper>(this->data->log, topicAdapter);
}

}