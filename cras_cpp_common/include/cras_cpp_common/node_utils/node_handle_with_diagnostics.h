#pragma once

/**
 * \file
 * \brief Utils for adding diagnostics to a topic via node handle.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/advertise_options.h>
#include <ros/datatypes.h>
#include <ros/forwards.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>

#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/message_utils.hpp>
#include <cras_cpp_common/node_utils/param_helper.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>

namespace cras
{

/**
 * \brief Utils for adding diagnostics to a topic via node handle.
 *
 * The publisher and subscriber diagnostic tasks can be configured from ROS parameters. The following parameters
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
 * The parameter namespace from which they are configured is determined as follows:
 *
 * - If no explicit namespace is specified, the topic name is used.
 *   - Name remapping is done only "until" the topic name level. I.e. node name or parent namespaces are affected by
 *     remappings, but the topic name is not. If the topic name is absolute, it is taken as is without any remappings.
 *   - Explicit namespaces are remapped as normal ROS parameters.
 * - If the namespace starts with ~ or is not specified (topic name is used), the private node namespace (node name) is
 *   prepended.
 *   - If this node handle already represents a private node handle, no additional namespace is prepended.
 *
 * Here are a few examples:
 *   <pre>
 *     // node name is "diag_utils"
 *     cras::NodeHandleWithDiagnostics nh;
 *     cras::NodeHandleWithDiagnostics pnh("~");
 *     cras::NodeHandleWithDiagnostics tnh("test");
 *     cras::NodeHandleWithDiagnostics rnh("", {{"/topic", "/topic2"}});  // remap /topic->/topic2
 *
 *     nh.advertiseDiagnosed(updater, "topic", 10);  // param namespace is /diag_utils/topic
 *     nh.advertiseDiagnosed(updater, "/topic", 10);  // param namespace is /topic
 *     nh.advertiseDiagnosed(updater, "test", "topic", 10);  // param namespace is /test
 *     nh.advertiseDiagnosed(updater, "~test", "topic", 10);  // param namespace is /diag_utils/test
 *     pnh.advertiseDiagnosed(updater, "topic", 10);  // param namespace is /diag_utils/topic
 *     pnh.advertiseDiagnosed(updater, "/topic", 10);  // param namespace is /topic
 *     pnh.advertiseDiagnosed(updater, "test", "topic", 10);  // param namespace is /diag_utils/test
 *     pnh.advertiseDiagnosed(updater, "~test", "topic", 10);  // param namespace is /diag_utils/test
 *     tnh.advertiseDiagnosed(updater, "topic", 10);  // param namespace is /test/topic
 *     tnh.advertiseDiagnosed(updater, "/topic", 10);  // param namespace is /topic
 *     tnh.advertiseDiagnosed(updater, "test", "topic", 10);  // param namespace is /test/test
 *     tnh.advertiseDiagnosed(updater, "~test", "topic", 10);  // param namespace is /test/test
 *     rnh.advertiseDiagnosed(updater, "topic", 10);  // subscribes /topic2, param namespace is /diag_utils/topic
 *     rnh.advertiseDiagnosed(updater, "/topic", 10);  // subscribes /topic2, param namespace is /topic
 *   </pre>
 */
class NodeHandleWithDiagnostics : public ::cras::NodeParamHelper
{
public:
  /**
   * \brief Create the node handle using the passed ROS node handle parameters
   * \param[in] ns Namespace of the handle.
   * \param[in] remappings Remappings.
   */
  explicit NodeHandleWithDiagnostics(const ::std::string& ns = "", const ::ros::M_string& remappings = {});

  /**
   * \brief Create the node handle using the passed ROS node handle parameters
   * \param[in] parent Parent node handle.
   * \param[in] ns Namespace of the handle.
   */
  NodeHandleWithDiagnostics(const ::ros::NodeHandle& parent, const ::std::string& ns);

  /**
   * \brief Create the node handle using the passed ROS node handle parameters
   * \param[in] parent Parent node handle.
   * \param[in] ns Namespace of the handle.
   * \param[in] remappings Remappings.
   */
  NodeHandleWithDiagnostics(const ::ros::NodeHandle& parent, const ::std::string& ns,
    const ::ros::M_string& remappings);

protected:
  /**
   * \brief Optionally add a private node namespace to the given topic if this nodehandle is in the root node namespace.
   * \param[in] ns The namespace to prefix.
   * \return The prefix namespace.
   */
  virtual ::std::string prefixDiagNamespace(const ::std::string& ns) const;

  /**
   * \brief Get the param helper from which parameters for the diagnostic task can be extracted.
   * \param[in] diagNs Namespace of the diagnostic task. Pass empty string if you want to search a parameter namespace
   *                   based on the topic name.
   * \param[in] topic Name of the diagnosed topic which will be used as a default parameter namespace.
   * \return The param helper.
   */
  virtual ::cras::BoundParamHelperPtr getDiagParams(const ::std::string& diagNs, const ::std::string& topic) const;

public:
  ///////////////////////
  // ADVERTISE METHODS //
  ///////////////////////

  /**
   * \brief Advertise publication of a message, automatically adding diagnostics to it.
   * \tparam Message Type of the message to be published.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to advertise and diagnose.
   * \param[in] queueSize Size of the publishing queue.
   * \param[in] latch Whether to latch the publication.
   * \return A publisher object whose `publish()` method has to be used to correctly automate the diagnostic task
   *         updating.
   */
  template <class Message>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>>
  advertiseDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, const size_t queueSize, const bool latch = false)
  {
    auto pub = this->template advertise<Message>(topic, queueSize, latch);
    auto result = ::std::make_unique<::cras::DiagnosedPublisher<Message>>(
      pub, this->getDiagParams(diagNamespace, topic), defaultDiagParams);
    result->attach(updater);
    return result;
  }

  /**
   * \brief Advertise publication of a message, automatically adding diagnostics to it.
   * \tparam Message Type of the message to be published.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in,out] options Advertise options that configure the publisher.
   * \return A publisher object whose `publish()` method has to be used to correctly automate the diagnostic task
   *         updating.
   */
  template <class Message>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>>
  advertiseDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    ::ros::AdvertiseOptions& options)
  {
    const auto topic = options.topic;  // make copy of topic before advertise has a chance to change it
    auto pub = this->advertise(options);
    auto result = ::std::make_unique<::cras::DiagnosedPublisher<Message>>(
      pub, this->getDiagParams(diagNamespace, topic), defaultDiagParams);
    result->attach(updater);
    return result;
  }

  /**
   * \brief Advertise publication of a message, automatically adding diagnostics to it.
   * \tparam Message Type of the message to be published.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to advertise and diagnose.
   * \param[in] queueSize Size of the publishing queue.
   * \param[in] latch Whether to latch the publication.
   * \return A publisher object whose `publish()` method has to be used to correctly automate the diagnostic task
   *         updating.
   */
  template <class Message>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>>
  advertiseDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, const size_t queueSize, const bool latch = false)
  {
    return this->template advertiseDiagnosed<Message>(updater, ::cras::SimpleTopicStatusParam<Message>(), diagNamespace,
      topic, queueSize, latch);
  }

  /**
   * \brief Advertise publication of a message, automatically adding diagnostics to it.
   * \tparam Message Type of the message to be published.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in,out] options Advertise options that configure the publisher.
   * \return A publisher object whose `publish()` method has to be used to correctly automate the diagnostic task
   *         updating.
   */
  template <class Message>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>>
  advertiseDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    ::ros::AdvertiseOptions& options)
  {
    return this->template advertiseDiagnosed<Message>(updater, ::cras::SimpleTopicStatusParam<Message>(), diagNamespace,
      options);
  }

  /**
   * \brief Advertise publication of a message, automatically adding diagnostics to it.
   * \tparam Message Type of the message to be published.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to advertise and diagnose.
   * \param[in] queueSize Size of the publishing queue.
   * \param[in] latch Whether to latch the publication.
   * \return A publisher object whose `publish()` method has to be used to correctly automate the diagnostic task
   *         updating.
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template <class Message>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>>
  advertiseDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, const size_t queueSize, const bool latch = false)
  {
    return this->template advertiseDiagnosed<Message>(updater, "", topic, queueSize, latch);
  }

  /**
   * \brief Advertise publication of a message, automatically adding diagnostics to it.
   * \tparam Message Type of the message to be published.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in,out] options Advertise options that configure the publisher.
   * \return A publisher object whose `publish()` method has to be used to correctly automate the diagnostic task
   *         updating.
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template <class Message>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>>
  advertiseDiagnosed(::diagnostic_updater::Updater& updater, ::ros::AdvertiseOptions& options)
  {
    return this->template advertiseDiagnosed<Message>(updater, "", options);
  }

  ///////////////////////
  // SUBSCRIBE METHODS //
  ///////////////////////

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename M, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(*cb)(M), ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
      new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
        *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    const boost::function<void(const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>(new ::cras::DiagnosedSubscriber<Message>(
      *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam C Signature of the callback.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj Tracked object.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename C, typename = ::std::enable_if_t<::cras::IsMessageParam<C>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    const boost::function<void(C)>& cb, ::ros::VoidConstPtr obj = {}, ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>(
      new ::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>(
        *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, obj, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), T* obj, ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
      new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
        *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, obj, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, T* obj, ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
      new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
        *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, obj, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M), const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
      new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
        *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, obj, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {})
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>(
      new ::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>(
        *this, this->getDiagParams(diagNamespace, topic), defaultDiagParams, topic, queue_size, cb, obj, hints));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in,out] options Subscription options.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    ::ros::SubscribeOptions& options)
  {
    auto result = ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>(new ::cras::DiagnosedSubscriber<Message>(
      *this, this->getDiagParams(diagNamespace, options.topic), defaultDiagParams, options));
    result->attach(updater);
    return ::std::move(result);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename M, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(*cb)(M), ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(),
      diagNamespace, topic, queue_size, cb, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    const boost::function<void(const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<Message>(updater, ::cras::SimpleTopicStatusParam<Message>(), diagNamespace,
      topic, queue_size, cb, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam C Signature of the callback.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj Tracked object.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename C, typename = ::std::enable_if_t<::cras::IsMessageParam<C>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    const boost::function<void(C)>& cb, ::ros::VoidConstPtr obj = {}, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<C>(updater, ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>(),
      diagNamespace, topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), T* obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(),
      diagNamespace, topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, T* obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(),
      diagNamespace, topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M), const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(),
      diagNamespace, topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(),
      diagNamespace, topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in,out] options Subscription options.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, const ::std::string& diagNamespace,
    ::ros::SubscribeOptions& options)
  {
    return this->template subscribeDiagnosed<Message>(updater, ::cras::SimpleTopicStatusParam<Message>(),
      diagNamespace, options);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template <typename M, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size, void(*cb)(M), ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, "", topic, queue_size, cb, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template <typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size,
    const boost::function<void(const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<Message>(updater, "", topic, queue_size, cb, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam C Signature of the callback.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj Tracked object.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename C, typename = ::std::enable_if_t<::cras::IsMessageParam<C>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size,
    const boost::function<void(C)>& cb, ::ros::VoidConstPtr obj = {}, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<C>(updater, "", topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), T* obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, "", topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, T* obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, "", topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M), const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, "", topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template<typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {})
  {
    return this->template subscribeDiagnosed<M>(updater, "", topic, queue_size, cb, obj, hints);
  }

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] updater The diagnostic updater to add the diagnostic task to.
   * \param[in,out] options Subscription options.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   * \note The diagnostic task will search its parameters in parameter namespace `~/topic`.
   */
  template<typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::diagnostic_updater::Updater& updater, ::ros::SubscribeOptions& options)
  {
    return this->template subscribeDiagnosed<Message>(updater, "", options);
  }

protected:
  //! \brief The parent node handle of this one.
  ::ros::NodeHandle parentNh {};
};

}
