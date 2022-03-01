/**
 * \file
 * \brief Helpers for setting up diagnostics for nodelets.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <memory>
#include <string>
#include <type_traits>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/advertise_options.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/diag_utils/updater.h>
#include <cras_cpp_common/message_utils.hpp>

namespace cras
{

namespace impl
{
// forward declaration
struct NodeletWithDiagnosticsPrivate;
}

/**
 * \brief Nodelet mixin that provides helper functions for running a diagnostics updater.
 * \tparam NodeletType Type of the base nodelet.
 * 
 * Methods `advertiseDiagnosed()` and `subscribeDiagnosed()` offer a versatile API based on the API of NodeHandle.
 * Additional required arguments like node handles or diagnostic task configuration are prepended to the list of
 * usual arguments of `advertise()`/`subscribe()`. All of these additional arguments are optional (which is achieved
 * by providing a lot of overrides of the functions).
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
 * The parameter namespace from which the parameters are read is determined as follows:
 * - If `diagNh` is missing and `diagNamespace` is empty or missing, the namespace is `~topic`.
 * - If `diagNh` is missing and `diagNamespace` is nonempty and starts with ~, the namespace is `~diagNamespace`.
 * - If `diagNh` is missing and `diagNamespace` is nonempty and does not start with ~, the namespace is 
 *   `pubNh/diagNamespace`.
 * - If `diagNh` is specified and `diagNamespace` is empty or missing, the namespace is `diagNh/topic`.
 * - If `diagNh` is specified and `diagNamespace` nonempty, the namespace is `diagNh/diagNamespace`.
 * 
 * Whenever the topic name is used in the topic diagnostic configuration namespace, the name of the topic is not
 * affected by topic remappings passed to the nodelet (but its parent namespaces are affected).
 * 
 * Here are a few examples:
 *   <pre>
 *     // nodelet name is "nodelet"
 *     ros::NodeHandle nh = this->getNodeHandle();
 *     ros::NodeHandle pnh = this->getPrivateNodeHandle();
 *     ros::NodeHandle tnh(nh, "test");
 *     ros::NodeHandle rnh(nh, "", {{"/topic", "/topic2"}});  // remap /topic->/topic2
 *     
 *     this->advertiseDiagnosed(nh, "topic", 10);  // param namespace is /nodelet/topic
 *     this->advertiseDiagnosed(nh, "/topic", 10);  // param namespace is /topic
 *     this->advertiseDiagnosed(nh, "test", "topic", 10);  // param namespace is /test
 *     this->advertiseDiagnosed(nh, "~test", "topic", 10);  // param namespace is /nodelet/test
 *     this->advertiseDiagnosed(nh, pnh, "topic", 10);  // param namespace is /nodelet/topic
 *     this->advertiseDiagnosed(nh, pnh, "test", "topic", 10);  // param namespace is /nodelet/test
 *     this->advertiseDiagnosed(nh, pnh, "~test", "topic", 10);  // param namespace is /nodelet/test
 *     this->advertiseDiagnosed(pnh, "topic", 10);  // param namespace is /nodelet/topic
 *     this->advertiseDiagnosed(pnh, "/topic", 10);  // param namespace is /topic
 *     this->advertiseDiagnosed(pnh, "test", "topic", 10);  // param namespace is /nodelet/test
 *     this->advertiseDiagnosed(pnh, "~test", "topic", 10);  // param namespace is /nodelet/test
 *     this->advertiseDiagnosed(pnh, tnh, "topic", 10);  // param namespace is /test/topic
 *     this->advertiseDiagnosed(pnh, tnh, "test", "topic", 10);  // param namespace is /test/test
 *     this->advertiseDiagnosed(pnh, tnh, "~test", "topic", 10);  // param namespace is /test/test
 *     this->advertiseDiagnosed(tnh, "topic", 10);  // param namespace is /test/topic
 *     this->advertiseDiagnosed(tnh, "/topic", 10);  // param namespace is /topic
 *     this->advertiseDiagnosed(tnh, "test", "topic", 10);  // param namespace is /test/test
 *     this->advertiseDiagnosed(tnh, "~test", "topic", 10);  // param namespace is /test/test
 *     this->advertiseDiagnosed(rnh, "topic", 10);  // subscribes /topic2, param namespace is /nodelet/topic
 *     this->advertiseDiagnosed(rnh, "/topic", 10);  // subscribes /topic2, param namespace is /topic
 *   </pre>
 */
template <typename NodeletType>
struct NodeletWithDiagnostics : public virtual NodeletType
{
public:
  NodeletWithDiagnostics();
  virtual ~NodeletWithDiagnostics();  // we need to be polymorphic
  
protected:
  /**
   * \brief Get a diagnostic updater to be used with this nodelet.
   * \param[in] forceNew If true, force creating a new updater, otherwise reuse the previously created one if it exists.
   * \return The updater.
   */
  ::cras::DiagnosticUpdater& getDiagUpdater(bool forceNew = false) const;
  
  /**
   * \brief Start periodic updates of the diagnostics updater.
   */
  void startDiagTimer() const;
  
  /**
   * \brief Start periodic updates of the diagnostics updater.
   * \param[in] nh The node handle on which the timer should be started.
   */
  void startDiagTimer(const ::ros::NodeHandle& nh) const;
  
  /**
   * \brief Stop the automatic updates of the diagnostic updater.
   */
  void stopDiagTimer() const;

  ///////////////////////
  // ADVERTISE METHODS //
  ///////////////////////
  
  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in,out] options Topic advertise options.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    ::ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in] topic The topic to subscribe to.
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in,out] options Topic advertise options.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    ::ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in] topic The topic to subscribe to (relative to public node handle of the nodelet).
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in,out] options Topic advertise options (topic is relative to public node handle of the nodelet).
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    ::ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in] topic The topic to subscribe to (relative to public node handle of the nodelet).
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh, const ::std::string& diagNamespace,
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in,out] options Topic advertise options (topic is relative to public node handle of the nodelet).
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh, const ::std::string& diagNamespace,
    ::ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in] topic The topic to subscribe to.
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, const ::std::string& diagNamespace,
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in,out] options Topic advertise options.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, const ::std::string& diagNamespace, ::ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in] topic The topic to subscribe to (relative to public node handle of the nodelet).
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    const ::std::string& diagNamespace, const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched
   *                          (using the private node handle of the nodelet).
   * \param[in,out] options Topic advertise options (topic is relative to public node handle of the nodelet).
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    const ::std::string& diagNamespace, ::ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task (namespace is same as topic).
   * \param[in] topic The topic to subscribe to.
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh,
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task (namespace is same as topic).
   * \param[in,out] options Topic advertise options.
   * \return The publisher object.
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ::ros::NodeHandle diagNh, ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   * \note Diagnostic task config is read from ~topic namespace (with `topic` not remapped).
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] publisherNh Node handle to use for setting up the publisher.
   * \param[in,out] options Topic advertise options.
   * \return The publisher object.
   * \note Diagnostic task config is read from ~topic namespace (with `topic` not remapped).
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    ::ros::NodeHandle publisherNh, ros::AdvertiseOptions& options);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in] topic The topic to subscribe to (relative to public node handle of the nodelet).
   * \param[in] queueSize Size of the subscription queue.
   * \param[in] latch Whether the topic should be latched.
   * \return The publisher object.
   * \note Diagnostic task config is read from ~topic namespace (with `topic` not remapped).
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(
    const ::std::string& topic, size_t queueSize, bool latch = false);

  /**
   * \brief Advertise a topic and setup up an automatic topic diagnostic task for it.
   * \tparam Message The published message type.
   * \tparam Enable SFINAE only. Do not explicitly set.
   * \param[in,out] options Topic advertise options.
   * \return The publisher object.
   * \note Diagnostic task config is read from ~topic namespace (with `topic` not remapped).
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedPublisher<Message>> advertiseDiagnosed(ros::AdvertiseOptions& options);
  
  ////////////////
  // DEPRECATED //
  ////////////////
  
  /**
   * \brief Create a diagnosed publisher for a message type without header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size.
   * \param[in] defaultRate Default expected rate.
   * \param[in] defaultMinRate Default minimum rate.
   * \param[in] defaultMaxRate Default maximum rate.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  [[deprecated("Use advertiseDiagnosed() instead")]]
  ::std::unique_ptr<::cras::DiagnosedPublisher<T>> createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
    const ::ros::Rate& defaultRate, const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate);

  /**
   * \brief Create a diagnosed publisher for a message type with header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size,
   *                           delay/min, delay/max.
   * \param[in] defaultRate Default expected rate.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  [[deprecated("Use advertiseDiagnosed() instead")]]
  ::std::unique_ptr<::cras::DiagnosedPublisher<T>> createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
    const ::ros::Rate& defaultRate);

  /**
   * \brief Create a diagnosed publisher for a message type with header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size,
   *                           delay/min, delay/max.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  [[deprecated("Use advertiseDiagnosed() instead")]]
  ::std::unique_ptr<::cras::DiagnosedPublisher<T>> createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace);

  ///////////////////////
  // SUBSCRIBE METHODS //
  ///////////////////////
  
  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename M, typename Enable = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(*cb)(M), ::ros::TransportHints hints = {});
  
  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    const ::boost::function<void (const ::boost::shared_ptr<Message>&)>& cb, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam C Signature of the callback.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj Tracked object.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template <typename C, typename Enable = ::std::enable_if_t<::cras::IsMessageParam<C>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<C>>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<C>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    const ::boost::function<void (C)>& cb, ::ros::VoidConstPtr obj = {}, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename Enable = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M), T* obj, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename Enable = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size, void(T::*cb)(M) const, T* obj, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename Enable = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M), const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam M Signature of the callback.
   * \tparam T Type of the object on which the callback will be called.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in] topic The topic to subscribe to.
   * \param[in] queue_size Size of the subscription queue.
   * \param[in] cb The callback to call when a message is received.
   * \param[in] obj The object to call the callback on.
   * \param[in] hints Connection hints.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename M, class T, typename Enable = ::std::enable_if_t<::cras::IsMessageParam<M>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams, const ::std::string& diagNamespace,
    const ::std::string& topic, uint32_t queue_size,
    void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj, ::ros::TransportHints hints = {});

  /**
   * \brief Subscribe to the given topic, automatically updating the diagnostic task every time a message is received.
   * \tparam Message Type of the message to subscribe to.
   * \param[in] subscriberNh Node handle to use for setting up the subscriber.
   * \param[in] diagNh Node handle to use for reading parameters of the diagnostic task.
   * \param[in] defaultDiagParams Default parameters of the diagnostic task.
   * \param[in] diagNamespace Parameter namespace in which the parameters for the diagnostic task will be searched.
   * \param[in,out] options Subscription options.
   * \return The subscriber object. Keep it alive whole time the subscription should be active!
   */
  template<typename Message, typename Enable = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
  ::std::unique_ptr<::cras::DiagnosedSubscriber<Message>>
  subscribeDiagnosed(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh,
    const ::cras::SimpleTopicStatusParam<Message>& defaultDiagParams, const ::std::string& diagNamespace,
    ::ros::SubscribeOptions& options);

  // The following macros create a cartesian product of overloads for all the optional options. The defined functions
  // are the same as above, but with a part of parameters before `topic` missing.
  
#define CRAS_NODELET_DIAG_GENERATE_OVERLOAD_M(NH, NH2, PARAM, PARAM2, CB, CB2) \
  template <typename M, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>> \
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>> \
  subscribeDiagnosed(NH PARAM const ::std::string& topic, uint32_t queue_size, CB, ::ros::TransportHints hints = {}) \
  { \
     return ::std::move(this->template subscribeDiagnosed<M>(NH2 PARAM2 topic, queue_size, CB2, hints));\
  }

#define CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MESSAGE(NH, NH2, PARAM, PARAM2, CB) \
  template <typename M, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<M>::value>> \
  ::std::unique_ptr<::cras::DiagnosedSubscriber<M>> \
  subscribeDiagnosed(NH PARAM const ::std::string& topic, uint32_t queue_size, CB, ::ros::TransportHints hints = {}) \
  { \
     return ::std::move(this->template subscribeDiagnosed<M>(NH2 PARAM2 topic, queue_size, cb, hints));\
  }

#define CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MT(NH, NH2, PARAM, PARAM2, CB) \
  template <typename M, class T, typename = ::std::enable_if_t<::cras::IsMessageParam<M>::value>> \
  ::std::unique_ptr<::cras::DiagnosedSubscriber<::cras::BaseMessage<M>>> \
  subscribeDiagnosed(NH PARAM const ::std::string& topic, uint32_t queue_size, CB, ::ros::TransportHints hints = {}) \
  { \
     return ::std::move(this->template subscribeDiagnosed<M, T>(NH2 PARAM2 topic, queue_size, cb, obj, hints)); \
  }

#define CRAS_NODELET_DIAG_GENERATE_OVERLOAD_OPTIONS(NH, NH2, PARAM, PARAM2) \
  template<typename M, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<M>::value>> \
  ::std::unique_ptr<::cras::DiagnosedSubscriber<M>> \
  subscribeDiagnosed(NH PARAM ::ros::SubscribeOptions& options) \
  { \
     const auto topic = options.topic; \
     return ::std::move(this->template subscribeDiagnosed<M>(NH2 PARAM2 options));\
  }

#define CRAS_SINGLE_ARG(...) __VA_ARGS__

#define CRAS_NODELET_DIAG_GENERATE_OVERLOADS(NH, NH2, PARAM, PARAM2) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_M(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), CRAS_SINGLE_ARG(void(*cb)(M)), CRAS_SINGLE_ARG(cb)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MESSAGE(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), \
    CRAS_SINGLE_ARG(const ::boost::function<void (const ::boost::shared_ptr<M>&)>& cb)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_M(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), \
    CRAS_SINGLE_ARG(const ::boost::function<void (M)>& cb, ::ros::VoidConstPtr obj = {}), \
    CRAS_SINGLE_ARG(cb, obj)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MT(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), CRAS_SINGLE_ARG(void(T::*cb)(M), T* obj)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MT(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), CRAS_SINGLE_ARG(void(T::*cb)(M) const, T* obj)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MT(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), \
    CRAS_SINGLE_ARG(void(T::*cb)(M), const ::boost::shared_ptr<T>& obj)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_MT(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2), \
    CRAS_SINGLE_ARG(void(T::*cb)(M) const, const ::boost::shared_ptr<T>& obj)) \
  CRAS_NODELET_DIAG_GENERATE_OVERLOAD_OPTIONS(CRAS_SINGLE_ARG(NH), CRAS_SINGLE_ARG(NH2), \
    CRAS_SINGLE_ARG(PARAM), CRAS_SINGLE_ARG(PARAM2))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(::ros::NodeHandle subscriberNh, ),
    CRAS_SINGLE_ARG(subscriberNh, this->getDefaultDiagNh(subscriberNh, diagNamespace), ),
    CRAS_SINGLE_ARG(const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams,
      const ::std::string& diagNamespace, ),
    CRAS_SINGLE_ARG(defaultDiagParams, diagNamespace, ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(),
    CRAS_SINGLE_ARG(this->getNodeHandle(), ),
    CRAS_SINGLE_ARG(const ::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>& defaultDiagParams,
      const ::std::string& diagNamespace, ),
    CRAS_SINGLE_ARG(defaultDiagParams, diagNamespace, ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh, ),
    CRAS_SINGLE_ARG(subscriberNh, diagNh, ),
    CRAS_SINGLE_ARG(const ::std::string& diagNamespace, ),
    CRAS_SINGLE_ARG(::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(), diagNamespace, ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(::ros::NodeHandle subscriberNh, ),
    CRAS_SINGLE_ARG(subscriberNh, this->getDefaultDiagNh(subscriberNh, diagNamespace), ),
    CRAS_SINGLE_ARG(const ::std::string& diagNamespace, ),
    CRAS_SINGLE_ARG(::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(), diagNamespace, ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(),
    CRAS_SINGLE_ARG(this->getNodeHandle(), ),
    CRAS_SINGLE_ARG(const ::std::string& diagNamespace, ),
    CRAS_SINGLE_ARG(::cras::SimpleTopicStatusParam<::cras::BaseMessage<M>>(), diagNamespace, ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(::ros::NodeHandle subscriberNh, ::ros::NodeHandle diagNh, ),
    CRAS_SINGLE_ARG(subscriberNh, diagNh, ),
    CRAS_SINGLE_ARG(),
    CRAS_SINGLE_ARG("", ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(::ros::NodeHandle subscriberNh, ),
    CRAS_SINGLE_ARG(subscriberNh, this->getDefaultDiagNh(subscriberNh, ""), ),
    CRAS_SINGLE_ARG(),
    CRAS_SINGLE_ARG("", ))

  CRAS_NODELET_DIAG_GENERATE_OVERLOADS(
    CRAS_SINGLE_ARG(),
    CRAS_SINGLE_ARG(this->getNodeHandle(), ),
    CRAS_SINGLE_ARG(),
    CRAS_SINGLE_ARG("", ))

  using NodeletType::getName;
  
protected:
  /**
   * \brief Get the parameters configuring a diagnosed publisher or subscriber.
   * \param[in] nh Node handle of the publisher or subscriber.
   * \param[in] diagNamespace Parameter sub-namespace inside the node handle's namespace.
   * \param[in] topic Name of the diagnosed topic.
   * \return Param helper with the parameters.
   */
  ::cras::BoundParamHelperPtr getDiagParams(
    const ::ros::NodeHandle& nh, const ::std::string& diagNamespace, const ::std::string& topic);

  /**
   * \brief Get the default node handle for reading diagnostic task configuration.
   * \param[in] pubSubNh The node handle used for creating the publisher/subscriber.
   * \param[in] diagNamespace The explicit namespace.
   * \return The node handle.
   */
  ::ros::NodeHandle getDefaultDiagNh(const ::ros::NodeHandle& pubSubNh, const ::std::string& diagNamespace);
  
private:
  //! \brief PIMPL
  ::std::unique_ptr<::cras::impl::NodeletWithDiagnosticsPrivate> data;
};

}

#include "impl/nodelet_with_diagnostics.hpp"