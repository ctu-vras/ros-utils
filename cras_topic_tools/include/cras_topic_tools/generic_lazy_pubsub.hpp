#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers. Version for unknown message
 *        type of both the subscriber and the publisher.
 * \author Martin Pecka
 */

#include <memory>
#include <mutex>
#include <string>

#include <boost/functional.hpp>

#include <ros/advertise_options.h>
#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/optional.hpp>

#include <cras_topic_tools/lazy_subscriber.hpp>

namespace cras
{

/**
 * \brief A pair of lazy subscriber and publisher which use the same message type (unknown at compile time).
 * \note If publisher type is known at compile time, use `LazySubscriber`.
 * \note Even though the subscriber is lazy, it has to subscribe to the input topic and receive one message from which
 *       it can derive the publisher parameters. After receiving this first message, the subscriber is disconnected in
 *       case the publisher has no subscribers.
 */
class GenericLazyPubSub : public ::cras::ConditionalSubscriber
{
public:
  //! \brief Signature of the subscriber's callback.
  typedef const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& CallbackMessageType;

  //! \brief Type of the user callback passed to this class.
  typedef ::boost::function<void(::cras::GenericLazyPubSub::CallbackMessageType, ::ros::Publisher&)> CallbackType;

  /**
   * \brief Create the pair of lazy subscriber and publisher.
   * \param[in] nhIn Node handle for message subscription.
   * \param[in] topicIn Input topic (relative to `nhIn`).
   * \param[in] nhOut Node handle for message publication.
   * \param[in] topicOut Output topic (relative to `nhOut`).
   * \param[in] inQueueSize Queue size of the input subscriber.
   * \param[in] outQueueSize Queue size of the output publisher.
   * \param[in] callback The callback to call when a message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::ros::NodeHandle& nhIn, const ::std::string& topicIn,
    const ::ros::NodeHandle& nhOut, const ::std::string& topicOut, size_t inQueueSize, size_t outQueueSize,
    CallbackType callback, ::ros::SubscribeOptions subscribeOptions = {},
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the pair of lazy subscriber and publisher (with equal input and output queue size).
   * \param[in] nhIn Node handle for message subscription.
   * \param[in] topicIn Input topic (relative to `nhIn`).
   * \param[in] nhOut Node handle for message publication.
   * \param[in] topicOut Output topic (relative to `nhOut`).
   * \param[in] queueSize Queue size of the subscriber and publisher.
   * \param[in] callback The callback to call when a message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::ros::NodeHandle& nhIn, const ::std::string& topicIn,
    const ::ros::NodeHandle& nhOut, const ::std::string& topicOut, size_t queueSize, CallbackType callback,
    ::ros::SubscribeOptions subscribeOptions = {},
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the pair of lazy subscriber and publisher (with input and output queue size of 10).
   * \param[in] nhIn Node handle for message subscription.
   * \param[in] topicIn Input topic (relative to `nhIn`).
   * \param[in] nhOut Node handle for message publication.
   * \param[in] topicOut Output topic (relative to `nhOut`).
   * \param[in] callback The callback to call when a message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::ros::NodeHandle& nhIn, const ::std::string& topicIn,
    const ::ros::NodeHandle& nhOut, const ::std::string& topicOut, CallbackType callback,
    ::ros::SubscribeOptions subscribeOptions = {},
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the pair of lazy subscriber and publisher (with the same node handle used for input and output).
   * \param[in] nh Node handle for message subscription and publication.
   * \param[in] topicIn Input topic (relative to `nh`).
   * \param[in] topicOut Output topic (relative to `nh`).
   * \param[in] inQueueSize Queue size of the input subscriber.
   * \param[in] outQueueSize Queue size of the output publisher.
   * \param[in] callback The callback to call when a message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::ros::NodeHandle& nh, const ::std::string& topicIn, const ::std::string& topicOut,
    size_t inQueueSize, size_t outQueueSize, CallbackType callback,
    ::ros::SubscribeOptions subscribeOptions = {},
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the pair of lazy subscriber and publisher (with the same node handle and queue size used for input
   *        and output).
   * \param[in] nh Node handle for message subscription and publication.
   * \param[in] topicIn Input topic (relative to `nh`).
   * \param[in] topicOut Output topic (relative to `nh`).
   * \param[in] queueSize Queue size of the subscriber and publisher.
   * \param[in] callback The callback to call when a message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::ros::NodeHandle& nh, const ::std::string& topicIn, const ::std::string& topicOut,
    size_t queueSize, CallbackType callback, ::ros::SubscribeOptions subscribeOptions = {},
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the pair of lazy subscriber and publisher (with the same node handle used for input and output and
   *        both using queue size of 10).
   * \param[in] nh Node handle for message subscription and publication.
   * \param[in] topicIn Input topic (relative to `nh`).
   * \param[in] topicOut Output topic (relative to `nh`).
   * \param[in] callback The callback to call when a message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::ros::NodeHandle& nh, const ::std::string& topicIn, const ::std::string& topicOut,
    CallbackType callback, ::ros::SubscribeOptions subscribeOptions = {},
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

protected:
  /**
   * \brief Perform the subscription to the input topic.
   * \param[out] sub The subscriber reference to fill with the created subscriber.
   */
  void subscribe(::ros::Subscriber& sub);

  /**
   * \brief Returns true when the subscriber should be connected - i.e. either at the start, or when pub has clients.
   * \return Whether the subscriber should be connected.
   */
  bool shouldBeSubscribed() const override;

  /**
   * \brief Callback that is called whenever someone (un)subscribes from the publisher.
   */
  void connectCb(const ::ros::SingleSubscriberPublisher&);

  /**
   * \brief Callback for the received messages from subscriber. It also handles publisher creation if none exists.
   * \param[in] event The message event.
   */
  void cb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);

  /**
   * \brief Create ros::AdvertiseOptions from a message event.
   * \param[in] event The event to examine.
   * \return The advertise options.
   */
  virtual ::ros::AdvertiseOptions createAdvertiseOptions(
    const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);

  //! \brief Input topic (relative to `nh`).
  ::std::string topicIn;

  //! \brief Output topic (relative to `nh`).
  ::std::string topicOut;

  //! \brief Queue size of the input subscriber.
  size_t inQueueSize;

  //! \brief Queue size of the output publisher.
  size_t outQueueSize;

  //! \brief The output publisher. It will be invalid until first message is received.
  ::ros::Publisher pub;

  //! \brief The input subscriber.
  ::ros::Subscriber sub;

  //! \brief Node handle for topic subscription.
  ::ros::NodeHandle nhIn;

  //! \brief Node handle for topic publication.
  ::ros::NodeHandle nhOut;

  //! \brief The user callback to be called when a message is received.
  ::cras::GenericLazyPubSub::CallbackType callback;

  //! \brief Mutex protecting `pub`.
  ::std::mutex pubCreateMutex;

  //! \brief The options used when the publisher was created. `nullopt` before the publisher is created.
  ::cras::optional<::ros::AdvertiseOptions> advertiseOptions;

  //! \brief Options used when subscribing to the input topic.
  const ::ros::SubscribeOptions subscribeOptions;
};

}
