#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers.
 * \author Martin Pecka
 */

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <boost/functional.hpp>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscriber.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>

namespace cras
{

/**
 * \brief A lazy subscriber that subscribes only when a condition is met.
 */
class ConditionalSubscriber : public ::cras::HasLogger
{
public:
  //! \brief Type of the function that connects the subscriber.
  //! \param[out] sub Reference to the subscriber object to be created.
  typedef ::std::function<void(::ros::Subscriber& sub)> ConnectFn;

  //! \brief Type of the function that disconnects the subscriber.
  //! \param [in,out] Reference to the subscriber object to be disconnected.
  typedef ::std::function<void(::ros::Subscriber& sub)> DisconnectFn;

  /**
   * \brief Create the conditional subscriber.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object.
   * \param[in] disconnectFn The function that disconnects the subscriber. The `sub` object passed to the function is
   *                         the one created previously by `connectFn`. The passed subscriber should be invalidated.
   * \param[in] logHelper Logging helper.
   * \note The base class itself does not regularly check the connect/disconnect condition. Subclasses are responsible
   *       for calling `updateSubscription()` when it is possible that the subscription condition changed.
   */
  ConditionalSubscriber(ConnectFn connectFn, DisconnectFn disconnectFn,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the conditional subscriber.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object. Disconnection is done by simply calling `sub.shutdown()`.
   * \param[in] logHelper Logging helper.
   * \note The base class itself does not regularly check the connect/disconnect condition. Subclasses are responsible
   *       for calling `updateSubscription()` when it is possible that the subscription condition changed.
   */
  explicit ConditionalSubscriber(ConnectFn connectFn,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Destroy this object and unsubscribe the subscriber if it was subscribed.
   */
  virtual ~ConditionalSubscriber();

  /**
   * \brief Tell whether this subscriber has the lazy behavior enabled.
   * \return Whether lazy behavior is enabled.
   */
  bool isLazy() const;

  /**
   * \brief Set whether the subscriber behaves in the lazy way.
   * \param[in] lazy If set to false, the subscriber is switched to always subscribed mode.
   * \note Calling this function checks the subscription condition and subscribes/unsubscribes as necessary.
   */
  void setLazy(bool lazy);

  /**
   * \brief Whether the subscriber is currently subscribed to its topic or not.
   * \return Whether the subscriber is currently subscribed to its topic or not.
   */
  bool isSubscribed() const;

protected:
  /**
   * \brief Tell whether the subscriber should be subscribed or not.
   * \return Whether the subscriber should be subscribed or not.
   * \note This function does not need to take into account the `lazy` flag.
   * \note This function cannot lock `connectMutex`.
   */
  virtual bool shouldBeSubscribed() const = 0;

  /**
   * \brief The callback called when the condition might have changed.
   * \note This function locks `connectMutex`.
   */
  void updateSubscription();

  /**
   * \brief The callback called when the condition might have changed.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  void updateSubscriptionNoLock();

  /**
   * \brief Connect the subscriber to its input.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  void connectNoLock();

  /**
   * \brief Disconnect the subscriber from its input.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  void disconnectNoLock();

  //! \brief The underlying subscriber (valid only when `subscribed` is true).
  ::ros::Subscriber sub;

  //! \brief Whether the lazy behavior is enabled (if false, the subscriber is always subscribed).
  bool lazy {true};

  //! \brief Whether the subscriber is currently subscribed to its topic or not.
  bool subscribed {false};

  //! \brief The function used to establish connection.
  ConnectFn connectFn;

  //! \brief The function used to close connection.
  DisconnectFn disconnectFn;

  //! \brief Mutex protecting `sub` and `subscribed`.
  mutable ::std::mutex connectMutex;
};

/**
 * \brief Base for lazy subscribers that subscribes only when a paired publisher has subscribers.
 * \tparam PublisherMsgType Type of the publisher messages.
 */
template<typename PublisherMsgType>
class LazySubscriberBase : public ::cras::ConditionalSubscriber
{
public:
  /**
   * \brief Create the lazy subscriber that subscribes only when `publisherTopic` has subscribers.
   * \param[in] publisherNh Node handle used for publisher topic.
   * \param[in] publisherTopic The topic whose number of subscribers decides whether to connect or not.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object.
   * \param[in] disconnectFn The function that disconnects the subscriber. The `sub` object passed to the function is
   *                         the one created previously by `connectFn`. The passed subscriber should be invalidated.
   * \param[in] logHelper Logging helper.
   */
  LazySubscriberBase(::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
    typename ::cras::ConditionalSubscriber::ConnectFn connectFn,
    typename ::cras::ConditionalSubscriber::DisconnectFn disconnectFn,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the lazy subscriber that subscribes only when `publisherTopic` has subscribers.
   * \param[in] publisherNh Node handle used for publisher topic.
   * \param[in] publisherTopic The topic whose number of subscribers decides whether to connect or not.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object. Disconnection of the subscriber is done by simply calling `sub.shutdown()`.
   * \param[in] logHelper Logging helper.
   */
  LazySubscriberBase(::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
    typename ::cras::ConditionalSubscriber::ConnectFn connectFn,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

protected:
  /**
   * \brief The callback called when a new subscriber appears or disappears.
   */
  void connectCb(const ::ros::SingleSubscriberPublisher&);

  bool shouldBeSubscribed() const override;

  //! \brief The publisher whose number of subscribers decides whether to connect or not. It is not used to publish any
  //!        actual messages.
  ::ros::Publisher pub;
};

/**
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers.
 * \tparam PublisherMsgType Type of the publisher messages.
 */
template<typename PublisherMsgType, typename CallbackType = const PublisherMsgType&>
class LazySubscriber : public ::cras::LazySubscriberBase<PublisherMsgType>
{
public:
  /**
   * \brief Create the lazy subscriber that subscribes to `subscriberTopic` when `publisherTopic` has subscribers.
   * \param[in] publisherNh Node handle used for publisher topic.
   * \param[in] publisherTopic The topic whose number of subscribers decides whether to connect or not.
   * \param[in] subscriberNh Node handle used for subscriber topic.
   * \param[in] subscriberTopic The topic to subscribe.
   * \param[in] subscriberQueueSize Queue size for the subscriber.
   * \param[in] subscriberCallback The callback called by the subscriber when a new message is received.
   * \param[in] subscribeOptions Options used when creating the subscriber. Only `allow_concurrent_callbacks` and
   *                             `transport_hints` fields from this object are used.
   * \param[in] logHelper Logging helper.
   */
  LazySubscriber(::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
                 ::ros::NodeHandle subscriberNh, const ::std::string& subscriberTopic, size_t subscriberQueueSize,
                 ::boost::function<void(CallbackType)> subscriberCallback, ::ros::SubscribeOptions subscribeOptions,
                 const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());

  /**
   * \brief Create the lazy subscriber that subscribes to `subscriberTopic` when `publisherTopic` has subscribers.
   * \param[in] publisherNh Node handle used for publisher topic.
   * \param[in] publisherTopic The topic whose number of subscribers decides whether to connect or not.
   * \param[in] subscriberNh Node handle used for subscriber topic.
   * \param[in] subscriberTopic The topic to subscribe.
   * \param[in] subscriberQueueSize Queue size for the subscriber.
   * \param[in] subscriberCallback The callback called by the subscriber when a new message is received.
   * \param[in] logHelper Logging helper.
   */
  LazySubscriber(::ros::NodeHandle publisherNh, const ::std::string& publisherTopic,
                 ::ros::NodeHandle subscriberNh, const ::std::string& subscriberTopic, size_t subscriberQueueSize,
                 ::boost::function<void(CallbackType)> subscriberCallback,
                 const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>());
};

}

#include "impl/lazy_subscriber.hpp"
