#pragma once

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <memory>
#include <mutex>

#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscriber.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>

namespace cras
{

/**
 * \brief Base for a lazy subscriber that subscribes only when a condition holds true.
 * \tparam SubscriberType Type of the subscriber created when this object should subscribe. Normally, this would be
 *                        ros::Subscriber, but actually anything can be here if suitable connectFn and disconnectFn
 *                        are provided (and it has to provide a `getTopic()` function).
 */
template<typename SubscriberType = ::ros::Subscriber>
class LazySubscriberBase
{
public:
  //! \brief Type of the function that connects the subscriber.
  //! \param[out] sub Reference to the subscriber object to be created.
  typedef ::std::function<void(SubscriberType& sub)> ConnectFn;
  
  //! \brief Type of the function that disconnects the subscriber.
  //! \param [in,out] Reference to the subscriber object to be disconnected.
  typedef ::std::function<void(SubscriberType& sub)> DisconnectFn;

  /**
   * \brief Create the lazy subscriber base.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object.
   * \param[in] disconnectFn The function that disconnects the subscriber. The `sub` object passed to the function is
   *                         the one created previously by `connectFn`. The passed subscriber should be invalidated.
   * \param[in] logHelper Logging helper.
   * \note The base class itself does not regularly check the connect/disconnect condition. Subclasses are responsible
   *       for calling `updateSubscription()` when it is possible that the subscription condition changed.
   */
  explicit LazySubscriberBase(const ConnectFn& connectFn,
    const DisconnectFn& disconnectFn = [](SubscriberType& sub) { sub.shutdown(); },
    ::cras::LogHelperPtr logHelper = ::std::make_shared<::cras::NodeLogHelper>());
  
  /**
   * \brief Destroy this object and unsubscribe the subscriber if it was subscribed.
   */
  virtual ~LazySubscriberBase();
  
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
   * \brief The callback called when a new subscriber appears or disappears.
   * \note This function locks `connectMutex`.
   */
  void updateSubscription();
  
  /**
   * \brief The callback called when a new subscriber appears or disappears.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  void updateSubscriptionNoLock();
  
  /**
   * \brief Connect the subscriber to its input.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  virtual void connectNoLock();

  /**
   * \brief Disconnect the subscriber from its input.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  virtual void disconnectNoLock();
  
  //! \brief The underlying subscriber (valid only when `subscribed` is true).
  SubscriberType sub;
  
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
  
  //! \brief Logging helper.
  ::cras::LogHelperPtr logHelper;
};

/**
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers.
 * \tparam M Type of the publisher messages.
 * \tparam SubscriberType Type of the subscriber created when this object should subscribe. Normally, this would be
 *                        ros::Subscriber, but actually anything can be here if suitable connectFn and disconnectFn
 *                        are provided (and it has to provide a `getTopic()` function).
 */
template<typename M, typename SubscriberType = ::ros::Subscriber>
class LazySubscriber : public ::cras::LazySubscriberBase<SubscriberType>
{
public:

  /**
   * \brief Create the lazy subscriber that subscribes only when `pub` has subscribers.
   * \param[in] pub The publisher whose number of subscribers decides whether to connect or not.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object.
   * \param[in] disconnectFn The function that disconnects the subscriber. The `sub` object passed to the function is
   *                         the one created previously by `connectFn`. The passed subscriber should be invalidated.
   * \param[in] logHelper Logging helper.
   */
  LazySubscriber(const ::ros::Publisher& pub,
    const typename ::cras::LazySubscriberBase<SubscriberType>::ConnectFn& connectFn,
    const typename ::cras::LazySubscriberBase<SubscriberType>::DisconnectFn& disconnectFn =
      [](SubscriberType& sub) { sub.shutdown(); },
    ::cras::LogHelperPtr logHelper = ::std::make_shared<::cras::NodeLogHelper>());
  
protected:
  /**
   * \brief The callback called when a new subscriber appears or disappears.
   */
  void connectCb(const ::ros::SingleSubscriberPublisher&);

  bool shouldBeSubscribed() const override;

  //! \brief The publisher whose number of subscribers decides whether to connect or not.
  ::ros::Publisher pub;	
};

}

#include "impl/lazy_subscriber.hpp"