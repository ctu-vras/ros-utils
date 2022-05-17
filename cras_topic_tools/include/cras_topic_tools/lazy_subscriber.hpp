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
#include <utility>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <ros/advertise_options.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscriber.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>

namespace cras
{

/**
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers.
 * \tparam M Type of the publisher messages.
 * \tparam SubscriberType Type of the subscriber created when this object should subscribe. Normally, this would be
 *                        ros::Subscriber, but actually anything can be here if suitable connectFn and disconnectFn
 *                        are provided (and it has to provide a `getTopic()` function).
 */
template<typename M, typename SubscriberType = ::ros::Subscriber>
class LazySubscriber
{
public:
  //! \brief Type of the function that connects the subscriber.
  typedef ::std::function<void(SubscriberType&)> ConnectFn;
  
  //! \brief Type of the function that disconnects the subscriber.
  typedef ::std::function<void(SubscriberType&)> DisconnectFn;

  /**
   * \brief Create the lazy subscriber that subscribes only when `pub` has subscribers.
   * \param[in] pub The publisher whose number of subscribers decides whether to connect or not.
   * \param[in] connectFn The function that connects the subscriber. It should store the subscriber in the passed
   *                      `sub` object.
   * \param[in] disconnectFn The function that disconnects the subscriber. The `sub` object passed to the function is
   *                         the one created previously by `connectFn`. The passed subscriber should be invalidated.
   * \param[in] logHelper Logging helper.
   */
  LazySubscriber(const ::ros::Publisher& pub, const ConnectFn& connectFn,
    const DisconnectFn& disconnectFn = [](SubscriberType& sub) { sub.shutdown(); },
    ::cras::LogHelperPtr logHelper = ::std::make_shared<::cras::NodeLogHelper>()) :
      connectFn(connectFn), disconnectFn(disconnectFn), logHelper(::std::move(logHelper))
  {
    ::ros::AdvertiseOptions opts;
    auto cb = ::boost::bind(&LazySubscriber<M>::connectCb, this, ::boost::placeholders::_1);
    opts.template init<M>(pub.getTopic(), 10, cb, cb);

    const auto ns = ::ros::names::parentNamespace(pub.getTopic());
    ::ros::NodeHandle nh(ns);

    // Need to create the publisher with connection mutex - connectCB can be called before the publisher is created
    // in nodelet, which means no topics will connect.
    ::std::lock_guard<::std::mutex> lock(this->connectMutex);
    this->pub = nh.advertise(opts);
  }
  
  /**
   * \brief Destroy this object and unsubscribe the subscriber if it was subscribed.
   */
  virtual ~LazySubscriber()
  {
    ::std::lock_guard<::std::mutex> lock(this->connectMutex);
    if (this->subscribed)
      this->disconnectNoLock();
  }
  
  /**
   * \brief Whether the subscriber is currently subscribed to its topic or not.
   * \return Whether the subscriber is currently subscribed to its topic or not.
   */
  bool isSubscribed() const
  {
    ::std::lock_guard<::std::mutex> lock(this->connectMutex);
    return this->subscribed;
  }
  
protected:
  /**
   * \brief The callback called when a new subscriber appears or disappears.
   */
  void connectCb(const ::ros::SingleSubscriberPublisher&)
  {
    ::std::lock_guard<::std::mutex> lock(this->connectMutex);
    if (this->subscribed && this->pub.getNumSubscribers() == 0)
    {
      this->disconnectNoLock();
    }
    else if (!this->subscribed && this->pub.getNumSubscribers() > 0)
    {
      this->connectNoLock();
    }
  }
  
  /**
   * \brief Connect the subscriber to its input.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  virtual void connectNoLock()
  {
    this->connectFn(this->sub);
    this->subscribed = true;
    this->logHelper->logDebug("Connected to topic " + this->sub.getTopic());
  }

  /**
   * \brief Disconnect the subscriber from its input.
   * \note You have to lock this->connectMutex prior to calling this method.
   */
  virtual void disconnectNoLock()
  {
    this->logHelper->logDebug("Disconnecting from topic " + this->sub.getTopic());
    this->disconnectFn(this->sub);
    this->subscribed = false;
  }
  
  //! \brief The publisher whose number of subscribers decides whether to connect or not.
  ::ros::Publisher pub;	
  
  //! \brief The underlying subscriber (valid only when `subscribed` is true).
  SubscriberType sub;

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

}