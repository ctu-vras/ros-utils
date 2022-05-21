#pragma once

/**
 * \file
 * \brief Lazy subscriber that subscribes only when a paired publisher has subscribers. Version for unknown message
 *        type of both the subscriber and the publisher.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>

#include <ros/advertise_options.h>
#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscriber.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>

#include <cras_topic_tools/lazy_subscriber.hpp>

namespace cras
{

/**
 * \brief A pair of lazy subscriber and publisher which use the same message type (unknown at compile time).
 * \note If publisher type is known at compile time, use LazySubscriber.
 * \note Even though the subscriber is lazy, it has to subscribe to the input topic and receive one message from which
 *       it can derive the publisher parameters. After receiving this first message, the subscriber is disconnected in
 *       case the publisher has no subscribers.
 * \tparam SubscriberType Type of the subscriber created when this object should subscribe. Normally, this would be
 *                        ros::Subscriber, but actually anything can be here if suitable connectFn and disconnectFn
 *                        are provided (and it has to provide a `getTopic()` function).
 */
template<typename SubscriberType = ::ros::Subscriber>
class GenericLazyPubSub : public ::cras::LazySubscriberBase<SubscriberType>
{
public:
  /**
   * \brief Create the pair of lazy subscriber and publisher.
   * \param[in] topicIn Input topic (relative to `nh`).
   * \param[in] topicOut Output topic (relative to `nh`).
   * \param[in] nh Node handle for message publication and subscription.
   * \param[in] inQueueSize Queue size of the input subscriber.
   * \param[in] outQueueSize Queue size of the output publisher.
   * \param[in] logHelper Log helper.
   */
  GenericLazyPubSub(const ::std::string& topicIn, const ::std::string& topicOut, const ::ros::NodeHandle& nh = {},
    size_t inQueueSize = 10, size_t outQueueSize = 10,
    ::cras::LogHelperPtr logHelper = ::std::make_shared<::cras::NodeLogHelper>());
  
protected:
  /**
   * \brief Perform the subscription to the input topic.
   * \param[out] sub The subscriber reference to fill with the created subscriber.
   */
  void subscribe(SubscriberType& sub);
  
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

  /**
   * \brief Process a message received by the subscriber. This method can be freely reimplemented in downstream classes.
   * \param[in] event The message event.
   */
  virtual void processMessage(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);
  
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
  SubscriberType sub;
  
  //! \brief Node handle for topic suscription.
  ::ros::NodeHandle nh;
  
  //! \brief Mutex protecting `pub`. 
  ::std::mutex pubCreateMutex;
};

}

#include "impl/generic_lazy_pubsub.hpp"