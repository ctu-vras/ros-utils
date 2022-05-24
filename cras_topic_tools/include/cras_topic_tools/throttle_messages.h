#pragma once

/**
 * \file
 * \brief This is a simple implementation of a throttle nodelet. It can process the messages on a single topic in
 *        parallel allowing for maximum throughput. It also allows using the more precise token bucket rate-limiting
 *        algorithm.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */
 
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/rate_limiter.h>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>

namespace cras
{

/**
 * \brief (Possibly lazy) publisher and subscriber pair that throttles published messages to the given rate. If the
 *        input rate is lower than the desired, the messages are published on the input rate.
 * \tparam SubscriberType Type of the lazy-created subscriber. 
 */
template <typename SubscriberType = ::ros::Subscriber>
class ThrottleMessagesPubSub : public ::cras::GenericLazyPubSub<SubscriberType>
{
public:
  /**
   * \brief Create the lazy pub-sub object. 
   * \param[in] limiter The rate limiter to use on output.
   * \param[in] topicIn Input topic.
   * \param[in] topicOut Output topic
   * \param[in] nh Nodehandle used for subscriber and publisher creation.
   * \param[in] inQueueSize Queue size for the subscriber.
   * \param[in] outQueueSize Queue size for the publisher.
   * \param[in] logHelper Log helper.
   */
  ThrottleMessagesPubSub(::std::unique_ptr<::cras::RateLimiter> limiter,
    const ::std::string& topicIn, const ::std::string& topicOut, const ::ros::NodeHandle& nh = {},
    size_t inQueueSize = 10, size_t outQueueSize = 10,
    const ::cras::LogHelperPtr& logHelper = ::std::make_shared<::cras::NodeLogHelper>()) :
      GenericLazyPubSub<SubscriberType>(topicIn, topicOut, nh, inQueueSize, outQueueSize, logHelper),
        limiter(::std::move(limiter))
  {
  }

  ~ThrottleMessagesPubSub() override = default;
  
  /**
   * \brief Reset the rate limiter, e.g. after a time jump.
   */
  void reset()
  {
    ::std::lock_guard<::std::mutex> lock(this->limiterMutex);
    this->limiter->reset();
  }

protected:
  /**
   * \brief Publish the incoming message if the rate limiter allows.
   * \param[in] event The incoming message event.
   * \note It is safe to call this function from multiple threads (access to the rate limiter is mutex-protected).
   */
  void processMessage(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>& event) override
  {
    bool shouldPublish;
    {
      ::std::lock_guard<::std::mutex> lock(this->limiterMutex);
      shouldPublish = this->limiter->shouldPublish(::ros::Time::now());
    }
    
    if (shouldPublish)
      this->pub.template publish(event.getConstMessage());
  }
  
  //! \brief The rate limiter used for limiting output rate.
  ::std::unique_ptr<::cras::RateLimiter> limiter;
  
  //! \brief Mutex for working with the limiter.
  ::std::mutex limiterMutex;
};

/**
 * \brief Nodelet for throttling messages on a topic.
 * 
 * ROS parameters:
 * - `~in_queue_size` (uint, default 10): Queue size for the subscriber. 
 * - `~out_queue_size` (uint, default $in_queue_size): Queue size for the publisher. 
 * - `~frequency` (Hz, positive double, default 1.0): The maximum rate of the published messages.
 * - `~lazy` (bool, default False): Whether to shut down the subscriber when the publisher has no subscribers.
 *                                  The `~input` topic will be subscribed in the beginning, and will unsubscribe
 *                                  automatically after the first message is received (this is needed to determine the
 *                                  full type of the topic to publish).
 * - `~method` (enum, default TOKEN_BUCKET): The rate-limiting method.
 *   - `TOKEN_BUCKET`: more precise algorithm.
 *     - `~bucket_capacity` (uint, default 2): Maximum burst size in case the incoming messages stop for a while and
 *                                             then start coming again.
 *     - `~initial_tokens` (uint, default 1): How many messages can be published right after the nodelet is started or
 *                                            reset. If you set it to 0, the first message can be published after
 *                                            1/frequency seconds.
 *   - `THROTTLE`: the algorithm from topic_tools/throttle. It is not very precise.
 * 
 * Subscribed topics:
 * - `~input` (any type): The input messages. If `lazy` is true, it will only be subscribed when `~output` has some
 *                        subscribers.
 * - `~reset` (any type): When a message is received on this topic, the rate limiter is reset into its initial state
 *                        (i.e. token bucket will set the number of tokens to `initial_tokens`). This should happen
 *                        every time the ROS time jumps, e.g. when simulation is reset or a bag file starts playing
 *                        again. The rate limiter should, however, reset itself when it notices a large jump backward in
 *                        ROS time. Forward jumps are not autocorrected and require publishing a message on this topic.
 * 
 * Published topics:
 * - `~output` (same type as `~input`): The throttled output messages.
 * 
 * Command-line arguments:
 * This nodelet (or node) can also be called in a way backwards compatible with topic_tools/throttle. This means you can
 * pass CLI arguments specifying the topics to subscribe/publish and the rate.
 * - `rosrun cras_topic_tools throttle messages TOPIC_IN RATE [TOPIC_OUT]`
 *   - `messages` is just a static word and is there for compatibility with topic_tools/relay. It has to be there if you
 *     configure this node(let) via CLI.
 *   - `TOPIC_IN`: The topic to subscribe. It is resolved against parent namespace of the node(let) (as opposed to the
 *                 `~input` topic which is resolved against the private node(let) namespace).
 *   - `RATE`: The output rate. If both this argument and `~frequency` parameter are specified, the ROS parameter wins.
 *   - `TOPIC_OUT`: The topic to publish. It is resolved against parent namespace of the node(let) (as opposed to the
 *                  `~output` topic which is resolved against the private node(let) namespace). If not specified, output
 *                  topic will be `${TOPIC_IN}_throttle`.
 */
class ThrottleMessagesNodelet : public ::cras::Nodelet
{
protected:
  //! \brief The lazy pair of subscriber and publisher.
  ::std::unique_ptr<::cras::ThrottleMessagesPubSub<>> pubSub;
  
  //! \brief Subscriber to the reset topic.
  ::ros::Subscriber resetSub;

  void onInit() override;
  
  /**
   * \brief Called when the rate limiter should be reset. The incoming message can be of any type and should not be
   *        examined.
   */
  virtual void onReset(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&);

};

}