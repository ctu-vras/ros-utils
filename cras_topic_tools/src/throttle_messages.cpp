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
#include <stdexcept>
#include <string>
#include <utility>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/rate.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/rate_limiter.h>
#include <cras_cpp_common/string_utils.hpp>

#include <cras_topic_tools/throttle_messages.h>

namespace cras
{

void ThrottleMessagesNodelet::onInit()
{
  auto nh = this->getMTPrivateNodeHandle();
  std::string inTopic = "input";
  std::string outTopic = "output";
  ros::Rate defaultRate(1.0);

  // Mimic the behavior of topic_tools/throttle when called with CLI args
  if (!this->getMyArgv().empty())
  {
    if (this->getMyArgv()[0] != "messages")
      throw std::runtime_error("First CLI argument of throttle node has to be 'messages'.");

    if (this->getMyArgv().size() < 3)
      throw std::runtime_error("Not enough arguments.\nUsage: throttle messages IN_TOPIC RATE [OUT_TOPIC].");

    nh = this->getMTNodeHandle();
    inTopic = this->getMyArgv()[1];
    outTopic = (this->getMyArgv().size() >= 4 ? this->getMyArgv()[3] : (inTopic + "_throttle"));
    try
    {
      defaultRate = ros::Rate(cras::parseDouble(this->getMyArgv()[2]));
    }
    catch (const std::invalid_argument& e)
    {
      CRAS_WARN("Could not parse the given throttling rate: %s", e.what());
    }
  }

  auto params = this->privateParams();
  const auto inQueueSize = params->getParam("in_queue_size", 10_sz, "messages");
  const auto outQueueSize = params->getParam("out_queue_size", inQueueSize, "messages");
  const auto rate = params->getParam("frequency", defaultRate, "Hz");
  const auto lazy = params->getParam("lazy", false);

  const auto method = params->getParam("method", "TOKEN_BUCKET");  // TOKEN_BUCKET / THROTTLE
  std::unique_ptr<cras::RateLimiter> limiter;

  if (method == "THROTTLE")
  {
    limiter = std::make_unique<cras::ThrottleLimiter>(rate);
  }
  else  // TOKEN_BUCKET and other
  {
    if (method != "TOKEN_BUCKET")
      CRAS_WARN("Unknown rate-limitation method %s. Using TOKEN_BUCKET instead.", method.c_str());
    const auto bucketCapacity = params->getParam("bucket_capacity", 2_sz, "tokens");
    const auto initialTokens = params->getParam("initial_tokens", 1_sz, "tokens");
    limiter = std::make_unique<cras::TokenBucketLimiter>(rate, bucketCapacity, initialTokens);
  }

  this->pubSub = std::make_unique<cras::ThrottleMessagesPubSub<>>(
    std::move(limiter), inTopic, outTopic, nh, inQueueSize, outQueueSize, this->log);

  if (!lazy)
    this->pubSub->setLazy(false);

  ros::SubscribeOptions opts;
  auto cb = boost::bind(&ThrottleMessagesNodelet::onReset, this, boost::placeholders::_1);
  opts.initByFullCallbackType<const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&>("reset", 1, cb);
  this->resetSub = this->getMTPrivateNodeHandle().subscribe(opts);

  CRAS_INFO("Created%s throttle from %s to %s at rate %s Hz.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str(),
    cras::to_string(rate).c_str());
}

void ThrottleMessagesNodelet::onReset(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&)
{
  this->pubSub->reset();
}

}

PLUGINLIB_EXPORT_CLASS(cras::ThrottleMessagesNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
