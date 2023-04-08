// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a simple implementation of a relay nodelet. It can process the messages on a single topic in parallel
 *        allowing for maximum throughput.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/publisher.h>
#include <ros/subscribe_options.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/functional.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/relay.h>

namespace cras
{

void RelayNodelet::onInit()
{
  const auto params = this->privateParams();
  const auto inQueueSize = params->getParam("in_queue_size", 10);
  const auto outQueueSize = params->getParam("out_queue_size", inQueueSize);
  const auto lazy = params->getParam("lazy", false);
  const auto tcpNoDelay = params->getParam("tcp_no_delay", false);

  auto nh = this->getMTPrivateNodeHandle();
  std::string inTopic = "input";
  std::string outTopic = "output";

  // Mimic the behavior of topic_tools/relay when called with CLI args
  if (!this->getMyArgv().empty())
  {
    nh = this->getMTNodeHandle();
    inTopic = this->getMyArgv()[0];
    outTopic = (this->getMyArgv().size() >= 2 ? this->getMyArgv()[1] : (inTopic + "_relay"));
  }

  ros::SubscribeOptions opts;
  opts.allow_concurrent_callbacks = true;
  opts.transport_hints.tcpNoDelay(tcpNoDelay);
  this->pubSub = std::make_unique<cras::GenericLazyPubSub>(nh, inTopic, outTopic, inQueueSize, outQueueSize,
    cras::bind_front(&RelayNodelet::processMessage, this), opts, this->log);

  if (!lazy)
    this->pubSub->setLazy(false);

  CRAS_INFO("Created%s relay from %s to %s.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str());
}

void RelayNodelet::processMessage(
  const ros::MessageEvent<const topic_tools::ShapeShifter>& event, ros::Publisher& pub)
{
  pub.publish(event.getConstMessage());
}

}

PLUGINLIB_EXPORT_CLASS(cras::RelayNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
