/**
 * \file
 * \brief This is a simple implementation of a relay nodelet. It can process the messages on a single topic in parallel
 *        allowing for maximum throughput.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

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
  
  this->pubSub = std::make_unique<cras::GenericLazyPubSub<>>(
    inTopic, outTopic, nh, inQueueSize, outQueueSize, this->log);

  if (!lazy)
    this->pubSub->setLazy(false);
  
  this->log->logInfo("Created%s relay from %s to %s.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str());
}

}

PLUGINLIB_EXPORT_CLASS(cras::RelayNodelet, nodelet::Nodelet) // NOLINT(cert-err58-cpp)