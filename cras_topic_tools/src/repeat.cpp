/**
 * \file
 * \brief This is a simple implementation of a repeater nodelet.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <stdexcept>
#include <string>

#include <boost/bind.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/rate.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/string_utils.hpp>

#include <cras_topic_tools/repeat.h>

namespace cras
{

void RepeatMessagesNodelet::onInit()
{
  auto nh = this->getMTPrivateNodeHandle();
  std::string inTopic = "input";
  std::string outTopic = "output";
  ros::Rate defaultRate(1.0);

  // Allow basic configuration via command line args.
  if (!this->getMyArgv().empty())
  {
    if (this->getMyArgv().size() == 1)
      throw std::runtime_error("Not enough arguments.\nUsage: repeat IN_TOPIC RATE [OUT_TOPIC].");

    nh = this->getMTNodeHandle();
    inTopic = this->getMyArgv()[0];
    outTopic = (this->getMyArgv().size() >= 3 ? this->getMyArgv()[2] : (inTopic + "_repeat"));
    try
    {
      defaultRate = ros::Rate(cras::parseDouble(this->getMyArgv()[1]));
    }
    catch (const std::invalid_argument& e)
    {
      this->log->logWarn("Could not parse the given repeat rate: %s", e.what());
    }
  }

  auto params = this->privateParams();
  const auto inQueueSize = params->getParam("in_queue_size", 10_sz, "messages");
  const auto outQueueSize = params->getParam("out_queue_size", inQueueSize, "messages");
  const auto rate = params->getParam("rate", defaultRate, "Hz");
  const auto lazy = params->getParam("lazy", false);

  cras::RepeatMessagesParams repeatParams {rate};
  if (params->hasParam("max_age"))
    repeatParams.maxAge = params->getParam("max_age", ros::Duration(1), "s");
  if (params->hasParam("max_repeats"))
    repeatParams.maxRepeats = params->getParam("max_repeats", 10_sz);
  repeatParams.discardOlderMessages = params->getParam("discard_older_messages", false);
  repeatParams.resetOnMsg = params->getParam("reset_on_msg", true);
  repeatParams.publishOnlyOnTimer = params->getParam("publish_only_on_timer", false);

  this->pubSub = std::make_unique<cras::RepeatMessagesPubSub<>>(
    repeatParams, inTopic, outTopic, nh, inQueueSize, outQueueSize, this->log);

  if (!lazy)
    this->pubSub->setLazy(false);

  ros::SubscribeOptions opts;
  auto cb = boost::bind(&RepeatMessagesNodelet::onReset, this, boost::placeholders::_1);
  opts.initByFullCallbackType<const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&>("reset", 1, cb);
  this->resetSub = this->getMTPrivateNodeHandle().subscribe(opts);

  this->log->logInfo("Created%s repeater from %s to %s at rate %s Hz.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str(),
    cras::to_string(rate).c_str());
}

void RepeatMessagesNodelet::onReset(const ::ros::MessageEvent<const ::topic_tools::ShapeShifter>&)
{
  this->pubSub->reset();
}

}

PLUGINLIB_EXPORT_CLASS(cras::RepeatMessagesNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
