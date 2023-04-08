// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a nodelet that republishes heartbeat of a topic with header.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/names.h>
#include <ros/subscribe_options.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_msgs/Heartbeat.h>

#include <cras_topic_tools/heartbeat.h>
#include <cras_topic_tools/lazy_subscriber.hpp>
#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

void HeartbeatNodelet::onInit()
{
  const auto params = this->privateParams();
  const auto queueSize = params->getParam("queue_size", 10);
  const auto lazy = params->getParam("lazy", false);
  const auto tcpNoDelay = params->getParam("tcp_no_delay", false);

  auto nh = this->getMTPrivateNodeHandle();
  std::string topic = "input";

  // Mimic the behavior of topic_tools nodes when called with CLI args
  if (!this->getMyArgv().empty())
  {
    nh = this->getMTNodeHandle();
    topic = this->getMyArgv()[0];
  }

  const auto heartbeatTopic = ros::names::append(nh.resolveName(topic), "heartbeat");

  this->pub = nh.advertise<cras_msgs::Heartbeat>(heartbeatTopic, queueSize);

  ros::SubscribeOptions opts;
  opts.allow_concurrent_callbacks = true;
  opts.transport_hints.tcpNoDelay(tcpNoDelay);
  this->sub = std::make_unique<SubscriberType>(nh, heartbeatTopic, nh, topic, queueSize,
      cras::bind_front(&HeartbeatNodelet::processMessage, this), opts, this->log);

  if (!lazy)
    this->sub->setLazy(false);

  CRAS_INFO("Created%s heartbeat subscribing to %s and publishing to %s.",
            (lazy ? " lazy" : ""), nh.resolveName(topic).c_str(), nh.resolveName(heartbeatTopic).c_str());
}

void HeartbeatNodelet::processMessage(const ros::MessageEvent<const topic_tools::ShapeShifter>& event)
{
  const auto& msg = event.getConstMessage();

  if (!this->hasHeader.has_value())
  {
    this->hasHeader = cras::hasHeader(*msg);

    if (!this->hasHeader.value())
    {
      CRAS_ERROR("Heartbeat did not find a header in message type %s! Ignoring all messages.",
                 event.getConnectionHeader()["type"].c_str());
    }
  }

  if (!this->hasHeader.value())
    return;

  auto header = cras::getHeader(*msg);
  if (!header.has_value())
  {
    CRAS_ERROR("Heartbeat failed to extract a header from the message of type %s! Ignoring the message.",
               event.getConnectionHeader()["type"].c_str());
    return;
  }

  cras_msgs::Heartbeat heartbeat;
  heartbeat.header = *header;
  this->pub.template publish(heartbeat);
}

}

PLUGINLIB_EXPORT_CLASS(cras::HeartbeatNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
