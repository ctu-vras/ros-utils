// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a relay nodelet that can modify headers.
 * \note It can process the messages on a single topic in parallel allowing for maximum throughput.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <utility>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/duration.h>
#include <ros/subscribe_options.h>
#include <ros/time.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/change_header.h>
#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

void ChangeHeaderNodelet::onInit()
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
    outTopic = (this->getMyArgv().size() >= 2 ? this->getMyArgv()[1] : (inTopic + "_change_header"));
  }

  if (params->hasParam("frame_id_prefix"))
    this->newFrameIdPrefix = params->getParam<std::string>("frame_id_prefix", cras::nullopt);

  if (params->hasParam("frame_id_suffix"))
    this->newFrameIdSuffix = params->getParam<std::string>("frame_id_suffix", cras::nullopt);

  if (params->hasParam("frame_id"))
    this->newFrameId = params->getParam<std::string>("frame_id", cras::nullopt);

  if (params->hasParam("frame_id_replace_start"))
  {
    const auto param = params->getParam<std::string>("frame_id_replace_start", cras::nullopt);
    const auto parts = cras::split(param, "|", 1);
    const auto& from = parts[0];
    const auto& to = parts.size() > 1 ? parts[1] : "";
    this->newFrameIdReplaceStart = std::make_pair(from, to);
  }

  if (params->hasParam("frame_id_replace_end"))
  {
    const auto param = params->getParam<std::string>("frame_id_replace_end", cras::nullopt);
    const auto parts = cras::split(param, "|", 1);
    const auto& from = parts[0];
    const auto& to = parts.size() > 1 ? parts[1] : "";
    this->newFrameIdReplaceEnd = std::make_pair(from, to);
  }

  if (params->hasParam("frame_id_replace"))
  {
    const auto param = params->getParam<std::string>("frame_id_replace", cras::nullopt);
    const auto parts = cras::split(param, "|", 1);
    const auto& from = parts[0];
    const auto& to = parts.size() > 1 ? parts[1] : "";
    this->newFrameIdReplace = std::make_pair(from, to);
  }

  if (params->hasParam("stamp_relative"))
    this->newStampRel = params->getParam<ros::Duration>("stamp_relative", cras::nullopt);

  if (params->hasParam("stamp"))
    this->newStampAbs = params->getParam<ros::Time>("stamp", cras::nullopt);

  this->newStampRosTime = params->getParam("stamp_ros_time", false);
  this->newStampWallTime = params->getParam("stamp_wall_time", false);

  ros::SubscribeOptions opts;
  opts.allow_concurrent_callbacks = true;
  opts.transport_hints.tcpNoDelay(tcpNoDelay);
  this->pubSub = std::make_unique<cras::GenericLazyPubSub>(nh, inTopic, outTopic, inQueueSize, outQueueSize,
    cras::bind_front(&ChangeHeaderNodelet::processMessage, this), opts, this->log);

  if (!lazy)
    this->pubSub->setLazy(false);

  CRAS_INFO("Created%s header changer subscribing to %s and publishing to %s.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str());
}

void ChangeHeaderNodelet::processMessage(
  const ros::MessageEvent<const topic_tools::ShapeShifter>& event, ros::Publisher& pub)
{
  const auto& msg = event.getConstMessage();

  if (!this->hasHeader.has_value())
  {
    this->hasHeader = cras::hasHeader(*msg);

    if (!this->hasHeader.value())
    {
      CRAS_ERROR("Running change_header on message type %s which does not have a header! Ignoring all messages.",
                 event.getConnectionHeader()["type"].c_str());
    }
  }

  if (!this->hasHeader.value())
    return;

  auto header = cras::getHeader(*msg);
  if (!header.has_value())
  {
    CRAS_ERROR("Change_header failed to extract a header from the message of type %s! "
               "Ignoring the message.", event.getConnectionHeader()["type"].c_str());
    return;
  }
  const auto origHeader = header.value();

  if (this->newFrameIdReplaceStart.has_value())
  {
    const auto fromTo = this->newFrameIdReplaceStart.value();
    cras::replace(header->frame_id, fromTo.first, fromTo.second, cras::ReplacePosition::START);
  }

  if (this->newFrameIdReplaceEnd.has_value())
  {
    const auto fromTo = this->newFrameIdReplaceEnd.value();
    cras::replace(header->frame_id, fromTo.first, fromTo.second, cras::ReplacePosition::END);
  }

  if (this->newFrameIdReplace.has_value())
  {
    const auto fromTo = this->newFrameIdReplace.value();
    cras::replace(header->frame_id, fromTo.first, fromTo.second);
  }

  if (this->newFrameIdPrefix.has_value())
    header->frame_id = this->newFrameIdPrefix.value() + header->frame_id;

  if (this->newFrameIdSuffix.has_value())
    header->frame_id += this->newFrameIdSuffix.value();

  if (this->newFrameId.has_value())
    header->frame_id = this->newFrameId.value();

  if (this->newStampRosTime)
  {
    header->stamp = ros::Time::now();
  }
  else if (this->newStampWallTime)
  {
    const auto now = ros::WallTime::now();
    header->stamp = {now.sec, now.nsec};
  }

  if (this->newStampRel.has_value())
  {
    // Correctly handle overflow cases
    try
    {
      header->stamp += this->newStampRel.value();
    }
    catch (const ::std::runtime_error&)
    {
      header->stamp = this->newStampRel.value().toSec() > 0 ? ros::TIME_MAX : ros::TIME_MIN;
    }
  }

  if (this->newStampAbs.has_value())
    header->stamp = this->newStampAbs.value();

  if (header.value() == origHeader)
  {
    pub.publish(msg);
    return;
  }

  topic_tools::ShapeShifter newMsg;
  // cannot use newMsg = *msg in Melodic, that would segfault
  cras::copyShapeShifter(*msg, newMsg);
  if (!cras::setHeader(newMsg, header.value()))
  {
    CRAS_ERROR("Change_header failed to modify the header of the message of type %s! "
               "Ignoring the message.", event.getConnectionHeader()["type"].c_str());
    return;
  }

  pub.publish(newMsg);
}

}

PLUGINLIB_EXPORT_CLASS(cras::ChangeHeaderNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
