/**
 * \file
 * \brief This is a relay nodelet that can modify headers. It can process the messages on a single topic in parallel
 *        allowing for maximum throughput.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>
#include <utility>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <cras_cpp_common/optional.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>
#include <cras_topic_tools/change_header.h>

namespace cras
{

void ChangeHeaderNodelet::onInit()
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
    outTopic = (this->getMyArgv().size() >= 2 ? this->getMyArgv()[1] : (inTopic + "_change_header"));
  }

  cras::ChangeHeaderParams changeHeaderParams;

  if (params->hasParam("frame_id_prefix"))
    changeHeaderParams.newFrameIdPrefix = params->getParam<std::string>("frame_id_prefix", cras::nullopt);

  if (params->hasParam("frame_id_suffix"))
    changeHeaderParams.newFrameIdSuffix = params->getParam<std::string>("frame_id_suffix", cras::nullopt);

  if (params->hasParam("frame_id"))
    changeHeaderParams.newFrameId = params->getParam<std::string>("frame_id", cras::nullopt);

  if (params->hasParam("frame_id_replace_start"))
  {
    const auto param = params->getParam<std::string>("frame_id_replace_start", cras::nullopt);
    const auto parts = cras::split(param, "|", 1);
    const auto& from = parts[0];
    const auto& to = parts.size() > 1 ? parts[1] : "";
    changeHeaderParams.newFrameIdReplaceStart = std::make_pair(from, to);
  }

  if (params->hasParam("frame_id_replace_end"))
  {
    const auto param = params->getParam<std::string>("frame_id_replace_end", cras::nullopt);
    const auto parts = cras::split(param, "|", 1);
    const auto& from = parts[0];
    const auto& to = parts.size() > 1 ? parts[1] : "";
    changeHeaderParams.newFrameIdReplaceEnd = std::make_pair(from, to);
  }

  if (params->hasParam("frame_id_replace"))
  {
    const auto param = params->getParam<std::string>("frame_id_replace", cras::nullopt);
    const auto parts = cras::split(param, "|", 1);
    const auto& from = parts[0];
    const auto& to = parts.size() > 1 ? parts[1] : "";
    changeHeaderParams.newFrameIdReplace = std::make_pair(from, to);
  }

  if (params->hasParam("stamp_relative"))
    changeHeaderParams.newStampRel = params->getParam<ros::Duration>("stamp_relative", cras::nullopt);

  if (params->hasParam("stamp"))
    changeHeaderParams.newStampAbs = params->getParam<ros::Time>("stamp", cras::nullopt);

  this->pubSub = std::make_unique<cras::ChangeHeaderPubSub<>>(
    changeHeaderParams, inTopic, outTopic, nh, inQueueSize, outQueueSize, this->log);

  if (!lazy)
    this->pubSub->setLazy(false);

  this->log->logInfo("Created%s header changer subscribing to %s and publishing to %s.",
    (lazy ? " lazy" : ""), nh.resolveName(inTopic).c_str(), nh.resolveName(outTopic).c_str());
}

}

PLUGINLIB_EXPORT_CLASS(cras::ChangeHeaderNodelet, nodelet::Nodelet)  // NOLINT(cert-err58-cpp)
