/**
 * \file
 * \brief TF buffer that can be correctly interrupted by nodelet unload.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>

#include <nodelet/nodelet.h>
#include <ros/duration.h>
#include <tf2/buffer_core.h>

#include <cras_cpp_common/nodelet_utils/nodelet_aware_tf_buffer.h>
#include <cras_cpp_common/nodelet_utils/stateful_nodelet.hpp>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>

namespace cras
{

NodeletAwareTFBuffer::NodeletAwareTFBuffer(const nodelet::Nodelet& nodelet, const ros::Duration& cacheTime) :
  nodelet(nodelet), cras::InterruptibleTFBuffer(cacheTime)
{
}

NodeletAwareTFBuffer::NodeletAwareTFBuffer(const nodelet::Nodelet& nodelet,
  const std::shared_ptr<tf2::BufferCore>& parentBuffer) : nodelet(nodelet), cras::InterruptibleTFBuffer(parentBuffer)
{
}

NodeletAwareTFBuffer::~NodeletAwareTFBuffer() = default;

bool NodeletAwareTFBuffer::ok() const
{
  if (!InterruptibleTFBuffer::ok())
    return false;
  // Support the more advanvced interface of StatefulNodelet (which allows explicit requestStop() calls)
  auto stateful = dynamic_cast<const cras::StatefulNodeletInterface*>(&this->nodelet);
  if (stateful != nullptr)
    return stateful->ok();
  else
    return !cras::isNodeletUnloading(this->nodelet);
}

}