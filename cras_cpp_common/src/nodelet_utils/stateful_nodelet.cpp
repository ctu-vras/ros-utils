/**
 * \file
 * \brief A nodelet mixin that can report that it is being unloaded.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// HACK: We need to access some private fields. See below for explanation. This should hopefully go away in the future.
#include <sstream>
#define private public
#define protected public
#include <nodelet/nodelet.h>
#include <nodelet/detail/callback_queue.h>
#include <nodelet/detail/callback_queue_manager.h>
#undef protected
#undef private

#include <ros/node_handle.h>

#include <cras_cpp_common/nodelet_utils/impl/stateful_nodelet.hpp>

// HACK: Until https://github.com/ros/nodelet_core/pull/116 is merged, there is no better way to determine whether
// the nodelet has started unloading.
/**
 * \brief Return whether the given callback queue is a part of its parent callback queue manager (only valid for
 *        nodelets run using `Loader`).
 * \param[in] queue The queue to query.
 * \return Whether the queue is valid.
 * \note When a nodelet is requested to be unloaded, its callback queues are removed from the parent callback queue
 *       manager, but they remain valid until all callbacks finish.
 */
bool isCallbackQueueValid(const ros::CallbackQueueInterface* const queue)
{
  const auto nodeletQueue = dynamic_cast<const nodelet::detail::CallbackQueue*>(queue);
  // if not a nodelet callback queue, we don't know what to do, so we rather report the queue as valid
  if (nodeletQueue == nullptr)
    return true;
  const auto& queues = nodeletQueue->parent_->queues_;
  return queues.find(const_cast<nodelet::detail::CallbackQueue*>(nodeletQueue)) != queues.cend();
}

namespace cras
{

bool isNodeletUnloading(const nodelet::Nodelet& nodelet)
{
  return nodelet.inited_ && (
    !isCallbackQueueValid(nodelet.getNodeHandle().getCallbackQueue()) ||
    !isCallbackQueueValid(nodelet.getMTNodeHandle().getCallbackQueue()));
}

}
