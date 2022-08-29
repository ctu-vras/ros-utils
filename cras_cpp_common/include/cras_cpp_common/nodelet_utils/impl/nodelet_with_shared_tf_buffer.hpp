#pragma once

/**
 * \file
 * \brief NodeletWithSharedTfBuffer allows you to use a tf2_ros::Buffer provided by the nodelet manager (private
 *   implementation details, do not include this directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "../nodelet_with_shared_tf_buffer.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <nodelet/nodelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cras_cpp_common/nodelet_utils/nodelet_aware_tf_buffer.h>

namespace cras
{

namespace impl
{

/**
 * \brief Private implementation (PIMPL) class for NodeletWithSharedTfBuffer.
 */
struct NodeletWithSharedTfBufferPrivate
{
  //! \brief The TF buffer used for lookups.
  ::std::shared_ptr<::cras::NodeletAwareTFBuffer> buffer {};

  //! \brief Transform listener for standaloneBuffer.
  ::std::unique_ptr<::tf2_ros::TransformListener> listener {nullptr};

  //! \brief Whether a shared TF buffer is used.
  bool usesSharedBuffer {false};

  //! \brief To correctly delete the unique_ptrs.
  virtual ~NodeletWithSharedTfBufferPrivate() { }
};

}

template <typename NodeletType>
NodeletWithSharedTfBuffer<NodeletType>::NodeletWithSharedTfBuffer() :
  data(new ::cras::impl::NodeletWithSharedTfBufferPrivate)
{
}

template <typename NodeletType>
NodeletWithSharedTfBuffer<NodeletType>::~NodeletWithSharedTfBuffer()
{
}

template <typename NodeletType>
void NodeletWithSharedTfBuffer<NodeletType>::setBuffer(const ::std::shared_ptr<::tf2_ros::Buffer>& buffer)
{
  if (this->data->buffer != nullptr || this->data->listener != nullptr)
  {
    throw ::std::runtime_error("tf2 buffer cannot be set multiple times");
  }

  // use a nodelet-aware relaying buffer
  this->data->buffer = ::std::make_shared<::cras::NodeletAwareTFBuffer>(*this, buffer);
  this->data->usesSharedBuffer = true;
  NODELET_INFO("Initialized shared tf2 buffer");
}

template <typename NodeletType>
::cras::NodeletAwareTFBuffer& NodeletWithSharedTfBuffer<NodeletType>::getBuffer() const
{
  if (this->data->buffer == nullptr)
  {
    this->data->buffer = ::std::make_shared<::cras::NodeletAwareTFBuffer>(*this);
    this->data->listener = ::std::make_unique<::tf2_ros::TransformListener>(
      this->data->buffer->getRawBuffer(), this->getNodeHandle());
    this->data->usesSharedBuffer = false;
    NODELET_INFO("Initialized standalone tf2 buffer");
  }
  return *this->data->buffer;
}

template <typename NodeletType>
bool NodeletWithSharedTfBuffer<NodeletType>::usesSharedBuffer() const
{
  return this->data->usesSharedBuffer;
}

}
