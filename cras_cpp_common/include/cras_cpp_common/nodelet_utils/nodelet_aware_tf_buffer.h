/**
 * \file
 * \brief TF buffer that can be correctly interrupted by nodelet unload.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <memory>

#include <nodelet/nodelet.h>
#include <ros/duration.h>
#include <tf2/buffer_core.h>

#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>

namespace cras
{

/**
 * Provides overrides of canTransform() that correctly end when the nodelet is asked to unload.
 * 
 * See issue https://github.com/ros/geometry2/issues/381 for more details.
 */
class NodeletAwareTFBuffer : public ::cras::InterruptibleTFBuffer
{
public:
  /**
   * \brief Create the buffer.
   * \param[in] nodelet The stateful nodelet to be aware of.
   * \param[in] cacheTime How long to keep a history of transforms
   */
  explicit NodeletAwareTFBuffer(const ::nodelet::Nodelet& nodelet,
    const ::ros::Duration& cacheTime=::ros::Duration(::tf2::BufferCore::DEFAULT_CACHE_TIME));
  
  /**
   * \brief Create the buffer that relays lookups to the given parentBuffer.
   * \param[in] nodelet The stateful nodelet to be aware of.
   * \param[in] parentBuffer The buffer to relay lookups to. It may be null.
   */
  NodeletAwareTFBuffer(const ::nodelet::Nodelet& nodelet,
    const ::std::shared_ptr<::tf2::BufferCore>& parentBuffer);

  ~NodeletAwareTFBuffer() override;

  bool ok() const override;

protected:
  //! \brief The stateful nodelet to be aware of.
  const ::nodelet::Nodelet& nodelet;
};

}