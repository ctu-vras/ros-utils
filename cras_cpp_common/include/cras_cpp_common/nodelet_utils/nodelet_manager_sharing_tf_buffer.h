#pragma once

/**
 * \file
 * \brief A nodelet manager that can share its TF buffer with cras::NodeletWithSharedTfBuffer nodelets.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cras_cpp_common/nodelet_utils/nodelet_manager.h>
#include <cras_cpp_common/nodelet_utils/nodelet_with_shared_tf_buffer.hpp>
#include <cras_cpp_common/resettable.h>

namespace cras
{

/**
 * \brief A nodelet manager that can share its TF buffer with cras::NodeletWithSharedTfBuffer nodelets.
 */
class NodeletManagerSharingTfBuffer : public ::cras::NodeletManager, public ::cras::TimeJumpResettable
{
public:
  /**
   * \brief Create the nodelet manager.
   * \param[in] nh The nodehandle used for nodelet load/unload ROS services.
   */
  explicit NodeletManagerSharingTfBuffer(::ros::NodeHandle nh = {"~"});

  void init() override;

  void reset() override;

protected:
  ::boost::shared_ptr<::nodelet::Nodelet> createInstance(const ::std::string& lookupName) override;

  //! \brief The shared TF buffer.
  ::std::shared_ptr<::tf2_ros::Buffer> buffer;

  //! \brief TF listener filling the shared buffer.
  ::std::unique_ptr<::tf2_ros::TransformListener> listener;
};

}
