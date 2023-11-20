/**
 * \file
 * \brief A nodelet manager that can share its TF buffer with cras::NodeletWithSharedTfBufferInterface nodelets.
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

#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils/nodelet_manager.h>
#include <cras_cpp_common/nodelet_utils/nodelet_manager_sharing_tf_buffer.h>
#include <cras_cpp_common/nodelet_utils/nodelet_with_shared_tf_buffer.hpp>
#include <cras_cpp_common/resettable.h>

namespace cras
{

NodeletManagerSharingTfBuffer::NodeletManagerSharingTfBuffer(ros::NodeHandle nh) :
  cras::NodeletManager(nh), cras::TimeJumpResettable(std::make_shared<cras::NodeLogHelper>())
{
}

void NodeletManagerSharingTfBuffer::init()
{
  cras::NodeletManager::init();
  this->buffer = std::make_shared<tf2_ros::Buffer>();
  this->listener = std::make_unique<tf2_ros::TransformListener>(*this->buffer, this->nh);
  TimeJumpResettable::initRos(this->nh);
  this->startAutoCheckTimeJump();
}

void NodeletManagerSharingTfBuffer::reset()
{
  if (this->buffer == nullptr || this->listener == nullptr)
    return;
  this->listener.reset();
  this->buffer->clear();
  // recreate the TF listener so that the buffer receives static transforms again
  this->listener = std::make_unique<tf2_ros::TransformListener>(*this->buffer, this->nh);
}

boost::shared_ptr<nodelet::Nodelet> NodeletManagerSharingTfBuffer::createInstance(const std::string& lookupName)
{
  auto ptr = cras::NodeletManager::createInstance(lookupName);
  if (ptr == nullptr)
    return nullptr;

  auto tfNodelet = boost::dynamic_pointer_cast<cras::NodeletWithSharedTfBufferInterface>(ptr);
  if (tfNodelet != nullptr)
    tfNodelet->setBuffer(this->buffer);

  return ptr;
}

}
