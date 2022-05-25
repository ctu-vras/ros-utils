/**
 * \file
 * \brief Count messages on a topic.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <mutex>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/subscribe_options.h>
#include <topic_tools/shape_shifter.h>

#include <cras_topic_tools/count_messages.h>

namespace cras
{

void CountMessagesNodelet::cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->count++;
  this->bytes += event.getConstMessage()->size();
  this->getMTPrivateNodeHandle().setParam("count", static_cast<int>(this->count));
  this->getMTPrivateNodeHandle().setParam("bytes", static_cast<int>(this->bytes));
}

void CountMessagesNodelet::onInit()
{
  auto pnh = this->getMTPrivateNodeHandle();
  const auto inQueueSize = pnh.param("in_queue_size", 1000);

  this->getMTPrivateNodeHandle().setParam("bytes", 0);
  this->getMTPrivateNodeHandle().setParam("count", 0);
  
  // we cannot use the simple one-liner pnh.subscribe() - otherwise there's double free from
  // https://github.com/ros/ros_comm/pull/1722
  ros::SubscribeOptions ops;
  ops.template initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
    "input", inQueueSize, boost::bind(&CountMessagesNodelet::cb, this, boost::placeholders::_1));
  this->sub = pnh.subscribe(ops);
  
  ops.template initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
    "reset", inQueueSize, boost::bind(&CountMessagesNodelet::resetCb, this, boost::placeholders::_1));
  this->resetSub = pnh.subscribe(ops);
}

void CountMessagesNodelet::resetCb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>&)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->count = this->bytes = 0;
  this->getMTPrivateNodeHandle().setParam("count", 0);
  this->getMTPrivateNodeHandle().setParam("bytes", 0);
}

}

PLUGINLIB_EXPORT_CLASS(cras::CountMessagesNodelet, nodelet::Nodelet)