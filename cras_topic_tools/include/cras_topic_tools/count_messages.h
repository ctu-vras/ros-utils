#pragma once

/**
 * \file
 * \brief Count messages on a topic.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/message_event.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <std_srvs/Trigger.h>
#include <topic_tools/shape_shifter.h>

namespace cras
{

class CountMessagesNodelet : public ::nodelet::Nodelet
{
  ::ros::Subscriber sub;
  ::ros::ServiceServer resetServer;
  int count;
  ::std::mutex mutex;

  void cb(const ::ros::MessageEvent<::topic_tools::ShapeShifter const>& event);
  bool resetCb(::std_srvs::Trigger::Request& req, ::std_srvs::Trigger::Response& resp);

  void onInit() override;
};

}