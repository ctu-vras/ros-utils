#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>
#include <cras_topic_tools/throttle_messages.h>

//! This is the simplest possible implementation of a non-lazy throttle nodelet

namespace cras
{

void ThrottleMessagesNodelet::cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
  const auto& now = ros::Time::now();
  if (now < this->lastPubTime)
  {
    ROS_WARN("Detected jump back in time, resetting throttle period to now.");
    this->lastPubTime = now;
  }
  
  if ((now - this->lastPubTime) < this->minPubPeriod)
    return;
  
  this->lastPubTime = now;

  const auto& msg = event.getConstMessage();
  
  if (!this->advertised)
  {
    const auto& connectionHeader = event.getConnectionHeaderPtr();
    bool latch = false;
    if (connectionHeader)
    {
      auto it = connectionHeader->find("latching");
      if((it != connectionHeader->end()) && (it->second == "1"))
      {
        ROS_DEBUG("input topic is latched; latching output topic to match");
        latch = true;
      }
    }
    this->pub = msg->advertise(this->getMTPrivateNodeHandle(), "output", this->outQueueSize, latch);
    this->advertised = true;
    ROS_INFO("advertised as %s\n", this->getMTPrivateNodeHandle().resolveName("output").c_str());
  }

  this->pub.publish(msg);
}

void ThrottleMessagesNodelet::onInit()
{
  auto pnh = this->getMTPrivateNodeHandle();

  const auto inQueueSize = pnh.param("in_queue_size", 10);
  this->outQueueSize = pnh.param("out_queue_size", inQueueSize);
  
  this->minPubPeriod = ros::Rate(pnh.param("frequency", 1.0)).expectedCycleTime();

  // we cannot use the simple one-liner pnh.subscribe() - otherwise there's double free from https://github.com/ros/ros_comm/pull/1722
  ros::SubscribeOptions ops;
  ops.template initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
    "input", inQueueSize, boost::bind(&ThrottleMessagesNodelet::cb, this, _1));
  this->sub = pnh.subscribe(ops);
}

}

PLUGINLIB_EXPORT_CLASS(cras::ThrottleMessagesNodelet, nodelet::Nodelet) // NOLINT(cert-err58-cpp)