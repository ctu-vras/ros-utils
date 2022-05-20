#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>

#include <cras_topic_tools/relay.h>

namespace cras
{

void RelayNodelet::cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
  const auto& msg = event.getConstMessage();

  if (!this->advertised)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->advertised)  // the first check is outside mutex, this one is inside
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
  }
  
  this->pub.publish(msg);
}

void RelayNodelet::onInit()
{
  auto pnh = this->getMTPrivateNodeHandle();

  const auto inQueueSize = pnh.param("in_queue_size", 10);
  this->outQueueSize = pnh.param("out_queue_size", inQueueSize);

  ros::SubscribeOptions ops;
  ops.template initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
    "input", inQueueSize, boost::bind(&RelayNodelet::cb, this, _1));
  // allow concurrent processing even for messages on a single subscriber
  ops.allow_concurrent_callbacks = true;
  this->sub = pnh.subscribe(ops);
}

}

PLUGINLIB_EXPORT_CLASS(cras::RelayNodelet, nodelet::Nodelet) // NOLINT(cert-err58-cpp)