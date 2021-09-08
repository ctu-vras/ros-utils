#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>

namespace cras
{

class RelayNodelet : public nodelet::Nodelet
{
  ros::Subscriber sub;
  ros::Publisher pub;
  size_t out_queue_size;
  bool advertised {false};
  std::mutex mutex;
  
  void cb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
  {
    const auto& msg = msg_event.getConstMessage();

    if (!this->advertised)
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (!this->advertised)  // the first check is outside mutex, this one is inside
      {
        const auto& connection_header = msg_event.getConnectionHeaderPtr();
        bool latch = false;
        if (connection_header)
        {
          auto it = connection_header->find("latching");
          if((it != connection_header->end()) && (it->second == "1"))
          {
            ROS_DEBUG("input topic is latched; latching output topic to match");
            latch = true;
          }
        }
        this->pub = msg->advertise(this->getMTPrivateNodeHandle(), "output", this->out_queue_size, latch);
        this->advertised = true;
        ROS_INFO("advertised as %s\n", this->getMTPrivateNodeHandle().resolveName("output").c_str());
      }
    }
    
    this->pub.publish(msg);
  }
  
  void onInit() override
  {
    auto pnh = this->getMTPrivateNodeHandle();
    this->out_queue_size = pnh.param("out_queue_size", 10);
    this->sub = pnh.subscribe("input", pnh.param("in_queue_size", 10), &RelayNodelet::cb, this);
  }
};

}

PLUGINLIB_EXPORT_CLASS(cras::RelayNodelet, nodelet::Nodelet) // NOLINT(cert-err58-cpp)