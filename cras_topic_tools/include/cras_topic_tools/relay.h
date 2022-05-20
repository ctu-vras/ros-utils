#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>

//! This is the simplest possible implementation of a non-lazy relay nodelet
//! It can process the messages on a single topic in parallel allowing for
//! maximum throughput.

namespace cras
{

class RelayNodelet : public nodelet::Nodelet
{
  ros::Subscriber sub;
  ros::Publisher pub;
  size_t outQueueSize;
  bool advertised {false};
  std::mutex mutex;
  
  void cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& event);
  
  void onInit() override;
};

}
