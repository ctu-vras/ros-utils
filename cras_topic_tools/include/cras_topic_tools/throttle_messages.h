#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>

//! This is the simplest possible implementation of a non-lazy throttle nodelet

namespace cras
{

class ThrottleMessagesNodelet : public nodelet::Nodelet
{
  ros::Subscriber sub;
  ros::Publisher pub;
  size_t outQueueSize;
  bool advertised {false};
  ros::Time lastPubTime;
  ros::Duration minPubPeriod;

  void cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& event);

  void onInit() override;
};

}