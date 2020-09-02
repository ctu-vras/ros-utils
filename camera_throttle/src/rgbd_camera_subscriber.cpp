#include <camera_throttle/rgbd_camera_subscriber.h>
#include <camera_throttle/rgbd_image_transport.h>

#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <cras_cpp_common/string_utils.hpp>

inline void increment(int* value)
{
  ++(*value);
}

namespace camera_throttle {

struct RgbdCameraSubscriber::Impl
{
  explicit Impl(size_t queue_size)
      : sync(queue_size), syncPcl(queue_size),
        unsubscribed(false), hasPcl(false),
        rgbReceived(0), rgbInfoReceived(0), depthReceived(0), depthInfoReceived(0), pclReceived(0), allReceived(0)
  {}

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !this->unsubscribed;
  }

  void shutdown()
  {
    if (!this->unsubscribed) {
      this->unsubscribed = true;
      this->rgbSub.unsubscribe();
      this->rgbInfoSub.unsubscribe();
      this->depthSub.unsubscribe();
      this->depthInfoSub.unsubscribe();
      if (this->hasPcl)
        this->pclSub.unsubscribe();
    }
  }

  void checkImagesSynchronized()
  {
    int threshold = (this->hasPcl ? 6 : 5) * allReceived;
    if (rgbReceived > threshold || rgbInfoReceived > threshold || depthReceived > threshold || depthInfoReceived > threshold || pclReceived > threshold) {
      ROS_WARN_NAMED("sync", // Can suppress ros.image_transport.sync independent of anything else
                     "[rgbd_image_transport] Topics '%s', '%s', '%s', '%s'%s do not appear to be synchronized. "
                     "In the last 10s:\n"
                     "\tRGB messages received:      %d\n"
                     "\tRGB CameraInfo messages received: %d\n"
                     "\tDepth messages received:      %d\n"
                     "\tDepth CameraInfo messages received: %d\n"
                     "%s"
                     "\tSynchronized pairs:           %d",
                     rgbSub.getTopic().c_str(), rgbInfoSub.getTopic().c_str(),
                     depthSub.getTopic().c_str(), depthInfoSub.getTopic().c_str(),
                     (this->hasPcl ? " and '" + pclSub.getTopic() + "'" : std::string()).c_str(),
                     rgbReceived, rgbInfoReceived, depthReceived, depthInfoReceived,
                     (this->hasPcl ? (std::string("\tPointcloud messages received: ") + cras::to_string(pclReceived) + "\n") : std::string()).c_str(),
                     allReceived);
    }
    rgbReceived = rgbInfoReceived = depthReceived = depthInfoReceived = pclReceived = allReceived = 0;
  }

  image_transport::SubscriberFilter rgbSub;
  image_transport::SubscriberFilter depthSub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> rgbInfoSub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pclSub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> sync;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> syncPcl;
  bool unsubscribed;
  bool hasPcl;
  // For detecting when the topics aren't synchronized
  ros::WallTimer checkSyncedTimer;
  int rgbReceived, rgbInfoReceived, depthReceived, depthInfoReceived, pclReceived, allReceived;
};

RgbdCameraSubscriber::RgbdCameraSubscriber(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh,
                                   const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
                                   const Callback& callback, const ros::VoidPtr& tracked_object,
                                   const image_transport::TransportHints& transport_hints)
    : impl(new Impl(queue_size))
{
  this->impl->hasPcl = false;
  
  // Must explicitly remap the image topic since we then do some string manipulation on it
  // to figure out the sibling camera_info topic.
  const auto rgb_topic = rgb_nh.resolveName(rgb_base_topic);
  const auto rgb_info_topic = image_transport::getCameraInfoTopic(rgb_topic);
  const auto depth_topic = depth_nh.resolveName(depth_base_topic);
  const auto depth_info_topic = image_transport::getCameraInfoTopic(depth_topic);
  
  impl->rgbSub.subscribe(image_it, rgb_topic, queue_size, transport_hints);
  impl->rgbInfoSub.subscribe(rgb_nh, rgb_info_topic, queue_size, transport_hints.getRosHints());
  impl->depthSub.subscribe(image_it, depth_topic, queue_size, transport_hints);
  impl->depthInfoSub.subscribe(depth_nh, depth_info_topic, queue_size, transport_hints.getRosHints());
  impl->sync.connectInput(impl->rgbSub, impl->rgbInfoSub, impl->depthSub, impl->depthInfoSub);
  // need for Boost.Bind here is kind of broken
  impl->sync.registerCallback(boost::bind(callback, _1, _2, _3, _4));

  // Complain every 10s if it appears that the image and info topics are not synchronized
  impl->rgbSub.registerCallback(boost::bind(increment, &impl->rgbReceived));
  impl->rgbInfoSub.registerCallback(boost::bind(increment, &impl->rgbInfoReceived));
  impl->depthSub.registerCallback(boost::bind(increment, &impl->depthReceived));
  impl->depthInfoSub.registerCallback(boost::bind(increment, &impl->depthInfoReceived));
  impl->sync.registerCallback(boost::bind(increment, &impl->allReceived));
  impl->checkSyncedTimer = rgb_nh.createWallTimer(ros::WallDuration(10.0),
                                                    boost::bind(&Impl::checkImagesSynchronized, impl.get()));
}

RgbdCameraSubscriber::RgbdCameraSubscriber(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh, ros::NodeHandle& pcl_nh,
                                   const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                   const std::string& pcl_topic, size_t queue_size,
                                   const PclCallback& callback, const ros::VoidPtr& tracked_object,
                                   const image_transport::TransportHints& transport_hints)
    : impl(new Impl(queue_size))
{
  this->impl->hasPcl = true;

  // Must explicitly remap the image topic since we then do some string manipulation on it
  // to figure out the sibling camera_info topic.
  const auto rgb_topic = rgb_nh.resolveName(rgb_base_topic);
  const auto rgb_info_topic = image_transport::getCameraInfoTopic(rgb_topic);
  const auto depth_topic = depth_nh.resolveName(depth_base_topic);
  const auto depth_info_topic = image_transport::getCameraInfoTopic(depth_topic);

  impl->rgbSub.subscribe(image_it, rgb_topic, queue_size, transport_hints);
  impl->rgbInfoSub.subscribe(rgb_nh, rgb_info_topic, queue_size, transport_hints.getRosHints());
  impl->depthSub.subscribe(image_it, depth_topic, queue_size, transport_hints);
  impl->depthInfoSub.subscribe(depth_nh, depth_info_topic, queue_size, transport_hints.getRosHints());
  impl->pclSub.subscribe(pcl_nh, pcl_topic, queue_size, transport_hints.getRosHints());
  impl->syncPcl.connectInput(impl->rgbSub, impl->rgbInfoSub, impl->depthSub, impl->depthInfoSub, impl->pclSub);
  // need for Boost.Bind here is kind of broken
  impl->syncPcl.registerCallback(boost::bind(callback, _1, _2, _3, _4, _5));

  // Complain every 10s if it appears that the image and info topics are not synchronized
  impl->rgbSub.registerCallback(boost::bind(increment, &impl->rgbReceived));
  impl->rgbInfoSub.registerCallback(boost::bind(increment, &impl->rgbInfoReceived));
  impl->depthSub.registerCallback(boost::bind(increment, &impl->depthReceived));
  impl->depthInfoSub.registerCallback(boost::bind(increment, &impl->depthInfoReceived));
  impl->pclSub.registerCallback(boost::bind(increment, &impl->pclReceived));
  impl->syncPcl.registerCallback(boost::bind(increment, &impl->allReceived));
  impl->checkSyncedTimer = rgb_nh.createWallTimer(ros::WallDuration(10.0),
                                                    boost::bind(&Impl::checkImagesSynchronized, impl.get()));
}

std::string RgbdCameraSubscriber::getRGBTopic() const
{
  if (impl) return impl->rgbSub.getTopic();
  return {};
}

std::string RgbdCameraSubscriber::getRGBInfoTopic() const
{
  if (impl) return impl->rgbInfoSub.getTopic();
  return {};
}

std::string RgbdCameraSubscriber::getDepthTopic() const
{
  if (impl) return impl->depthSub.getTopic();
  return {};
}

std::string RgbdCameraSubscriber::getDepthInfoTopic() const
{
  if (impl) return impl->depthInfoSub.getTopic();
  return {};
}

std::string RgbdCameraSubscriber::getPclTopic() const
{
  if (impl && impl->hasPcl) return impl->pclSub.getTopic();
  return {};
}

size_t RgbdCameraSubscriber::getNumPublishers() const
{
  if (impl && impl->isValid())
    return std::max(std::max(
        std::max(impl->rgbSub.getNumPublishers(), impl->rgbInfoSub.getSubscriber().getNumPublishers()),
        std::max(impl->depthSub.getNumPublishers(), impl->depthInfoSub.getSubscriber().getNumPublishers())
    ), (this->impl->hasPcl ? impl->pclSub.getSubscriber().getNumPublishers() : 0));
  
  return 0;
}

std::string RgbdCameraSubscriber::getRGBTransport() const
{
  if (impl) return impl->rgbSub.getTransport();
  return {};
}

std::string RgbdCameraSubscriber::getDepthTransport() const
{
  if (impl) return impl->depthSub.getTransport();
  return {};
}

void RgbdCameraSubscriber::shutdown()
{
  if (impl) impl->shutdown();
}

RgbdCameraSubscriber::operator void*() const
{
  return (impl && impl->isValid()) ? (void*)1 : (void*)0;
}

}