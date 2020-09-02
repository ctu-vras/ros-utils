#include <camera_throttle/rgbd_camera_publisher.h>
#include <camera_throttle/rgbd_image_transport.h>
#include <image_transport/camera_common.h>

namespace camera_throttle
{

struct RgbdCameraPublisher::Impl
{
  Impl() : unadvertised(false), hasPcl(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !this->unadvertised;
  }

  void shutdown()
  {
    if (!this->unadvertised) {
      this->unadvertised = true;
      this->rgbPub.shutdown();
      this->rgbInfoPub.shutdown();
      this->depthPub.shutdown();
      this->depthInfoPub.shutdown();
      if (this->hasPcl)
        this->pclPub.shutdown();
    }
  }

  image_transport::Publisher rgbPub;
  ros::Publisher rgbInfoPub;
  image_transport::Publisher depthPub;
  ros::Publisher depthInfoPub;
  ros::Publisher pclPub;
  bool unadvertised;
  bool hasPcl;
};

RgbdCameraPublisher::RgbdCameraPublisher(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh,
  const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
  const image_transport::SubscriberStatusCallback& rgb_connect_cb,
  const image_transport::SubscriberStatusCallback& rgb_disconnect_cb,
  const image_transport::SubscriberStatusCallback& depth_connect_cb,
  const image_transport::SubscriberStatusCallback& depth_disconnect_cb,
  const ros::SubscriberStatusCallback& rgb_info_connect_cb,
  const ros::SubscriberStatusCallback& rgb_info_disconnect_cb,
  const ros::SubscriberStatusCallback& depth_info_connect_cb,
  const ros::SubscriberStatusCallback& depth_info_disconnect_cb,
  const ros::VoidPtr& tracked_object, bool latch)
    : impl(new Impl)
{
  this->impl->hasPcl = false;

  // Explicitly resolve name here so we compute the correct CameraInfo topic when the
  // image topic is remapped
  const auto rgb_image_topic = rgb_nh.resolveName(rgb_base_topic);
  const auto rgb_info_topic = image_transport::getCameraInfoTopic(rgb_image_topic);
  const auto depth_image_topic = depth_nh.resolveName(depth_base_topic);
  const auto depth_info_topic = image_transport::getCameraInfoTopic(depth_image_topic);

  this->impl->rgbPub = image_it.advertise(
      rgb_image_topic, queue_size, rgb_connect_cb, rgb_disconnect_cb, tracked_object, latch);
  this->impl->rgbInfoPub = rgb_nh.advertise<sensor_msgs::CameraInfo>(
      rgb_info_topic, queue_size, rgb_info_connect_cb, rgb_info_disconnect_cb, tracked_object, latch);
  this->impl->depthPub = image_it.advertise(
      depth_image_topic, queue_size, depth_connect_cb, depth_disconnect_cb, tracked_object, latch);
  this->impl->depthInfoPub = depth_nh.advertise<sensor_msgs::CameraInfo>(
      depth_info_topic, queue_size, depth_info_connect_cb, depth_info_disconnect_cb, tracked_object, latch);
}

RgbdCameraPublisher::RgbdCameraPublisher(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh, ros::NodeHandle& pcl_nh,
  const std::string& rgb_base_topic, const std::string& depth_base_topic, const std::string& pcl_topic,
  size_t queue_size,
  const image_transport::SubscriberStatusCallback& rgb_connect_cb,
  const image_transport::SubscriberStatusCallback& rgb_disconnect_cb,
  const image_transport::SubscriberStatusCallback& depth_connect_cb,
  const image_transport::SubscriberStatusCallback& depth_disconnect_cb,
  const ros::SubscriberStatusCallback& pcl_connect_cb,
  const ros::SubscriberStatusCallback& pcl_disconnect_cb,
  const ros::SubscriberStatusCallback& rgb_info_connect_cb,
  const ros::SubscriberStatusCallback& rgb_info_disconnect_cb,
  const ros::SubscriberStatusCallback& depth_info_connect_cb,
  const ros::SubscriberStatusCallback& depth_info_disconnect_cb,
  const ros::VoidPtr& tracked_object, bool latch)
    : impl(new Impl)
{
  this->impl->hasPcl = true;

  // Explicitly resolve name here so we compute the correct CameraInfo topic when the
  // image topic is remapped
  const auto rgb_image_topic = rgb_nh.resolveName(rgb_base_topic);
  const auto rgb_info_topic = image_transport::getCameraInfoTopic(rgb_image_topic);
  const auto depth_image_topic = depth_nh.resolveName(depth_base_topic);
  const auto depth_info_topic = image_transport::getCameraInfoTopic(depth_image_topic);

  this->impl->rgbPub = image_it.advertise(
      rgb_image_topic, queue_size, rgb_connect_cb, rgb_disconnect_cb, tracked_object, latch);
  this->impl->rgbInfoPub = rgb_nh.advertise<sensor_msgs::CameraInfo>(
      rgb_info_topic, queue_size, rgb_info_connect_cb, rgb_info_disconnect_cb, tracked_object, latch);
  this->impl->depthPub = image_it.advertise(
      depth_image_topic, queue_size, depth_connect_cb, depth_disconnect_cb, tracked_object, latch);
  this->impl->depthInfoPub = depth_nh.advertise<sensor_msgs::CameraInfo>(
      depth_info_topic, queue_size, depth_info_connect_cb, depth_info_disconnect_cb, tracked_object, latch);
  this->impl->pclPub = pcl_nh.advertise<sensor_msgs::PointCloud2>(
      pcl_topic, queue_size, pcl_connect_cb, pcl_disconnect_cb, tracked_object, latch);
}

size_t RgbdCameraPublisher::getNumSubscribers() const
{
  if (impl && impl->isValid())
    return std::max(std::max(
      std::max(impl->rgbPub.getNumSubscribers(), impl->rgbInfoPub.getNumSubscribers()),
      std::max(impl->depthPub.getNumSubscribers(), impl->depthInfoPub.getNumSubscribers())
    ), (this->impl->hasPcl ? impl->pclPub.getNumSubscribers() : 0));
  return 0;
}

std::string RgbdCameraPublisher::getRGBTopic() const
{
  if (impl) return impl->rgbPub.getTopic();
  return {};
}

std::string RgbdCameraPublisher::getRGBInfoTopic() const
{
  if (impl) return impl->rgbInfoPub.getTopic();
  return {};
}

std::string RgbdCameraPublisher::getDepthTopic() const
{
  if (impl) return impl->depthPub.getTopic();
  return {};
}

std::string RgbdCameraPublisher::getDepthInfoTopic() const
{
  if (impl) return impl->depthInfoPub.getTopic();
  return {};
}

std::string RgbdCameraPublisher::getPclTopic() const
{
  if (impl && impl->hasPcl) return impl->pclPub.getTopic();
  return {};
}

void RgbdCameraPublisher::publish(const sensor_msgs::Image& rgb_image, const sensor_msgs::CameraInfo& rgb_info,
                                  const sensor_msgs::Image& depth_image, const sensor_msgs::CameraInfo& depth_info) const
{
  if (!impl || !impl->isValid() || impl->hasPcl) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::RgbdCameraPublisher");
    return;
  }

  impl->rgbPub.publish(rgb_image);
  impl->rgbInfoPub.publish(rgb_info);

  impl->depthPub.publish(depth_image);
  impl->depthInfoPub.publish(depth_info);
}

void RgbdCameraPublisher::publish(const sensor_msgs::Image& rgb_image, const sensor_msgs::CameraInfo& rgb_info,
                                  const sensor_msgs::Image& depth_image, const sensor_msgs::CameraInfo& depth_info,
                                  const sensor_msgs::PointCloud2& pcl) const
{
  if (!impl || !impl->isValid() || !impl->hasPcl) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::RgbdCameraPublisher");
    return;
  }

  impl->rgbPub.publish(rgb_image);
  impl->rgbInfoPub.publish(rgb_info);

  impl->depthPub.publish(depth_image);
  impl->depthInfoPub.publish(depth_info);

  impl->pclPub.publish(pcl);
}

void RgbdCameraPublisher::publish(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info,
                                  const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info) const
{
  if (!impl || !impl->isValid() || impl->hasPcl) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::RgbdCameraPublisher");
    return;
  }

  impl->rgbPub.publish(rgb_image);
  impl->rgbInfoPub.publish(rgb_info);

  impl->depthPub.publish(depth_image);
  impl->depthInfoPub.publish(depth_info);
}

void RgbdCameraPublisher::publish(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info,
                                  const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info,
                                  const sensor_msgs::PointCloud2ConstPtr& pcl) const
{
  if (!impl || !impl->isValid() || !impl->hasPcl) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::RgbdCameraPublisher");
    return;
  }

  impl->rgbPub.publish(rgb_image);
  impl->rgbInfoPub.publish(rgb_info);

  impl->depthPub.publish(depth_image);
  impl->depthInfoPub.publish(depth_info);

  impl->pclPub.publish(pcl);
}

void RgbdCameraPublisher::publish(sensor_msgs::Image& rgb_image, sensor_msgs::CameraInfo& rgb_info,
                                  sensor_msgs::Image& depth_image, sensor_msgs::CameraInfo& depth_info, ros::Time stamp) const
{
  if (!impl || !impl->isValid() || impl->hasPcl) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::RgbdCameraPublisher");
    return;
  }

  rgb_image.header.stamp = stamp;
  rgb_info.header.stamp = stamp;
  depth_image.header.stamp = stamp;
  depth_info.header.stamp = stamp;

  this->publish(rgb_image, rgb_info, depth_image, depth_info);
}

void RgbdCameraPublisher::publish(sensor_msgs::Image& rgb_image, sensor_msgs::CameraInfo& rgb_info,
                                  sensor_msgs::Image& depth_image, sensor_msgs::CameraInfo& depth_info,
                                  sensor_msgs::PointCloud2& pcl, ros::Time stamp) const
{
  if (!impl || !impl->isValid() || !impl->hasPcl) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::RgbdCameraPublisher");
    return;
  }

  rgb_image.header.stamp = stamp;
  rgb_info.header.stamp = stamp;
  depth_image.header.stamp = stamp;
  depth_info.header.stamp = stamp;
  pcl.header.stamp = stamp;

  this->publish(rgb_image, rgb_info, depth_image, depth_info, stamp);
}

void RgbdCameraPublisher::shutdown()
{
  if (impl) {
    impl->shutdown();
    impl.reset();
  }
}

RgbdCameraPublisher::operator void*() const
{
  return (impl && impl->isValid()) ? (void*)1 : (void*)0;
}

}
