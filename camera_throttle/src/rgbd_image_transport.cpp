#include <camera_throttle/rgbd_image_transport.h>

namespace camera_throttle
{

RgbdImageTransport::RgbdImageTransport(const ros::NodeHandle& infoNh) : infoNh(infoNh), ImageTransport(infoNh)
{
}

RgbdImageTransport::RgbdImageTransport(const ros::NodeHandle& infoNh, const ros::NodeHandle& pclNh) : infoNh(infoNh), pclNh(pclNh), ImageTransport(infoNh)
{
}

RgbdCameraPublisher RgbdImageTransport::advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                                        size_t queue_size, bool latch)
{
  return advertiseRgbdCamera(rgb_base_topic, depth_base_topic, queue_size,
                         image_transport::SubscriberStatusCallback(), image_transport::SubscriberStatusCallback(),
                         image_transport::SubscriberStatusCallback(), image_transport::SubscriberStatusCallback(),
                         ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                         ros::VoidPtr(), latch);
}

RgbdCameraPublisher RgbdImageTransport::advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
  size_t queue_size,
  const image_transport::SubscriberStatusCallback& rgb_connect_cb,
  const image_transport::SubscriberStatusCallback& rgb_disconnect_cb,
  const image_transport::SubscriberStatusCallback& depth_connect_cb,
  const image_transport::SubscriberStatusCallback& depth_disconnect_cb,
  const ros::SubscriberStatusCallback& info_connect_cb,
  const ros::SubscriberStatusCallback& info_disconnect_cb,
  const ros::VoidPtr& tracked_object, bool latch)
{
  return RgbdCameraPublisher(*this, this->infoNh, rgb_base_topic, depth_base_topic, queue_size,
                             rgb_connect_cb, rgb_disconnect_cb, depth_connect_cb, depth_disconnect_cb,
                             info_connect_cb, info_disconnect_cb, tracked_object, latch);
}

RgbdCameraPublisher RgbdImageTransport::advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                                            const std::string& pcl_topic, size_t queue_size, bool latch)
{
  return advertiseRgbdCamera(rgb_base_topic, depth_base_topic, pcl_topic, queue_size,
                         image_transport::SubscriberStatusCallback(), image_transport::SubscriberStatusCallback(),
                         image_transport::SubscriberStatusCallback(), image_transport::SubscriberStatusCallback(),
                         ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                         ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                         ros::VoidPtr(), latch);
}

RgbdCameraPublisher RgbdImageTransport::advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
  const std::string& pcl_topic, size_t queue_size,
  const image_transport::SubscriberStatusCallback& rgb_connect_cb,
  const image_transport::SubscriberStatusCallback& rgb_disconnect_cb,
  const image_transport::SubscriberStatusCallback& depth_connect_cb,
  const image_transport::SubscriberStatusCallback& depth_disconnect_cb,
  const ros::SubscriberStatusCallback& pcl_connect_cb,
  const ros::SubscriberStatusCallback& pcl_disconnect_cb,
  const ros::SubscriberStatusCallback& info_connect_cb,
  const ros::SubscriberStatusCallback& info_disconnect_cb,
  const ros::VoidPtr& tracked_object, bool latch)
{
  return RgbdCameraPublisher(*this, this->infoNh, this->pclNh, rgb_base_topic, depth_base_topic, pcl_topic, queue_size,
                             rgb_connect_cb, rgb_disconnect_cb, depth_connect_cb, depth_disconnect_cb, pcl_connect_cb, pcl_disconnect_cb,
                             info_connect_cb, info_disconnect_cb, tracked_object, latch);
}

RgbdCameraSubscriber RgbdImageTransport::subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
                                                 const RgbdCameraSubscriber::Callback& callback,
                                                 const ros::VoidPtr& tracked_object,
                                                 const image_transport::TransportHints& transport_hints)
{
  return RgbdCameraSubscriber(*this, this->infoNh, rgb_base_topic, depth_base_topic, queue_size, callback, tracked_object, transport_hints);
}

RgbdCameraSubscriber RgbdImageTransport::subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                                   const std::string& pcl_topic, size_t queue_size,
                                                 const RgbdCameraSubscriber::PclCallback& callback,
                                                 const ros::VoidPtr& tracked_object,
                                                 const image_transport::TransportHints& transport_hints)
{
  return RgbdCameraSubscriber(*this, this->infoNh, this->pclNh, rgb_base_topic, depth_base_topic, pcl_topic, queue_size, callback, tracked_object, transport_hints);
}

}
