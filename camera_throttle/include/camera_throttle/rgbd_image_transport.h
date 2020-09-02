#pragma once

#include <image_transport/image_transport.h>
#include <camera_throttle/rgbd_camera_publisher.h>
#include <camera_throttle/rgbd_camera_subscriber.h>

namespace camera_throttle
{

class RgbdImageTransport : public image_transport::ImageTransport
{
  public: RgbdImageTransport(const ros::NodeHandle& rgbNh, const ros::NodeHandle& depthNh);
  public: RgbdImageTransport(const ros::NodeHandle& rgbNh, const ros::NodeHandle& depthNh, const ros::NodeHandle& pclNh);
  public: virtual ~RgbdImageTransport() = default;

  /*!
   * \brief Advertise a synchronized RGBD camera raw image + info topic pair, simple version.
   */
  public: RgbdCameraPublisher advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                             size_t queue_size, bool latch = false);

  /*!
   * \brief Advertise a synchronized RGBD camera raw image + info topic pair with subscriber status
   * callbacks.
   */
  public: RgbdCameraPublisher advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                  size_t queue_size,
                                  const image_transport::SubscriberStatusCallback& rgb_connect_cb,
                                  const image_transport::SubscriberStatusCallback& rgb_disconnect_cb = image_transport::SubscriberStatusCallback(),
                                  const image_transport::SubscriberStatusCallback& depth_connect_cb = image_transport::SubscriberStatusCallback(),
                                  const image_transport::SubscriberStatusCallback& depth_disconnect_cb = image_transport::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& rgb_info_connect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& rgb_info_disconnect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& depth_info_connect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& depth_info_disconnect_cb = ros::SubscriberStatusCallback(),
                                  const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /*!
   * \brief Advertise a synchronized RGBD camera raw image + info topic pair and pointcloud, simple version.
   */
  public: RgbdCameraPublisher advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                              const std::string& pcl_topic, size_t queue_size, bool latch = false);

  /*!
   * \brief Advertise a synchronized RGBD camera raw image + info topic pair and pointcloud with subscriber status
   * callbacks.
   */
  public: RgbdCameraPublisher advertiseRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                  const std::string& pcl_topic, size_t queue_size,
                                  const image_transport::SubscriberStatusCallback& rgb_connect_cb,
                                  const image_transport::SubscriberStatusCallback& rgb_disconnect_cb = image_transport::SubscriberStatusCallback(),
                                  const image_transport::SubscriberStatusCallback& depth_connect_cb = image_transport::SubscriberStatusCallback(),
                                  const image_transport::SubscriberStatusCallback& depth_disconnect_cb = image_transport::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& pcl_connect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& pcl_disconnect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& rgb_info_connect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& rgb_info_disconnect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& depth_info_connect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& depth_info_disconnect_cb = ros::SubscriberStatusCallback(),
                                  const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary
   * boost::function object.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
                                   const RgbdCameraSubscriber::Callback& callback,
                                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints());

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   */
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
                                   void(*fp)(const sensor_msgs::ImageConstPtr&,
                                             const sensor_msgs::CameraInfoConstPtr&,
                                             const sensor_msgs::ImageConstPtr&,
                                             const sensor_msgs::CameraInfoConstPtr&),
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints())
  {
    return subscribeRgbdCamera(rgb_base_topic, depth_base_topic, queue_size, RgbdCameraSubscriber::Callback(fp), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with bare pointer.
   */
  template<class T>
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&,
                                                const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&), T* obj,
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints())
  {
    return subscribeRgbdCamera(rgb_base_topic, depth_base_topic, queue_size, boost::bind(fp, obj, _1, _2, _3, _4), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with shared_ptr.
   */
  template<class T>
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&,
                                                const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&),
                                   const boost::shared_ptr<T>& obj,
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints())
  {
    return subscribeRgbdCamera(rgb_base_topic, depth_base_topic, queue_size, boost::bind(fp, obj.get(), _1, _2, _3, _4), obj,
                           transport_hints);
  }



  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary
   * boost::function object.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                   const std::string& pcl_topic, size_t queue_size,
                                   const RgbdCameraSubscriber::PclCallback& callback,
                                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints());

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   */
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                   const std::string& pcl_topic, size_t queue_size,
                                   void(*fp)(const sensor_msgs::ImageConstPtr&,
                                             const sensor_msgs::CameraInfoConstPtr&,
                                             const sensor_msgs::ImageConstPtr&,
                                             const sensor_msgs::CameraInfoConstPtr&,
                                             const sensor_msgs::PointCloud2ConstPtr&),
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints())
  {
    return subscribeRgbdCamera(rgb_base_topic, depth_base_topic, pcl_topic, queue_size, RgbdCameraSubscriber::PclCallback(fp), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with bare pointer.
   */
  template<class T>
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                   const std::string& pcl_topic, size_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&,
                                                const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&,
                                                const sensor_msgs::PointCloud2ConstPtr&), T* obj,
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints())
  {
    return subscribeRgbdCamera(rgb_base_topic, depth_base_topic, pcl_topic, queue_size, boost::bind(fp, obj, _1, _2, _3, _4, _5), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with shared_ptr.
   */
  template<class T>
  RgbdCameraSubscriber subscribeRgbdCamera(const std::string& rgb_base_topic, const std::string& depth_base_topic,
                                   const std::string& pcl_topic, size_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&,
                                                const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&,
                                                const sensor_msgs::PointCloud2ConstPtr&),
                                   const boost::shared_ptr<T>& obj,
                                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints())
  {
    return subscribeRgbdCamera(rgb_base_topic, depth_base_topic, pcl_topic, queue_size, boost::bind(fp, obj.get(), _1, _2, _3, _4, _5), obj,
                           transport_hints);
  }

  protected: ros::NodeHandle rgbNh;
  protected: ros::NodeHandle depthNh;
  protected: ros::NodeHandle pclNh;
};

}