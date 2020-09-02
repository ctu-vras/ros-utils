#pragma once

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace camera_throttle
{

class RgbdImageTransport;

/**
 * \brief Manages advertisements for publishing RGBD camera images.
 *
 * RgbdCameraPublisher is a convenience class for publishing synchronized image,
 * camera info and pointcloud topics using the standard topic naming convention, where the info
 * topic name is "camera_info" in the same namespace as the base image topic.
 *
 * On the client side, RgbdCameraSubscriber simplifies subscribing to camera images.
 *
 * A RgbdCameraPublisher should always be created through a call to
 * RgbdImageTransport::advertiseCamera(), or copied from one that was.
 * Once all copies of a specific RgbdCameraPublisher go out of scope, any subscriber callbacks
 * associated with that handle will stop being called. Once all RgbdCameraPublisher for a
 * given base topic go out of scope the topic (and all subtopics) will be unadvertised.
 */
class RgbdCameraPublisher
{
public:
  RgbdCameraPublisher() = default;
  virtual ~RgbdCameraPublisher() = default;

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this CameraPublisher.
   *
   * Returns max(image topic subscribers, info topic subscribers).
   */
  size_t getNumSubscribers() const;

  /*!
   * \brief Returns the RGB base (image) topic of this CameraPublisher.
   */
  std::string getRGBTopic() const;

  /**
   * \brief Returns the RGB camera info topic of this CameraPublisher.
   */
  std::string getRGBInfoTopic() const;

  /*!
   * \brief Returns the depth base (image) topic of this CameraPublisher.
   */
  std::string getDepthTopic() const;

  /**
   * \brief Returns the depth camera info topic of this CameraPublisher.
   */
  std::string getDepthInfoTopic() const;

  /*!
   * \brief Get the topic of the pointcloud (can be empty if pointcloud is not processed).
   */
  std::string getPclTopic() const;

  /*!
   * \brief Publish an RGB and depth (image, info) pair on the topics associated with this RgbdCameraPublisher.
   */
  void publish(const sensor_msgs::Image& rgb_image, const sensor_msgs::CameraInfo& rgb_info,
               const sensor_msgs::Image& depth_image, const sensor_msgs::CameraInfo& depth_info) const;

  /*!
   * \brief Publish an RGB and depth (image, info) pair and PCL on the topics associated with this RgbdCameraPublisher.
   */
  void publish(const sensor_msgs::Image& rgb_image, const sensor_msgs::CameraInfo& rgb_info,
               const sensor_msgs::Image& depth_image, const sensor_msgs::CameraInfo& depth_info,
               const sensor_msgs::PointCloud2& pcl) const;

  /*!
   * \brief Publish an RGB and depth (image, info) pair on the topics associated with this RgbdCameraPublisher.
   */
  void publish(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info,
               const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info) const;

  /*!
   * \brief Publish an RGB and depth (image, info) pair and PCL on the topics associated with this RgbdCameraPublisher.
   */
  void publish(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& rgb_info,
               const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_info,
               const sensor_msgs::PointCloud2ConstPtr& pcl) const;

  /*!
   * \brief Publish an RGB and depth (image, info) pair with given timestamp on the topics associated with
   * this RgbdCameraPublisher.
   *
   * Convenience version, which sets the timestamps of both image and info to stamp before
   * publishing.
   */
  void publish(sensor_msgs::Image& rgb_image, sensor_msgs::CameraInfo& rgb_info,
               sensor_msgs::Image& depth_image, sensor_msgs::CameraInfo& depth_info, ros::Time stamp) const;

  /*!
   * \brief Publish an RGB and depth (image, info) pair and PCL with given timestamp on the topics associated with
   * this RgbdCameraPublisher.
   *
   * Convenience version, which sets the timestamps of both image and info to stamp before
   * publishing.
   */
  void publish(sensor_msgs::Image& rgb_image, sensor_msgs::CameraInfo& rgb_info,
               sensor_msgs::Image& depth_image, sensor_msgs::CameraInfo& depth_info,
               sensor_msgs::PointCloud2& pcl, ros::Time stamp) const;

  /*!
   * \brief Shutdown the advertisements associated with this Publisher.
   */
  void shutdown();

  explicit operator void*() const;
  bool operator< (const RgbdCameraPublisher& rhs) const { return impl <  rhs.impl; }
  bool operator!=(const RgbdCameraPublisher& rhs) const { return impl != rhs.impl; }
  bool operator==(const RgbdCameraPublisher& rhs) const { return impl == rhs.impl; }

private:
  RgbdCameraPublisher(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh,
    const std::string& rgb_base_topic, const std::string& depth_base_topic, size_t queue_size,
    const image_transport::SubscriberStatusCallback& rgb_connect_cb,
    const image_transport::SubscriberStatusCallback& rgb_disconnect_cb,
    const image_transport::SubscriberStatusCallback& depth_connect_cb,
    const image_transport::SubscriberStatusCallback& depth_disconnect_cb,
    const ros::SubscriberStatusCallback& rgb_info_connect_cb,
    const ros::SubscriberStatusCallback& rgb_info_disconnect_cb,
    const ros::SubscriberStatusCallback& depth_info_connect_cb,
    const ros::SubscriberStatusCallback& depth_info_disconnect_cb,
    const ros::VoidPtr& tracked_object, bool latch);

  RgbdCameraPublisher(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh, ros::NodeHandle& pcl_nh,
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
    const ros::VoidPtr& tracked_object, bool latch);

  struct Impl;
  std::shared_ptr<Impl> impl;

  friend class RgbdImageTransport;
};

}