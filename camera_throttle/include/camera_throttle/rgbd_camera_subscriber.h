#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/transport_hints.h>

namespace camera_throttle {

class RgbdImageTransport;

/**
 * \brief Manages a subscription callback on synchronized Image and CameraInfo topics for RGB and depth camera, and possibly also the pointcloud from depth.
 *
 * RgbdCameraSubscriber is the client-side counterpart to CameraPublisher, and assumes the
 * same topic naming convention. The callback is of type:
\verbatim
void callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
\endverbatim
 * or
\verbatim
void callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&, const sensor_msgs::PointCloud2ConstPtr&);
\endverbatim
 * A RgbdCameraSubscriber should always be created through a call to
 * RgbdImageTransport::subscribeCamera(), or copied from one that was.
 * Once all copies of a specific RgbdCameraSubscriber go out of scope, the subscription callback
 * associated with that handle will stop being called. Once all RgbdCameraSubscriber for a given
 * topic go out of scope the topic will be unsubscribed.
 */
class RgbdCameraSubscriber
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&,
                               const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&)> Callback;
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&,
                               const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&,
                               const sensor_msgs::PointCloud2ConstPtr&)> PclCallback;

  RgbdCameraSubscriber() {}

  /**
   * \brief Get the RGB base topic (on which the raw image is published).
   */
  std::string getRGBTopic() const;

  /**
   * \brief Get the RGB camera info topic.
   */
  std::string getRGBInfoTopic() const;

  /**
   * \brief Get the depth base topic (on which the raw image is published).
   */
  std::string getDepthTopic() const;

  /**
   * \brief Get the depth camera info topic.
   */
  std::string getDepthInfoTopic() const;

  /**
   * \brief Get the topic of the pointcloud (can be empty if pointcloud is not processed).
   */
  std::string getPclTopic() const;

  /**
   * \brief Returns the number of publishers this subscriber is connected to.
   */
  size_t getNumPublishers() const;

  /**
   * \brief Returns the name of the transport being used for RGB.
   */
  std::string getRGBTransport() const;

  /**
   * \brief Returns the name of the transport being used for depth.
   */
  std::string getDepthTransport() const;

  /**
   * \brief Unsubscribe the callback associated with this RgbdCameraSubscriber.
   */
  void shutdown();

  explicit operator void*() const;
  bool operator< (const RgbdCameraSubscriber& rhs) const { return impl <  rhs.impl; }
  bool operator!=(const RgbdCameraSubscriber& rhs) const { return impl != rhs.impl; }
  bool operator==(const RgbdCameraSubscriber& rhs) const { return impl == rhs.impl; }

private:
  RgbdCameraSubscriber(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh,
                   const std::string& rgb_base_topic, const std::string& depth_base_topic,
                   size_t queue_size, const Callback& callback,
                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints());
  RgbdCameraSubscriber(RgbdImageTransport& image_it, ros::NodeHandle& rgb_nh, ros::NodeHandle& depth_nh, ros::NodeHandle& pcl_nh,
                   const std::string& rgb_base_topic, const std::string& depth_base_topic,
                   const std::string& pcl_topic, size_t queue_size, const PclCallback& callback,
                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                   const image_transport::TransportHints& transport_hints = image_transport::TransportHints());

  struct Impl;
  std::shared_ptr<Impl> impl;

  friend class RgbdImageTransport;
};

}