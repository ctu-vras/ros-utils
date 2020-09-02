#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <camera_throttle/rgbd_camera_subscriber.h>
#include <camera_throttle/rgbd_camera_publisher.h>

namespace camera_throttle
{

/**
 * Throttle (or just relay) a RGB and depth camera topic pair (image_raw + camera_info) and possibly pointcloud and publish the output via image_transport.
 *
 * Parameters:
 *  ~rate: If set, the output topic will be rate-limited to this rate (probably a little lower).
 *  ~queue_size: Queue size for both subscription and publication. Default is 10.
 *  ~sub_rgb_base_name: Base name of the RGB input image. Default is image_raw.
 *  ~pub_rgb_base_name: Base name of the RGB output image. Default is whatever is set to ~sub_rgb_base_name.
 *  ~sub_depth_base_name: Base name of the depth input image. Default is depth.
 *  ~pub_depth_base_name: Base name of the depth output image. Default is whatever is set to ~sub_depth_base_name.
 *  ~subscribe_pcl: If true, also the pointcloud will be subscribed and throttled. Default is true.
 *
 * Topics:
 *  camera_in: The input camera topics
 *    camera_in/camera_info
 *    camera_in/<~sub_rgb_base_name>
 *    camera_in/<~sub_depth_base_name>
 *  points_in: Incoming pointclouds
 *  camera_out: The output camera topics
 *    camera_out/camera_info
 *    camera_out/<~sub_rgb_base_name>
 *    camera_out/<~sub_rgb_base_name>/... - the classic set of topics created by image_transport publisher
 *    camera_out/<~sub_depth_base_name>
 *    camera_out/<~sub_depth_base_name>/... - the classic set of topics created by image_transport publisher
 *  points_out: Outgoing pointclouds
 */
class RgbdCameraThrottleNodelet : public cras::Nodelet
{
  public: RgbdCameraThrottleNodelet() = default;
  public: ~RgbdCameraThrottleNodelet() override {};

  protected: void onInit() override;

  protected: virtual void cb(const sensor_msgs::ImageConstPtr& rgbImg, const sensor_msgs::ImageConstPtr& depthImg, const sensor_msgs::CameraInfoConstPtr& info);
  protected: virtual void cbPcl(const sensor_msgs::ImageConstPtr& rgbImg, const sensor_msgs::ImageConstPtr& depthImg, const sensor_msgs::CameraInfoConstPtr& info, const sensor_msgs::PointCloud2ConstPtr& pcl);

  protected: virtual void onFirstConnect();
  protected: virtual void onLastDisconnect();

  protected: ros::NodeHandle subNh;
  protected: ros::NodeHandle pubNh;
  protected: std::unique_ptr<RgbdImageTransport> subTransport;
  protected: std::unique_ptr<RgbdImageTransport> pubTransport;
  protected: std::optional<RgbdCameraSubscriber> sub;
  protected: RgbdCameraPublisher pub;
  protected: std::optional<ros::Rate> rate;
  protected: size_t queueSize {10};
  protected: ros::Time lastUpdate;
  protected: std::string subRGBBaseName;
  protected: std::string pubRGBBaseName;
  protected: std::string subDepthBaseName;
  protected: std::string pubDepthBaseName;
  protected: bool subscribePcl;

  protected: std::mutex publishersMutex;

  private: void img_connect_cb(const image_transport::SingleSubscriberPublisher&);
  private: void info_connect_cb(const ros::SingleSubscriberPublisher&);
  private: void img_disconnect_cb(const image_transport::SingleSubscriberPublisher&);
  private: void info_disconnect_cb(const ros::SingleSubscriberPublisher&);
};

}