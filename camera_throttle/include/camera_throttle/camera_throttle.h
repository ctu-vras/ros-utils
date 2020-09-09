#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>

namespace camera_throttle
{

/**
 * Throttle (or just relay) a camera topic pair (image_raw + camera_info) and publish the output via image_transport.
 *
 * Parameters:
 *  ~rate: If set, the output topic will be rate-limited to this rate (probably a little lower).
 *  ~queue_size: Queue size for both subscription and publication. Default is 10.
 *  ~sub_base_name: Base name of the input image. Default is image_raw.
 *  ~pub_base_name: Base name of the output image. Default is whatever is set to ~sub_base_name.
 *  ~fix_frame_id: If set and nonempty, the images and camera infos will get this frame ID instead of the one they came with.
 *
 * Topics:
 *  camera_in: The input camera topics
 *    camera_in/camera_info
 *    camera_in/<~sub_base_name>
 *  camera_out: The output camera topics
 *    camera_out/camera_info
 *    camera_out/<~sub_base_name>
 *    camera_out/<~sub_base_name>/... - the classic set of topics created by image_transport publisher
 */
class CameraThrottleNodelet : public cras::Nodelet
{
  public: CameraThrottleNodelet() = default;
  public: virtual ~CameraThrottleNodelet() {};

  protected: void onInit() override;

  protected: virtual void cb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info);

  protected: virtual void onFirstConnect();
  protected: virtual void onLastDisconnect();

  protected: ros::NodeHandle subNh;
  protected: ros::NodeHandle pubNh;
  protected: std::unique_ptr<image_transport::ImageTransport> subTransport;
  protected: std::unique_ptr<image_transport::ImageTransport> pubTransport;
  protected: std::optional<image_transport::CameraSubscriber> sub;
  protected: image_transport::CameraPublisher pub;
  protected: std::optional<ros::Rate> rate;
  protected: std::optional<std::string> frameId;
  protected: size_t queueSize {10};
  protected: ros::Time lastUpdate;
  protected: std::string subBaseName;
  protected: std::string pubBaseName;

  protected: std::mutex publishersMutex;

  private: void img_connect_cb(const image_transport::SingleSubscriberPublisher&);
  private: void info_connect_cb(const ros::SingleSubscriberPublisher&);
  private: void img_disconnect_cb(const image_transport::SingleSubscriberPublisher&);
  private: void info_disconnect_cb(const ros::SingleSubscriberPublisher&);
};

}

