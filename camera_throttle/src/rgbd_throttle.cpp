#include <camera_throttle/rgbd_throttle.h>
#include <camera_throttle/rgbd_image_transport.h>
#include <pluginlib/class_list_macros.hpp>

namespace camera_throttle
{

void RgbdCameraThrottleNodelet::onInit()
{
  auto pnh = this->getPrivateNodeHandle();
  if (pnh.hasParam("rate"))
    this->rate = this->getParam(pnh, "rate", ros::Rate(10.0), "Hz");
  else
    NODELET_INFO("No rate limiting");
  this->queueSize = this->getParam(pnh, "queue_size", 10u, "messages");
  this->subRGBBaseName = this->getParam(pnh, "sub_rgb_base_name", "image_raw");
  this->pubRGBBaseName = this->getParam(pnh, "pub_rgb_base_name", this->subRGBBaseName);
  this->subDepthBaseName = this->getParam(pnh, "sub_depth_base_name", "depth");
  this->pubDepthBaseName = this->getParam(pnh, "pub_depth_base_name", this->subDepthBaseName);
  this->subscribePcl = this->getParam(pnh, "subscribe_pcl", true);

  if (pnh.hasParam("fix_rgb_frame_id"))
    this->rgbFrameId = this->getParam(pnh, "fix_rgb_frame_id", "");
  if (this->rgbFrameId.has_value() && this->rgbFrameId.value().empty())
    this->rgbFrameId.reset();
  if (this->rgbFrameId)
    NODELET_INFO("Fixing RGB frame_id to %s", this->rgbFrameId->c_str());
  
  if (pnh.hasParam("fix_depth_frame_id"))
    this->depthFrameId = this->getParam(pnh, "fix_depth_frame_id", "");
  if (this->depthFrameId.has_value() && this->depthFrameId.value().empty())
    this->depthFrameId.reset();
  if (this->depthFrameId)
    NODELET_INFO("Fixing depth frame_id to %s", this->depthFrameId->c_str());

  this->subRgbNh = ros::NodeHandle(this->getNodeHandle(), "camera_rgb_in");
  this->subDepthNh = ros::NodeHandle(this->getNodeHandle(), "camera_depth_in");
  this->subPclNh = this->getNodeHandle();
  this->subTransport = std::make_unique<RgbdImageTransport>(this->subRgbNh, this->subDepthNh, this->subPclNh);

  this->pubRgbNh = ros::NodeHandle(this->getNodeHandle(), "camera_rgb_out");
  this->pubDepthNh = ros::NodeHandle(this->getNodeHandle(), "camera_depth_out");
  this->pubPclNh = this->getNodeHandle();
  this->pubTransport = std::make_unique<RgbdImageTransport>(this->pubRgbNh, this->pubDepthNh, this->pubPclNh);

  if (this->subscribePcl)
    this->pub = this->pubTransport->advertiseRgbdCamera(
        this->pubRGBBaseName, this->pubDepthBaseName, "points_out", this->queueSize,
        boost::bind(&RgbdCameraThrottleNodelet::img_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1));
  else
    this->pub = this->pubTransport->advertiseRgbdCamera(
        this->pubRGBBaseName, this->pubDepthBaseName, this->queueSize,
        boost::bind(&RgbdCameraThrottleNodelet::img_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1));
}

void RgbdCameraThrottleNodelet::cb(const sensor_msgs::ImageConstPtr& rgbImg, const sensor_msgs::CameraInfoConstPtr& rgbInfo, const sensor_msgs::ImageConstPtr& depthImg, const sensor_msgs::CameraInfoConstPtr& depthInfo)
{
  if (this->rate)
  {
    if (this->lastUpdate + this->rate.value().expectedCycleTime() > ros::Time::now())
    {
      NODELET_DEBUG("throttle last update at %f skipping", this->lastUpdate.toSec());
      return;
    }
  }

  this->lastUpdate = ros::Time::now();

  if (!this->rgbFrameId.has_value() && !this->depthFrameId.has_value())
  {
    this->pub.publish(rgbImg, rgbInfo, depthImg, depthInfo);
  } else {
    sensor_msgs::ImageConstPtr pubRgbImg = rgbImg;
    sensor_msgs::CameraInfoConstPtr pubRgbInfo = rgbInfo;
    sensor_msgs::ImageConstPtr pubDepthImg = depthImg;
    sensor_msgs::CameraInfoConstPtr pubDepthInfo = depthInfo;

    if (this->rgbFrameId)
    {
      sensor_msgs::ImagePtr newRgbImg(new sensor_msgs::Image);
      sensor_msgs::CameraInfoPtr newRgbInfo(new sensor_msgs::CameraInfo);
      *newRgbImg = *rgbImg;
      *newRgbInfo = *rgbInfo;
      newRgbImg->header.frame_id = this->rgbFrameId.value();
      newRgbInfo->header.frame_id = this->rgbFrameId.value();
      
      pubRgbImg = newRgbImg;
      pubRgbInfo = newRgbInfo;
    }

    if (this->depthFrameId)
    {
      sensor_msgs::ImagePtr newDepthImg(new sensor_msgs::Image);
      sensor_msgs::CameraInfoPtr newDepthInfo(new sensor_msgs::CameraInfo);
      *newDepthImg = *depthImg;
      *newDepthInfo = *depthInfo;
      newDepthImg->header.frame_id = this->depthFrameId.value();
      newDepthInfo->header.frame_id = this->depthFrameId.value();
      
      pubDepthImg = newDepthImg;
      pubDepthInfo = newDepthInfo;
    }

    this->pub.publish(pubRgbImg, pubRgbInfo, pubDepthImg, pubDepthInfo);
  }
}

void RgbdCameraThrottleNodelet::cbPcl(const sensor_msgs::ImageConstPtr& rgbImg, const sensor_msgs::CameraInfoConstPtr& rgbInfo, const sensor_msgs::ImageConstPtr& depthImg, const sensor_msgs::CameraInfoConstPtr& depthInfo, const sensor_msgs::PointCloud2ConstPtr& pcl)
{
  if (this->rate)
  {
    if (this->lastUpdate + this->rate.value().expectedCycleTime() > ros::Time::now())
    {
      NODELET_DEBUG("throttle last update at %f skipping", this->lastUpdate.toSec());
      return;
    }
  }

  this->lastUpdate = ros::Time::now();

  if (!this->rgbFrameId.has_value() && !this->depthFrameId.has_value())
  {
    this->pub.publish(rgbImg, rgbInfo, depthImg, depthInfo, pcl);
  } else {
    sensor_msgs::ImageConstPtr pubRgbImg = rgbImg;
    sensor_msgs::CameraInfoConstPtr pubRgbInfo = rgbInfo;
    sensor_msgs::ImageConstPtr pubDepthImg = depthImg;
    sensor_msgs::CameraInfoConstPtr pubDepthInfo = depthInfo;

    if (this->rgbFrameId)
    {
      sensor_msgs::ImagePtr newRgbImg(new sensor_msgs::Image);
      sensor_msgs::CameraInfoPtr newRgbInfo(new sensor_msgs::CameraInfo);
      *newRgbImg = *rgbImg;
      *newRgbInfo = *rgbInfo;
      newRgbImg->header.frame_id = this->rgbFrameId.value();
      newRgbInfo->header.frame_id = this->rgbFrameId.value();

      pubRgbImg = newRgbImg;
      pubRgbInfo = newRgbInfo;
    }

    if (this->depthFrameId)
    {
      sensor_msgs::ImagePtr newDepthImg(new sensor_msgs::Image);
      sensor_msgs::CameraInfoPtr newDepthInfo(new sensor_msgs::CameraInfo);
      *newDepthImg = *depthImg;
      *newDepthInfo = *depthInfo;
      newDepthImg->header.frame_id = this->depthFrameId.value();
      newDepthInfo->header.frame_id = this->depthFrameId.value();

      pubDepthImg = newDepthImg;
      pubDepthInfo = newDepthInfo;
    }

    this->pub.publish(pubRgbImg, pubRgbInfo, pubDepthImg, pubDepthInfo, pcl);
  }
}

void RgbdCameraThrottleNodelet::img_connect_cb(const image_transport::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 1 && !this->sub)
    this->onFirstConnect();
}

void RgbdCameraThrottleNodelet::info_connect_cb(const ros::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 1 && !this->sub)
    this->onFirstConnect();
}

void RgbdCameraThrottleNodelet::img_disconnect_cb(const image_transport::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 0 && this->sub)
    this->onLastDisconnect();
}

void RgbdCameraThrottleNodelet::info_disconnect_cb(const ros::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 0 && this->sub)
    this->onLastDisconnect();
}

void RgbdCameraThrottleNodelet::onFirstConnect()
{
  const auto& defaultTransport = this->getPrivateNodeHandle().param("image_transport", std::string("raw"));
  image_transport::TransportHints hintsRgb(defaultTransport, {}, this->getPrivateNodeHandle(), "image_transport_rgb");
  image_transport::TransportHints hintsDepth(defaultTransport, {}, this->getPrivateNodeHandle(), "image_transport_depth");
  if (this->subscribePcl)
  {
    NODELET_DEBUG("Started lazy-subscription to %s, %s and %s", this->subRgbNh.resolveName(this->subRGBBaseName).c_str(),
                  this->subDepthNh.resolveName(this->subDepthBaseName).c_str(),
                  this->subPclNh.resolveName("points_in").c_str());
    this->sub = this->subTransport->subscribeRgbdCamera(this->subRGBBaseName, this->subDepthBaseName, "points_in", this->queueSize, &RgbdCameraThrottleNodelet::cbPcl, this, hintsRgb, hintsDepth);
  } else
  {
    NODELET_DEBUG("Started lazy-subscription to %s and %s", this->subRgbNh.resolveName(this->subRGBBaseName).c_str(),
                  this->subDepthNh.resolveName(this->subDepthBaseName).c_str());
    this->sub = this->subTransport->subscribeRgbdCamera(this->subRGBBaseName, this->subDepthBaseName, this->queueSize, &RgbdCameraThrottleNodelet::cb, this, hintsRgb, hintsDepth);
  }
}

void RgbdCameraThrottleNodelet::onLastDisconnect()
{
  if (this->subscribePcl)
    NODELET_DEBUG("Stopped lazy-subscription to %s, %s and %s", this->sub.value().getRGBTopic().c_str(), this->sub.value().getDepthTopic().c_str(), this->sub.value().getPclTopic().c_str());
  else
    NODELET_DEBUG("Stopped lazy-subscription to %s and %s", this->sub.value().getRGBTopic().c_str(), this->sub.value().getDepthTopic().c_str());
  this->sub.value().shutdown();
  this->sub.reset();
}

}

PLUGINLIB_EXPORT_CLASS(camera_throttle::RgbdCameraThrottleNodelet, nodelet::Nodelet)