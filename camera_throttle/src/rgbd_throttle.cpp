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

  this->subNh = ros::NodeHandle(this->getNodeHandle(), "camera_in");
  this->subTransport = std::make_unique<RgbdImageTransport>(this->subNh, this->getNodeHandle());

  this->pubNh = ros::NodeHandle(this->getNodeHandle(), "camera_out");
  this->pubTransport = std::make_unique<RgbdImageTransport>(this->pubNh, this->getNodeHandle());

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
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1));
  else
    this->pub = this->pubTransport->advertiseRgbdCamera(
        this->pubRGBBaseName, this->pubDepthBaseName, this->queueSize,
        boost::bind(&RgbdCameraThrottleNodelet::img_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::img_disconnect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_connect_cb, this, _1),
        boost::bind(&RgbdCameraThrottleNodelet::info_disconnect_cb, this, _1));
}

void RgbdCameraThrottleNodelet::cb(const sensor_msgs::ImageConstPtr& rgbImg, const sensor_msgs::ImageConstPtr& depthImg, const sensor_msgs::CameraInfoConstPtr& info)
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
  this->pub.publish(rgbImg, depthImg, info);
}

void RgbdCameraThrottleNodelet::cbPcl(const sensor_msgs::ImageConstPtr& rgbImg, const sensor_msgs::ImageConstPtr& depthImg, const sensor_msgs::CameraInfoConstPtr& info, const sensor_msgs::PointCloud2ConstPtr& pcl)
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
  this->pub.publish(rgbImg, depthImg, info, pcl);
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
  if (this->subscribePcl)
  {
    NODELET_DEBUG("Started lazy-subscription to %s, %s and %s", this->subNh.resolveName(this->subRGBBaseName).c_str(),
                  this->subNh.resolveName(this->subDepthBaseName).c_str(),
                  this->subNh.resolveName("points_in").c_str());
    this->sub = this->subTransport->subscribeRgbdCamera(this->subRGBBaseName, this->subDepthBaseName, "points_in", this->queueSize, &RgbdCameraThrottleNodelet::cbPcl, this);
  } else
  {
    NODELET_DEBUG("Started lazy-subscription to %s and %s", this->subNh.resolveName(this->subRGBBaseName).c_str(),
                  this->subNh.resolveName(this->subDepthBaseName).c_str());
    this->sub = this->subTransport->subscribeRgbdCamera(this->subRGBBaseName, this->subDepthBaseName, this->queueSize, &RgbdCameraThrottleNodelet::cb, this);
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