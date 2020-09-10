#include <camera_throttle/camera_throttle.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.hpp>
#include <memory>

#include <cv_bridge/cv_bridge.h>

namespace camera_throttle
{

struct CameraThrottleNodelet::CameraThrottlePrivate
{
  cv::Mat opencvMat;
};

CameraThrottleNodelet::CameraThrottleNodelet() : data(new CameraThrottlePrivate)
{

}

void CameraThrottleNodelet::onInit()
{
  auto pnh = this->getPrivateNodeHandle();
  if (pnh.hasParam("rate"))
    this->rate = this->getParam(pnh, "rate", ros::Rate(10.0), "Hz");
  else
    NODELET_INFO("No rate limiting");
  this->queueSize = this->getParam(pnh, "queue_size", 10u, "messages");
  this->subBaseName = this->getParam(pnh, "sub_base_name", "image_raw");
  this->pubBaseName = this->getParam(pnh, "pub_base_name", this->subBaseName);

  if (pnh.hasParam("fix_frame_id"))
    this->frameId = this->getParam(pnh, "fix_frame_id", "");
  if (this->frameId.has_value() && this->frameId.value().empty())
    this->frameId.reset();
  if (this->frameId)
    NODELET_INFO("Fixing RGB frame_id to %s", this->frameId->c_str());

  this->flipHorizontal = this->getParam(pnh, "flip_horizontal", false);
  this->flipVertical = this->getParam(pnh, "flip_vertical", false);

  this->subNh = ros::NodeHandle(this->getNodeHandle(), "camera_in");
  this->subTransport = std::make_unique<image_transport::ImageTransport>(this->subNh);

  this->pubNh = ros::NodeHandle(this->getNodeHandle(), "camera_out");
  this->pubTransport = std::make_unique<image_transport::ImageTransport>(this->pubNh);
  this->pub = this->pubTransport->advertiseCamera(this->pubBaseName, this->queueSize,
    boost::bind(&CameraThrottleNodelet::img_connect_cb, this, _1),
    boost::bind(&CameraThrottleNodelet::img_disconnect_cb, this, _1),
    boost::bind(&CameraThrottleNodelet::info_connect_cb, this, _1),
    boost::bind(&CameraThrottleNodelet::info_disconnect_cb, this, _1));
}

void CameraThrottleNodelet::cb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info)
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
  if (!this->frameId.has_value() && !this->flipHorizontal && !this->flipVertical)
  {
    this->pub.publish(img, info);
  } else {
    sensor_msgs::ImagePtr newImg(new sensor_msgs::Image);
    sensor_msgs::CameraInfoPtr newInfo(new sensor_msgs::CameraInfo);

    *newImg = *img;
    *newInfo = *info;

    if (this->flipVertical || this->flipHorizontal)
    {
      int flipValue;
      if (this->flipVertical && this->flipHorizontal)
        flipValue = -1;
      else if (this->flipVertical)
        flipValue = 0;
      else
        flipValue = 1;

      cv_bridge::CvImagePtr cvImage;
      try
      {
        cvImage = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::flip(cvImage->image, this->data->opencvMat, flipValue);

      newImg = cv_bridge::CvImage(img->header, "bgr8", this->data->opencvMat).toImageMsg();
    }

    if (this->frameId)
    {
      newImg->header.frame_id = this->frameId.value();
      newInfo->header.frame_id = this->frameId.value();
    }

    this->pub.publish(newImg, newInfo);
  }
}

void CameraThrottleNodelet::img_connect_cb(const image_transport::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 1 && !this->sub)
    this->onFirstConnect();
}

void CameraThrottleNodelet::info_connect_cb(const ros::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 1 && !this->sub)
    this->onFirstConnect();
}

void CameraThrottleNodelet::img_disconnect_cb(const image_transport::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 0 && this->sub)
    this->onLastDisconnect();
}

void CameraThrottleNodelet::info_disconnect_cb(const ros::SingleSubscriberPublisher& status)
{
  std::lock_guard<std::mutex> g(this->publishersMutex);
  if (this->pub.getNumSubscribers() == 0 && this->sub)
    this->onLastDisconnect();
}

void CameraThrottleNodelet::onFirstConnect()
{
  NODELET_DEBUG("Started lazy-subscription to %s", this->subNh.resolveName(this->subBaseName).c_str());
  this->sub = this->subTransport->subscribeCamera(this->subBaseName, this->queueSize, &CameraThrottleNodelet::cb, this);
}

void CameraThrottleNodelet::onLastDisconnect()
{
  NODELET_DEBUG("Stopped lazy-subscription to %s", this->sub.value().getTopic().c_str());
  this->sub.value().shutdown();
  this->sub.reset();
}

}

PLUGINLIB_EXPORT_CLASS(camera_throttle::CameraThrottleNodelet, nodelet::Nodelet)