#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <mutex>
#include <XmlRpcException.h>

using XmlRpc::XmlRpcValue;
typedef XmlRpcValue::ValueStruct::value_type KeyValue;
using geometry_msgs::TransformStamped;

TransformStamped value_to_transform(XmlRpcValue& value)
{
  ROS_ASSERT_MSG(value.getType() == XmlRpcValue::TypeArray
                 && (value.size() == 8 || value.size() == 9),
                 "Transform must be an array of 8 or 9 elements.");

  TransformStamped msg;
  // Header
  msg.header.frame_id = static_cast<std::string>(value[value.size() - 2]);
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = static_cast<std::string>(value[value.size() - 1]);
  // Translation
  msg.transform.translation.x = static_cast<double>(value[0]);
  msg.transform.translation.y = static_cast<double>(value[1]);
  msg.transform.translation.z = static_cast<double>(value[2]);
  // Rotation
  if (value.size() == 8)
  {
    // [x y z yaw pitch roll 'parent' 'child']
    double halfYaw = static_cast<double>(value[3]) / 2;
    double halfPitch = static_cast<double>(value[4]) / 2;
    double halfRoll = static_cast<double>(value[5]) / 2;
    double cy = std::cos(halfYaw);
    double sy = std::sin(halfYaw);
    double cp = std::cos(halfPitch);
    double sp = std::sin(halfPitch);
    double cr = std::cos(halfRoll);
    double sr = std::sin(halfRoll);

    msg.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    msg.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    msg.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    msg.transform.rotation.w = cr * cp * cy + sr * sp * sy;
  }
  else if (value.size() == 9)
  {
    // [x y z qx qy qz qw 'parent' 'child']
    msg.transform.rotation.x = static_cast<double>(value[3]);
    msg.transform.rotation.y = static_cast<double>(value[4]);
    msg.transform.rotation.z = static_cast<double>(value[5]);
    msg.transform.rotation.w = static_cast<double>(value[6]);
  }
  return msg;
}

namespace geometry_msgs
{
bool operator==(const TransformStamped &lhs, const TransformStamped &rhs)
{
  return lhs.header.frame_id == rhs.header.frame_id &&
         lhs.child_frame_id == rhs.child_frame_id &&
         lhs.transform.translation.x == rhs.transform.translation.x &&
         lhs.transform.translation.y == rhs.transform.translation.y &&
         lhs.transform.translation.z == rhs.transform.translation.z &&
         lhs.transform.rotation.x == rhs.transform.rotation.x &&
         lhs.transform.rotation.y == rhs.transform.rotation.y &&
         lhs.transform.rotation.z == rhs.transform.rotation.z &&
         lhs.transform.rotation.w == rhs.transform.rotation.w;
}
}


class TfStaticPublisher
{
  public: void loadAndPublish()
  {
    if (!ros::ok()) return;

    ROS_DEBUG("Reloading static transforms");

    XmlRpcValue transforms;
    {
      std::lock_guard<std::mutex> lock(this->paramMutex);
      if (!this->nh.hasParam("transforms"))
      {
        ROS_WARN_ONCE("Parameter 'transforms' was not found. Will not publish any transforms until it appears.");
        return;
      }

      try
      {
        this->nh.param("transforms", transforms, transforms);
      }
      catch (XmlRpc::XmlRpcException& e)
      {
        ROS_ERROR("Error reading transforms from parameter %s: %s, error code %i",
                  this->nh.resolveName("transforms").c_str(),
                  e.getMessage().c_str(), e.getCode());
        return;
      }
    }

    if (transforms.getType() != XmlRpcValue::TypeArray
        && transforms.getType() != XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Parameter %s must be array or struct of arrays, got type %i.",
                this->nh.resolveName("transforms").c_str(), transforms.getType());
      return;
    }

    std::vector<TransformStamped> msgs;
    msgs.reserve(static_cast<size_t>(transforms.size()));

    if (transforms.getType() == XmlRpcValue::TypeArray)
    {
      // Do not use range-based for-loop! It's used for structs, not arrays!
      for (size_t i = 0; i < transforms.size(); ++i)
      {
        auto& transform = transforms[i];
        try
        {
          msgs.push_back(value_to_transform(transform));
        }
        catch (XmlRpc::XmlRpcException& e)
        {
          ROS_ERROR("Error reading static transform nr. %lu: %s, error code %i",
              i, e.getMessage().c_str(), e.getCode());
        }
      }
    }
    else
    {
      for (KeyValue &v : transforms)
      {
        try
        {
          msgs.push_back(value_to_transform(v.second));
        }
        catch (XmlRpc::XmlRpcException& e)
        {
          ROS_ERROR("Error reading static transform %s: %s, error code %i",
              v.first.c_str(), e.getMessage().c_str(), e.getCode());
        }
      }
    }

    if (msgs != this->lastTransforms)
    {
      broadcaster.sendTransform(msgs);

      for (const auto &msg : msgs)
      {
        ROS_DEBUG("Publishing static transform from %s to %s.",
                  msg.header.frame_id.c_str(),
                  msg.child_frame_id.c_str());
      }
      this->lastTransforms.clear();
      this->lastTransforms = msgs;
    }
  }

  protected: bool onReloadTrigger(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& resp)
  {
    this->loadAndPublish();
    resp.success = true;
    return true;
  }

  public: explicit TfStaticPublisher(ros::NodeHandle nh) : nh(nh)
  {
    triggerServer = nh.advertiseService("reload", &TfStaticPublisher::onReloadTrigger, this);
  }

  private: ros::NodeHandle nh;
  private: ros::ServiceServer triggerServer;
  private: std::mutex paramMutex;
  private: tf2_ros::StaticTransformBroadcaster broadcaster;
  private: std::vector<geometry_msgs::TransformStamped> lastTransforms;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_static_publisher",
            ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");

  TfStaticPublisher node(pnh);

  const auto reload_frequency = pnh.param("reload_frequency", 0.0);

  node.loadAndPublish();

  if (reload_frequency == 0.0)
  {
    ros::spin();
  }
  else
  {
    ros::Rate rate(reload_frequency);
    std::thread reload_thread([&rate, &node]()
    {
      while (ros::ok())
      {
        rate.sleep();
        node.loadAndPublish();
      }
    });

    ros::spin();
    reload_thread.join();
  }

  return 0;
}
