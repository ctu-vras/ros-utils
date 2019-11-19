#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>

using XmlRpc::XmlRpcValue;
typedef XmlRpcValue::ValueStruct::value_type KeyValue;
using geometry_msgs::TransformStamped;

TransformStamped value_to_transform(XmlRpcValue &value)
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_static_publisher",
            ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");

  XmlRpcValue transforms;
  pnh.param("transforms", transforms, transforms);
  ROS_ASSERT_MSG(transforms.getType() == XmlRpcValue::TypeArray
                 || transforms.getType() == XmlRpcValue::TypeStruct,
                 "Parameter must be array or struct of arrays.");

  std::vector<TransformStamped> msgs;
  msgs.reserve(static_cast<size_t>(transforms.size()));

  if (transforms.getType() == XmlRpcValue::TypeArray)
    for (int i = 0; i < transforms.size(); ++i)
      msgs.push_back(value_to_transform(transforms[i]));
  else
    for (KeyValue &v : transforms)
      msgs.push_back(value_to_transform(v.second));

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(msgs);
  for (const auto &msg : msgs)
  {
    ROS_INFO("Publishing static transform from %s to %s.",
             msg.header.frame_id.c_str(),
             msg.child_frame_id.c_str());
  }
  ros::spin();
  return 0;
}
