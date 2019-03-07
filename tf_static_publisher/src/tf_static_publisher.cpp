#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_static_publisher",
            ros::init_options::AnonymousName);
  ros::NodeHandle pnh("~");

  using XmlRpc::XmlRpcValue;
  XmlRpcValue transforms;
  pnh.param("transforms", transforms, transforms);
  ROS_ASSERT_MSG(transforms.getType() == XmlRpcValue::TypeArray,
                 "Parameter must be array of arrays.");

  std::vector<geometry_msgs::TransformStamped> msgs;
  msgs.reserve(transforms.size());
  for (int i = 0; i < transforms.size(); ++i)
  {
    ROS_ASSERT_MSG(transforms[i].getType() == XmlRpcValue::TypeArray,
                   "Parameter must be array of arrays.");
    
    geometry_msgs::TransformStamped msg;
    switch (transforms[i].size())
    {
    case 8: // [x y z yaw pitch roll 'parent' 'child']
    {
      msg.transform.translation.x = static_cast<double>(transforms[i][0]);
      msg.transform.translation.y = static_cast<double>(transforms[i][1]);
      msg.transform.translation.z = static_cast<double>(transforms[i][2]);

      double halfYaw = static_cast<double>(transforms[i][3]) / 2;
      double halfPitch = static_cast<double>(transforms[i][4]) / 2;
      double halfRoll = static_cast<double>(transforms[i][5]) / 2;
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

      msg.header.frame_id = static_cast<std::string>(transforms[i][6]);
      msg.header.stamp = ros::Time::now();
      msg.child_frame_id = static_cast<std::string>(transforms[i][7]);
      break;
    }
    case 9: // [x y z qx qy qz qw 'parent' 'child']
    {
      msg.transform.translation.x = static_cast<double>(transforms[i][0]);
      msg.transform.translation.y = static_cast<double>(transforms[i][1]);
      msg.transform.translation.z = static_cast<double>(transforms[i][2]);

      msg.transform.rotation.x = static_cast<double>(transforms[i][3]);
      msg.transform.rotation.y = static_cast<double>(transforms[i][4]);
      msg.transform.rotation.z = static_cast<double>(transforms[i][5]);
      msg.transform.rotation.w = static_cast<double>(transforms[i][6]);

      msg.header.frame_id = static_cast<std::string>(transforms[i][7]);
      msg.header.stamp = ros::Time::now();
      msg.child_frame_id = static_cast<std::string>(transforms[i][8]);
      break;
    }
    default:
    {
      ROS_ERROR("Parameter must be array of arrays with 8 or 9 elements.");
      continue;
    }
    } // switch
    msgs.push_back(msg);
  }
  
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
