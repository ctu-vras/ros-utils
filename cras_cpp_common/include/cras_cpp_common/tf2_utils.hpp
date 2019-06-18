#ifndef CRAS_CPP_COMMON_TF2_UTILS_HPP
#define CRAS_CPP_COMMON_TF2_UTILS_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

namespace cras
{
  void getRPY(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw);
  void getRPY(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw);
}

#endif //CRAS_CPP_COMMON_TF2_UTILS_HPP
