// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Utilities for working with transforms.
 * \author Martin Pecka
 */

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cras_cpp_common/tf2_utils.hpp>

namespace cras {

void getRPY(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw) {
  tf2::Matrix3x3 tmp_mat(quat);
  tmp_mat.getRPY(roll, pitch, yaw);
}

void getRPY(const geometry_msgs::msg::Quaternion& quat, double& roll, double& pitch, double& yaw) {
  tf2::Quaternion tmp_quat;
  tf2::fromMsg(quat, tmp_quat);
  getRPY(tmp_quat, roll, pitch, yaw);
}

double getRoll(const tf2::Quaternion& quat) {
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return roll;
}

double getRoll(const geometry_msgs::msg::Quaternion& quat) {
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return roll;
}

double getPitch(const tf2::Quaternion& quat) {
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return pitch;
}

double getPitch(const geometry_msgs::msg::Quaternion& quat) {
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return pitch;
}

double getYaw(const tf2::Quaternion& quat) {
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return yaw;
}

double getYaw(const geometry_msgs::msg::Quaternion& quat) {
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return yaw;
}

}  // namespace cras
