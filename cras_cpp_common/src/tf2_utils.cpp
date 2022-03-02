/**
 * \file
 * \brief Utilities for working with transforms.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */
 
#include <cras_cpp_common/tf2_utils.hpp>

#include <geometry_msgs//Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace cras
{

void getRPY(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3 tmpMat(quat);
  tmpMat.getRPY(roll, pitch, yaw);
}

void getRPY(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tmpQuat;
  tf2::fromMsg(quat, tmpQuat);
  getRPY(tmpQuat, roll, pitch, yaw);
}

}