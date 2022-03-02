/**
 * \file
 * \brief Utilities for working with transforms.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

namespace cras
{

/**
 * \brief Get roll, pitch and yaw from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \param[out] roll Roll in radians.
 * \param[out] pitch Pitch in radians.
 * \param[out] yaw Yaw in radians.
 */
void getRPY(const ::tf2::Quaternion& quat, double& roll, double& pitch, double& yaw);

/**
 * \brief Get roll, pitch and yaw from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \param[out] roll Roll in radians.
 * \param[out] pitch Pitch in radians.
 * \param[out] yaw Yaw in radians.
 */
void getRPY(const ::geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw);

}
