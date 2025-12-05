#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Utilities for working with transforms.
 * \author Martin Pecka
 */

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

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
void getRPY(const ::geometry_msgs::msg::Quaternion& quat, double& roll, double& pitch, double& yaw);

/**
 * \brief Get roll from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \return Roll in radians.
 */
double getRoll(const ::tf2::Quaternion& quat);

/**
 * \brief Get roll from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \return Roll in radians.
 */
double getRoll(const ::geometry_msgs::msg::Quaternion& quat);

/**
 * \brief Get pitch from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \return Pitch in radians.
 */
double getPitch(const ::tf2::Quaternion& quat);

/**
 * \brief Get pitch from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \return Pitch in radians.
 */
double getPitch(const ::geometry_msgs::msg::Quaternion& quat);

/**
 * \brief Get yaw from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \return Yaw in radians.
 */
double getYaw(const ::tf2::Quaternion& quat);

/**
 * \brief Get yaw from the given quaternion.
 * \param[in] quat The quaternion to convert.
 * \return Yaw in radians.
 */
double getYaw(const ::geometry_msgs::msg::Quaternion& quat);

}
