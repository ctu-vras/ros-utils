#pragma once

/**
 * \file
 * \brief Utilities for conversions between URDF and other types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <Eigen/Geometry>

#include <urdf_model/pose.h>

namespace cras
{

/**
 * \brief URDF Vector3 to Eigen translation.
 * \param[in] position The vector to convert.
 * \return The corresponding Eigen translation.
 */
::Eigen::Translation3d toEigen(const ::urdf::Vector3& position);

/**
 * \brief Convert URDF rotation to Eigen quaternion.
 * \param[in] rotation The rotation to convert.
 * \return The corresponding Eigen quaternion.
 */
::Eigen::Quaterniond toEigen(const ::urdf::Rotation& rotation);

/**
 * \brief Convert URDF pose to Eigen transform.
 * \param[in] pose The pose to convert.
 * \return The corresponding Eigen transform.
 */
::Eigen::Isometry3d toEigen(const ::urdf::Pose& pose);

/**
 * \brief Convert Eigen translation to URDF Vector3.
 * \param[in] translation The translation to convert.
 * \return The corresponding URDF Vector3.
 */
::urdf::Vector3 toURDF(const ::Eigen::Translation3d& translation);

/**
 * \brief Convert Eigen vector to URDF Vector3.
 * \param[in] vector The vector to convert.
 * \return The corresponding URDF Vector3.
 */
::urdf::Vector3 toURDF(const ::Eigen::Vector3d& vector);

/**
 * \brief Convert Eigen quaternion to URDF Rotation.
 * \param[in] rotation The quaternion to convert.
 * \return The corresponding URDF Rotation.
 */
::urdf::Rotation toURDF(const ::Eigen::Quaterniond& rotation);

/**
 * \brief Convert Eigen isometry to URDF Pose.
 * \param[in] pose The pose to convert.
 * \return The corresponding URDF Pose.
 */
::urdf::Pose toURDF(const ::Eigen::Isometry3d& pose);

}
