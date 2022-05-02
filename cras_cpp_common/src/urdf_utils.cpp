/**
 * \file
 * \brief Utilities for conversions between URDF and other types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <Eigen/Geometry>

#include <urdf_model/model.h>

#include <cras_cpp_common/urdf_utils.h>

namespace cras
{

Eigen::Translation3d toEigen(const urdf::Vector3& position)
{
	return {position.x, position.y, position.z};
}

Eigen::Quaterniond toEigen(const urdf::Rotation& rotation)
{
	// W is first in Quaterniond constructor!
	return {rotation.w, rotation.x, rotation.y, rotation.z};
}

Eigen::Isometry3d toEigen(const urdf::Pose& pose)
{
	return cras::toEigen(pose.position) * cras::toEigen(pose.rotation);
}

urdf::Vector3 toURDF(const Eigen::Translation3d& translation)
{
	return {translation.x(), translation.y(), translation.z()};
}

urdf::Vector3 toURDF(const Eigen::Vector3d& vector)
{
	return {vector.x(), vector.y(), vector.z()};
}

urdf::Rotation toURDF(const Eigen::Quaterniond& rotation)
{
	// W is last in Rotation constructor!
	return {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
}

urdf::Pose toURDF(const Eigen::Isometry3d& pose)
{
	urdf::Pose urdfPose;
	urdfPose.position = cras::toURDF(pose.translation());
	urdfPose.rotation = cras::toURDF(Eigen::Quaterniond(pose.rotation()));
	return urdfPose;
}

}
