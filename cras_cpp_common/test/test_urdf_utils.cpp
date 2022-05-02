/**
 * \file
 * \brief Unit test for urdf_utils.h.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <Eigen/Geometry>

#include <urdf_model/model.h>

#include <cras_cpp_common/urdf_utils.h>

TEST(UrdfEigen, VectorTranslation)  // NOLINT
{
  urdf::Vector3 uPosition(1.0, 2.0, 3.0);

  const auto eTranslation = cras::toEigen(uPosition);

  EXPECT_DOUBLE_EQ(1.0, eTranslation.x());
  EXPECT_DOUBLE_EQ(2.0, eTranslation.y());
  EXPECT_DOUBLE_EQ(3.0, eTranslation.z());
}

TEST(UrdfEigen, RotationQuaternion)  // NOLINT
{
  urdf::Rotation uRotation(0.0, 0.0, M_SQRT1_2, M_SQRT1_2);

  const auto eQuaternion = cras::toEigen(uRotation);

  EXPECT_DOUBLE_EQ(0.0, eQuaternion.x());
  EXPECT_DOUBLE_EQ(0.0, eQuaternion.y());
  EXPECT_DOUBLE_EQ(M_SQRT1_2, eQuaternion.z());
  EXPECT_DOUBLE_EQ(M_SQRT1_2, eQuaternion.z());
}

TEST(UrdfEigen, PoseTransform)  // NOLINT
{
  urdf::Pose uPose;
  uPose.position = {1.0, 2.0, 3.0};
  uPose.rotation = {0.0, 0.0, M_SQRT1_2, M_SQRT1_2};

  const auto ePose = cras::toEigen(uPose);

  EXPECT_DOUBLE_EQ(1.0, ePose.translation().x());
  EXPECT_DOUBLE_EQ(2.0, ePose.translation().y());
  EXPECT_DOUBLE_EQ(3.0, ePose.translation().z());

	Eigen::Quaterniond eQuaternion(ePose.rotation());
  EXPECT_DOUBLE_EQ(0.0, eQuaternion.x());
  EXPECT_DOUBLE_EQ(0.0, eQuaternion.y());
  EXPECT_DOUBLE_EQ(M_SQRT1_2, eQuaternion.z());
  EXPECT_DOUBLE_EQ(M_SQRT1_2, eQuaternion.w());
}

TEST(EigenUrdf, TranslationVector)  // NOLINT
{
	Eigen::Translation3d eTranslation(1.0, 2.0, 3.0);

	const auto uPosition = cras::toURDF(eTranslation);

	EXPECT_DOUBLE_EQ(1.0, uPosition.x);
	EXPECT_DOUBLE_EQ(2.0, uPosition.y);
	EXPECT_DOUBLE_EQ(3.0, uPosition.z);
}

TEST(EigenUrdf, VectorVector)  // NOLINT
{
	Eigen::Vector3d eVector(1.0, 2.0, 3.0);

	const auto uPosition = cras::toURDF(eVector);

	EXPECT_DOUBLE_EQ(1.0, uPosition.x);
	EXPECT_DOUBLE_EQ(2.0, uPosition.y);
	EXPECT_DOUBLE_EQ(3.0, uPosition.z);
}

TEST(EigenUrdf, QuaternionRotation)  // NOLINT
{
	Eigen::Quaterniond eQuaternion(M_SQRT1_2, 0.0, 0.0, M_SQRT1_2);

	const auto uRotation = cras::toURDF(eQuaternion);

	EXPECT_DOUBLE_EQ(0.0, uRotation.x);
	EXPECT_DOUBLE_EQ(0.0, uRotation.y);
	EXPECT_DOUBLE_EQ(M_SQRT1_2, uRotation.z);
	EXPECT_DOUBLE_EQ(M_SQRT1_2, uRotation.w);
}

TEST(EigenUrdf, TransformPose)  // NOLINT
{
	Eigen::Isometry3d eTransform = Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::Quaterniond(M_SQRT1_2, 0, 0, M_SQRT1_2);

	const auto uPose = cras::toURDF(eTransform);

	EXPECT_DOUBLE_EQ(1.0, uPose.position.x);
	EXPECT_DOUBLE_EQ(2.0, uPose.position.y);
	EXPECT_DOUBLE_EQ(3.0, uPose.position.z);
	
	EXPECT_DOUBLE_EQ(0.0, uPose.rotation.x);
	EXPECT_DOUBLE_EQ(0.0, uPose.rotation.y);
	EXPECT_DOUBLE_EQ(M_SQRT1_2, uPose.rotation.z);
	EXPECT_DOUBLE_EQ(M_SQRT1_2, uPose.rotation.w);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}