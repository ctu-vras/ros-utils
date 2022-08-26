/**
 * \file
 * \brief Unit test for tf2_sensor_msgs.h.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>

#include <cras_cpp_common/tf2_sensor_msgs.h>

struct Point
{
  float x;
  float y;
  float z;
  float vp_x;
  float vp_y;
  float vp_z;
  float normal_x;
  float normal_y;
  float normal_z;
  float test_x;
  float test_y;
  float test_z;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t intensity;
};

const static std::vector<Point> pointsIn =  // NOLINT
{
  {1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 1, 1, 0, 128, 255, 10000},
  {10, 20, 30, 10, 20, 30, 10, 20, 30, 1, 1, 1, 1, 129, 254, 1000},
  {-1, -2, -3, -1, -2, -3, -1, -2, -3, 1, 1, 1, 2, 130, 253, 1001},
  {-10, -20, -30, -10, -20, -30, -10, -20, -30, 1, 1, 1, 3, 131, 252, 1002},
};

const static std::vector<Point> pointsOut =  // NOLINT
{
  {-2, 0, -2, -2, 0, -2, -3, 2, 1, -1, 1, 1, 0, 128, 255, 10000},
  {-29, 18, 7, -29, 18, 7, -30, 20, 10, -1, 1, 1, 1, 129, 254, 1000},
  {4, -4, -4, 4, -4, -4, 3, -2, -1, -1, 1, 1, 2, 130, 253, 1001},
  {31, -22, -13, 31, -22, -13, 30, -20, -10, -1, 1, 1, 3, 131, 252, 1002},
};

/**
 * \brief Create a cloud used for the tests.
 * \param[in] width Width of the cloud.
 * \return The created cloud.
 */
sensor_msgs::PointCloud2 createCloud(const uint32_t width)
{
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp.sec = 2;
  msg.header.frame_id = "odom";
  msg.width = width;
  msg.is_dense = true;
  msg.is_bigendian = false;
  
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2Fields(14,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::PointField::FLOAT32,
    "vp_x", 1, sensor_msgs::PointField::FLOAT32,
    "vp_y", 1, sensor_msgs::PointField::FLOAT32,
    "vp_z", 1, sensor_msgs::PointField::FLOAT32,
    "normal_x", 1, sensor_msgs::PointField::FLOAT32,
    "normal_y", 1, sensor_msgs::PointField::FLOAT32,
    "normal_z", 1, sensor_msgs::PointField::FLOAT32,
    "test_x", 1, sensor_msgs::PointField::FLOAT32,
    "test_y", 1, sensor_msgs::PointField::FLOAT32,
    "test_z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::UINT16);

  mod.resize(4);

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(msg, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(msg, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(msg, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(msg, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(msg, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(msg, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(msg, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(msg, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(msg, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(msg, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(msg, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };
  
  for (const auto& point : pointsIn)
  {
    *it_x = point.x;
    *it_y = point.y;
    *it_z = point.z;
    *it_vp_x = point.vp_x;
    *it_vp_y = point.vp_y;
    *it_vp_z = point.vp_z;
    *it_normal_x = point.normal_x;
    *it_normal_y = point.normal_y;
    *it_normal_z = point.normal_z;
    *it_test_x = point.test_x;
    *it_test_y = point.test_y;
    *it_test_z = point.test_z;
    *it_r = point.r;
    *it_g = point.g;
    *it_b = point.b;
    *it_intensity = point.intensity;

    nextPoint();
  }
  
  return msg;
}

/**
 * \brief Create a transform to be used in the tests.
 * \return The transform.
 */
geometry_msgs::TransformStamped createTestTf()
{
  geometry_msgs::TransformStamped tf;
  tf.header.stamp.sec = 2;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "odom";
  tf.transform.translation.x = 1;
  tf.transform.translation.y = -2;
  tf.transform.translation.z = -3;
  // rotate 90 deg around y-axis
  tf.transform.rotation.x = tf.transform.rotation.z = 0;
  tf.transform.rotation.y = -M_SQRT1_2;
  tf.transform.rotation.w = M_SQRT1_2;

  return tf;
}

TEST(TF2SensorMsgs, TransformChannelBadField)  // NOLINT
{
  const auto msg = createCloud(2);

  sensor_msgs::PointCloud2 out = msg;
  const auto tf = createTestTf().transform;
  EXPECT_THROW(cras::transformChannel(out, tf, "bad", cras::CloudChannelType::POINT), std::runtime_error);
}

TEST(TF2SensorMsgs, TransformChannel)  // NOLINT
{
  const auto msg = createCloud(2);

  sensor_msgs::PointCloud2 out = msg;
  cras::transformChannel(out, createTestTf().transform, "vp_", cras::CloudChannelType::POINT);

  ASSERT_EQ("odom", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(msg.fields, out.fields);
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_EQ(msg.point_step, out.point_step);
  ASSERT_EQ(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(out, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(out, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(out, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(out, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };

  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& point = pointsOut[i];
    const auto& pointIn = pointsIn[i];

    EXPECT_NEAR(pointIn.x, *it_x, 1e-6);
    EXPECT_NEAR(pointIn.y, *it_y, 1e-6);
    EXPECT_NEAR(pointIn.z, *it_z, 1e-6);

    EXPECT_NEAR(point.vp_x, *it_vp_x, 1e-6);
    EXPECT_NEAR(point.vp_y, *it_vp_y, 1e-6);
    EXPECT_NEAR(point.vp_z, *it_vp_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(pointIn.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(pointIn.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(pointIn.normal_z, *it_normal_z, 1e-6);

    // should not be changed
    EXPECT_NEAR(pointIn.test_x, *it_test_x, 1e-6);
    EXPECT_NEAR(pointIn.test_y, *it_test_y, 1e-6);
    EXPECT_NEAR(pointIn.test_z, *it_test_z, 1e-6);

    EXPECT_EQ(pointIn.r, *it_r);
    EXPECT_EQ(pointIn.g, *it_g);
    EXPECT_EQ(pointIn.b, *it_b);

    EXPECT_EQ(pointIn.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudOrganized)  // NOLINT
{
  const auto msg = createCloud(2);

  tf2_ros::Buffer buffer;
  buffer.setTransform(createTestTf(), "test");

  sensor_msgs::PointCloud2 out;
  cras::transformWithChannels(msg, out, buffer, "base_link");

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(msg.fields, out.fields);
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_EQ(msg.point_step, out.point_step);
  ASSERT_EQ(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(out, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(out, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(out, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(out, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };

  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& point = pointsOut[i];
    const auto& pointIn = pointsIn[i];

    EXPECT_NEAR(point.x, *it_x, 1e-6);
    EXPECT_NEAR(point.y, *it_y, 1e-6);
    EXPECT_NEAR(point.z, *it_z, 1e-6);

    EXPECT_NEAR(point.vp_x, *it_vp_x, 1e-6);
    EXPECT_NEAR(point.vp_y, *it_vp_y, 1e-6);
    EXPECT_NEAR(point.vp_z, *it_vp_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(point.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(point.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(point.normal_z, *it_normal_z, 1e-6);

    // should not be changed
    EXPECT_NEAR(pointIn.test_x, *it_test_x, 1e-6);
    EXPECT_NEAR(pointIn.test_y, *it_test_y, 1e-6);
    EXPECT_NEAR(pointIn.test_z, *it_test_z, 1e-6);

    EXPECT_EQ(point.r, *it_r);
    EXPECT_EQ(point.g, *it_g);
    EXPECT_EQ(point.b, *it_b);

    EXPECT_EQ(point.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudUnorganized)  // NOLINT
{
  const auto msg = createCloud(1);

  tf2_ros::Buffer buffer;
  buffer.setTransform(createTestTf(), "test");

  sensor_msgs::PointCloud2 out;
  cras::transformWithChannels(msg, out, buffer, "base_link");

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(msg.fields, out.fields);
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_EQ(msg.point_step, out.point_step);
  ASSERT_EQ(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(out, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(out, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(out, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(out, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };

  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& point = pointsOut[i];
    const auto& pointIn = pointsIn[i];

    EXPECT_NEAR(point.x, *it_x, 1e-6);
    EXPECT_NEAR(point.y, *it_y, 1e-6);
    EXPECT_NEAR(point.z, *it_z, 1e-6);

    EXPECT_NEAR(point.vp_x, *it_vp_x, 1e-6);
    EXPECT_NEAR(point.vp_y, *it_vp_y, 1e-6);
    EXPECT_NEAR(point.vp_z, *it_vp_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(point.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(point.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(point.normal_z, *it_normal_z, 1e-6);

    // should not be changed
    EXPECT_NEAR(pointIn.test_x, *it_test_x, 1e-6);
    EXPECT_NEAR(pointIn.test_y, *it_test_y, 1e-6);
    EXPECT_NEAR(pointIn.test_z, *it_test_z, 1e-6);

    EXPECT_EQ(point.r, *it_r);
    EXPECT_EQ(point.g, *it_g);
    EXPECT_EQ(point.b, *it_b);

    EXPECT_EQ(point.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudNoChannels)  // NOLINT
{
  const auto msg = createCloud(4);

  tf2_ros::Buffer buffer;
  buffer.setTransform(createTestTf(), "test");

  sensor_msgs::PointCloud2 out;
  cras::transformWithChannels(msg, out, buffer, "base_link", {});

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(msg.fields, out.fields);
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_EQ(msg.point_step, out.point_step);
  ASSERT_EQ(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(out, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(out, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(out, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(out, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };

  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& point = pointsOut[i];
    const auto& pointIn = pointsIn[i];
    
    EXPECT_NEAR(pointIn.x, *it_x, 1e-6);
    EXPECT_NEAR(pointIn.y, *it_y, 1e-6);
    EXPECT_NEAR(pointIn.z, *it_z, 1e-6);

    EXPECT_NEAR(pointIn.vp_x, *it_vp_x, 1e-6);
    EXPECT_NEAR(pointIn.vp_y, *it_vp_y, 1e-6);
    EXPECT_NEAR(pointIn.vp_z, *it_vp_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(pointIn.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(pointIn.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(pointIn.normal_z, *it_normal_z, 1e-6);

    // should not be changed
    EXPECT_NEAR(pointIn.test_x, *it_test_x, 1e-6);
    EXPECT_NEAR(pointIn.test_y, *it_test_y, 1e-6);
    EXPECT_NEAR(pointIn.test_z, *it_test_z, 1e-6);

    EXPECT_EQ(pointIn.r, *it_r);
    EXPECT_EQ(pointIn.g, *it_g);
    EXPECT_EQ(pointIn.b, *it_b);

    EXPECT_EQ(pointIn.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudSomeChannels)  // NOLINT
{
  const auto msg = createCloud(2);

  sensor_msgs::PointCloud2 out;
  cras::transformWithChannels(msg, out, createTestTf(),
    {{"", cras::CloudChannelType::POINT}, {"normal_", cras::CloudChannelType::DIRECTION}});

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(msg.fields, out.fields);
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_EQ(msg.point_step, out.point_step);
  ASSERT_EQ(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(out, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(out, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(out, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(out, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };
  
  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& point = pointsOut[i];
    const auto& pointIn = pointsIn[i];

    EXPECT_NEAR(point.x, *it_x, 1e-6);
    EXPECT_NEAR(point.y, *it_y, 1e-6);
    EXPECT_NEAR(point.z, *it_z, 1e-6);

    EXPECT_NEAR(pointIn.vp_x, *it_vp_x, 1e-6);
    EXPECT_NEAR(pointIn.vp_y, *it_vp_y, 1e-6);
    EXPECT_NEAR(pointIn.vp_z, *it_vp_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(point.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(point.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(point.normal_z, *it_normal_z, 1e-6);
    
    // should not be changed
    EXPECT_NEAR(pointIn.test_x, *it_test_x, 1e-6);
    EXPECT_NEAR(pointIn.test_y, *it_test_y, 1e-6);
    EXPECT_NEAR(pointIn.test_z, *it_test_z, 1e-6);

    EXPECT_EQ(point.r, *it_r);
    EXPECT_EQ(point.g, *it_g);
    EXPECT_EQ(point.b, *it_b);

    EXPECT_EQ(point.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudChangeDefaultChannels)  // NOLINT
{
  const auto msg = createCloud(2);

  cras::registerCloudChannelType("test_", cras::CloudChannelType::DIRECTION);
  
  sensor_msgs::PointCloud2 out;
  cras::transformWithChannels(msg, out, createTestTf());
  
  cras::unregisterCloudChannelType("test_");

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(msg.fields, out.fields);
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_EQ(msg.point_step, out.point_step);
  ASSERT_EQ(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(out, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<float> it_test_x(out, "test_x");
  sensor_msgs::PointCloud2Iterator<float> it_test_y(out, "test_y");
  sensor_msgs::PointCloud2Iterator<float> it_test_z(out, "test_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_vp_x; ++it_vp_y; ++it_vp_z; ++it_normal_x; ++it_normal_y; ++it_normal_z;
    ++it_test_x; ++it_test_y; ++it_test_z; ++it_rgb; ++it_intensity;
    it_r = &it_rgb[0];
    it_g = &it_rgb[1];
    it_b = &it_rgb[2];
  };
  
  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& point = pointsOut[i];
    const auto& pointIn = pointsIn[i];

    EXPECT_NEAR(point.x, *it_x, 1e-6);
    EXPECT_NEAR(point.y, *it_y, 1e-6);
    EXPECT_NEAR(point.z, *it_z, 1e-6);

    EXPECT_NEAR(point.vp_x, *it_vp_x, 1e-6);
    EXPECT_NEAR(point.vp_y, *it_vp_y, 1e-6);
    EXPECT_NEAR(point.vp_z, *it_vp_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(point.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(point.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(point.normal_z, *it_normal_z, 1e-6);

    // should be changed
    EXPECT_NEAR(point.test_x, *it_test_x, 1e-6);
    EXPECT_NEAR(point.test_y, *it_test_y, 1e-6);
    EXPECT_NEAR(point.test_z, *it_test_z, 1e-6);

    EXPECT_EQ(point.r, *it_r);
    EXPECT_EQ(point.g, *it_g);
    EXPECT_EQ(point.b, *it_b);

    EXPECT_EQ(point.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudOnlySomeChannels)  // NOLINT
{
  const auto msg = createCloud(2);

  sensor_msgs::PointCloud2 out;
  cras::transformOnlyChannels(msg, out, createTestTf(),
    {{"", cras::CloudChannelType::POINT},
     {"normal_", cras::CloudChannelType::DIRECTION},
     {"intensity", cras::CloudChannelType::SCALAR}});

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(7u, out.fields.size());
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_GT(msg.point_step, out.point_step);
  ASSERT_GT(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(out, "intensity");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z; ++it_normal_x; ++it_normal_y; ++it_normal_z; ++it_intensity;
  };
  
  for (size_t i = 0; i < pointsOut.size(); ++i)
  {
    const auto& pointIn = pointsIn[i];
    const auto& pointOut = pointsOut[i];

    EXPECT_NEAR(pointOut.x, *it_x, 1e-6);
    EXPECT_NEAR(pointOut.y, *it_y, 1e-6);
    EXPECT_NEAR(pointOut.z, *it_z, 1e-6);

    // should be rotated as vector, not as a point
    EXPECT_NEAR(pointOut.normal_x, *it_normal_x, 1e-6);
    EXPECT_NEAR(pointOut.normal_y, *it_normal_y, 1e-6);
    EXPECT_NEAR(pointOut.normal_z, *it_normal_z, 1e-6);

    EXPECT_EQ(pointOut.intensity, *it_intensity);

    nextPoint();
  }
}

TEST(TF2SensorMsgs, CreateTransformedCloudOnlyXYZ)  // NOLINT
{
  const auto msg = createCloud(2);

  sensor_msgs::PointCloud2 out;
  cras::transformOnlyXYZ(msg, out, createTestTf());

  ASSERT_EQ("base_link", out.header.frame_id);
  ASSERT_EQ(msg.header.stamp, out.header.stamp);
  ASSERT_EQ(msg.width, out.width);
  ASSERT_EQ(msg.height, out.height);
  ASSERT_EQ(3u, out.fields.size());
  ASSERT_EQ(msg.is_bigendian, out.is_bigendian);
  ASSERT_EQ(msg.is_dense, out.is_dense);
  ASSERT_GT(msg.point_step, out.point_step);
  ASSERT_GT(msg.row_step, out.row_step);

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

  const auto nextPoint = [&]()
  {
    ++it_x; ++it_y; ++it_z;
  };

  for (const auto& point : pointsOut)
  {
    EXPECT_NEAR(point.x, *it_x, 1e-6);
    EXPECT_NEAR(point.y, *it_y, 1e-6);
    EXPECT_NEAR(point.z, *it_z, 1e-6);

    nextPoint();
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}