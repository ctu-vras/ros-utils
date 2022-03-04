/**
 * \file
 * \brief Unit test for cloud.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"
#include <cras_cpp_common/cloud.hpp>

void fillXYZ(sensor_msgs::PointCloud2& msg, sensor_msgs::PointCloud2Modifier& mod, size_t size, bool allZeros = false)
{
  mod.resize(size);

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  for (size_t i = 0; i < size; ++i)
  {
    *it_x = *it_y = *it_z = static_cast<float>(allZeros ? 0 : i);
    ++it_x; ++it_y; ++it_z;
  }
}

TEST(Cloud, NumPoints)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");

  mod.resize(3);
  EXPECT_EQ(3, cras::numPoints(msg));

  mod.resize(1);
  EXPECT_EQ(1, cras::numPoints(msg));

  mod.resize(0);
  EXPECT_EQ(0, cras::numPoints(msg));
}

TEST(Cloud, CreateFilteredCloudUnorganized)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msg, mod, 3);
  
  sensor_msgs::PointCloud2 out;
  CREATE_FILTERED_CLOUD(msg, out, true, *x_it == 2.0)

  EXPECT_EQ(1, cras::numPoints(out));
  EXPECT_EQ(1, out.width);
  EXPECT_EQ(1, out.height);

  sensor_msgs::PointCloud2ConstIterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> out_z(out, "z");

  EXPECT_EQ(2.0, *out_x);
  EXPECT_EQ(2.0, *out_y);
  EXPECT_EQ(2.0, *out_z);
}

TEST(Cloud, CreateFilteredCloudOrganized)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msg, mod, 4);

  msg.width = 2;
  msg.height = 2;
  msg.row_step = msg.row_step / 2;
  msg.is_dense = false;

  sensor_msgs::PointCloud2 out;
  CREATE_FILTERED_CLOUD(msg, out, true, int(*x_it) % 2 == 0)

  ASSERT_EQ(4, cras::numPoints(out));
  EXPECT_EQ(2, out.width);
  EXPECT_EQ(2, out.height);
  EXPECT_EQ(false, out.is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> out_z(out, "z");

  EXPECT_EQ(0.0, *out_x);
  EXPECT_EQ(0.0, *out_y);
  EXPECT_EQ(0.0, *out_z);

  ++out_x; ++out_y; ++out_z;

  EXPECT_TRUE(std::isnan(*out_x));
  EXPECT_TRUE(std::isnan(*out_y));
  EXPECT_TRUE(std::isnan(*out_z));

  ++out_x; ++out_y; ++out_z;

  EXPECT_EQ(2.0, *out_x);
  EXPECT_EQ(2.0, *out_y);
  EXPECT_EQ(2.0, *out_z);

  ++out_x; ++out_y; ++out_z;

  EXPECT_TRUE(std::isnan(*out_x));
  EXPECT_TRUE(std::isnan(*out_y));
  EXPECT_TRUE(std::isnan(*out_z));
}

TEST(Cloud, CreateFilteredCloudOrganizedDoNotKeep)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msg, mod, 4);

  msg.width = 2;
  msg.height = 2;
  msg.row_step = msg.row_step / 2;
  msg.is_dense = false;

  sensor_msgs::PointCloud2 out;
  CREATE_FILTERED_CLOUD(msg, out, false, int(*x_it) % 2 == 0)

  ASSERT_EQ(2, cras::numPoints(out));
  EXPECT_EQ(2, out.width);
  EXPECT_EQ(1, out.height);
  EXPECT_EQ(true, out.is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> out_z(out, "z");

  EXPECT_EQ(0.0, *out_x);
  EXPECT_EQ(0.0, *out_y);
  EXPECT_EQ(0.0, *out_z);

  ++out_x; ++out_y; ++out_z;

  EXPECT_EQ(2.0, *out_x);
  EXPECT_EQ(2.0, *out_y);
  EXPECT_EQ(2.0, *out_z);
}

TEST(Cloud, GenericConstIterator)  // NOLINT
{
  sensor_msgs::PointCloud2 msgOrig;
  sensor_msgs::PointCloud2Modifier mod(msgOrig);
  const auto& msg = msgOrig;

  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msgOrig, mod, 4);

  cras::GenericCloudConstIter it_g_x(msg, "x");
  cras::GenericCloudConstIter it_g_y(msg, "y");
  cras::GenericCloudConstIter it_g_z(msg, "z");
  
  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *reinterpret_cast<const float*>(it_g_x.rawData()));
    EXPECT_EQ(i, *reinterpret_cast<const float*>(it_g_y.rawData()));
    EXPECT_EQ(i, *reinterpret_cast<const float*>(it_g_z.rawData()));
    ++it_g_x; ++it_g_y; ++it_g_z;
  }
}

TEST(Cloud, GenericConstIteratorDataAs)  // NOLINT
{
  sensor_msgs::PointCloud2 msgOrig;
  sensor_msgs::PointCloud2Modifier mod(msgOrig);
  const auto& msg = msgOrig;

  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msgOrig, mod, 4);

  cras::GenericCloudConstIter it_g_x(msg, "x");
  cras::GenericCloudConstIter it_g_y(msg, "y");
  cras::GenericCloudConstIter it_g_z(msg, "z");
  
  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *it_g_x.dataAs<float>());
    EXPECT_EQ(i, *it_g_y.dataAs<float>());
    EXPECT_EQ(i, *it_g_z.dataAs<float>());
    ++it_g_x; ++it_g_y; ++it_g_z;
  }

  cras::GenericCloudConstIter it_g2(msg, "x");
  EXPECT_THROW(it_g2.dataAs<char>(), std::runtime_error);
  EXPECT_THROW(it_g2.dataAs<short>(), std::runtime_error);
  EXPECT_THROW(it_g2.dataAs<long>(), std::runtime_error);
  EXPECT_THROW(it_g2.dataAs<double>(), std::runtime_error);
  try
  {
    it_g2.dataAs<int32_t>();  // int32_t is also 4-byte, the same as float
  }
  catch (const std::runtime_error& e)
  {
    GTEST_NONFATAL_FAILURE_(e.what());
  }
}

TEST(Cloud, GenericIterator)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msg, mod, 4, true);

  cras::GenericCloudIter it_g_x(msg, "x");
  cras::GenericCloudIter it_g_y(msg, "y");
  cras::GenericCloudIter it_g_z(msg, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    auto f = static_cast<float>(i);
    *reinterpret_cast<float*>(it_g_x.rawData()) = f;
    *reinterpret_cast<float*>(it_g_y.rawData()) = f * 2;
    *reinterpret_cast<float*>(it_g_z.rawData()) = f * 3;
    ++it_g_x; ++it_g_y; ++it_g_z;
  }

  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *it_x);
    EXPECT_EQ(i * 2, *it_y);
    EXPECT_EQ(i * 3, *it_z);
    ++it_x; ++it_y; ++it_z;
  }
}

TEST(Cloud, GenericIteratorDataAs)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  fillXYZ(msg, mod, 4, true);

  cras::GenericCloudIter it_g_x(msg, "x");
  cras::GenericCloudIter it_g_y(msg, "y");
  cras::GenericCloudIter it_g_z(msg, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    auto f = static_cast<float>(i);
    *it_g_x.dataAs<float>() = f;
    *it_g_y.dataAs<float>() = f * 2;
    *it_g_z.dataAs<float>() = f * 3;
    ++it_g_x; ++it_g_y; ++it_g_z;
  }

  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *it_x);
    EXPECT_EQ(i * 2, *it_y);
    EXPECT_EQ(i * 3, *it_z);
    ++it_x; ++it_y; ++it_z;
  }

  cras::GenericCloudIter it_g2(msg, "x");
  EXPECT_THROW(it_g2.dataAs<char>(), std::runtime_error);
  EXPECT_THROW(it_g2.dataAs<short>(), std::runtime_error);
  EXPECT_THROW(it_g2.dataAs<long>(), std::runtime_error);
  EXPECT_THROW(it_g2.dataAs<double>(), std::runtime_error);
  try
  {
    it_g2.dataAs<int32_t>();  // int32_t is also 4-byte, the same as float
  }
  catch (const std::runtime_error& e)
  {
    GTEST_NONFATAL_FAILURE_(e.what());
  }
}

TEST(Cloud, GenericIteratorCopyData)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2 msg2;
  sensor_msgs::PointCloud2Modifier mod2(msg2);
  mod2.setPointCloud2FieldsByString(1, "xyz");
  
  fillXYZ(msg, mod, 4);
  fillXYZ(msg2, mod2, 4, true);

  cras::GenericCloudConstIter it_g_x(msg, "x");
  cras::GenericCloudConstIter it_g_y(msg, "y");
  cras::GenericCloudConstIter it_g_z(msg, "z");

  cras::GenericCloudIter it2_g_x(msg2, "x");
  cras::GenericCloudIter it2_g_y(msg2, "y");
  cras::GenericCloudIter it2_g_z(msg2, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    it2_g_x.copyData(it_g_x);
    it2_g_y.copyData(it_g_y);
    it2_g_z.copyData(it_g_z);
    ++it_g_x; ++it_g_y; ++it_g_z;
    ++it2_g_x; ++it2_g_y; ++it2_g_z;
  }

  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg2, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *it_x);
    EXPECT_EQ(i, *it_y);
    EXPECT_EQ(i, *it_z);
    ++it_x; ++it_y; ++it_z;
  }
}

TEST(Cloud, CopyChannelData)  // NOLINT
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  
  sensor_msgs::PointCloud2 msg2;
  sensor_msgs::PointCloud2Modifier mod2(msg2);
  mod2.setPointCloud2FieldsByString(1, "xyz");

  using F = sensor_msgs::PointField;
  sensor_msgs::PointCloud2 msg3;
  sensor_msgs::PointCloud2Modifier mod3(msg3);
  mod3.setPointCloud2Fields(8,
    "a", 1, F::FLOAT64, "b", 1, F::FLOAT32,
    "c", 1, F::INT8, "d", 1, F::INT16, "e", 1, F::INT32,
    "f", 1, F::UINT8, "g", 1, F::UINT16, "h", 1, F::UINT32);
  mod3.resize(4);
  
  fillXYZ(msg, mod, 4);
  fillXYZ(msg2, mod2, 4, true);
  
  cras::copyChannelData(msg, msg2, "x");

  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg2, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *it_x);
    EXPECT_EQ(0, *it_y);
    EXPECT_EQ(0, *it_z);
    ++it_x; ++it_y; ++it_z;
  }

  cras::copyChannelData(msg, msg2, "y");
  cras::copyChannelData(msg, msg2, "z");

  sensor_msgs::PointCloud2ConstIterator<float> it2_x(msg2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it2_y(msg2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it2_z(msg2, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(i, *it2_x);
    EXPECT_EQ(i, *it2_y);
    EXPECT_EQ(i, *it2_z);
    ++it2_x; ++it2_y; ++it2_z;
  }

  EXPECT_THROW(cras::copyChannelData(msg, msg3, "x"), std::runtime_error);
  EXPECT_THROW(cras::copyChannelData(msg3, msg, "x"), std::runtime_error);
  EXPECT_THROW(cras::copyChannelData(msg, msg3, "test"), std::runtime_error);
  mod3.resize(1);
  EXPECT_THROW(cras::copyChannelData(msg, msg3, "x"), std::runtime_error);
  EXPECT_THROW(cras::copyChannelData(msg3, msg, "x"), std::runtime_error);
  EXPECT_THROW(cras::copyChannelData(msg, msg3, "test"), std::runtime_error);
  mod2.resize(3);
  EXPECT_THROW(cras::copyChannelData(msg, msg2, "x"), std::runtime_error);
  EXPECT_THROW(cras::copyChannelData(msg, msg2, "y"), std::runtime_error);
  EXPECT_THROW(cras::copyChannelData(msg, msg2, "z"), std::runtime_error);
  mod.resize(2);
  try
  {
    cras::copyChannelData(msg, msg2, "x");
    cras::copyChannelData(msg, msg2, "y");
    cras::copyChannelData(msg, msg2, "z");
  }
  catch (const std::runtime_error& e)
  {
    GTEST_NONFATAL_FAILURE_(e.what());
  }
}

TEST(Cloud, HasGetField)  // NOLINT
{
  sensor_msgs::PointCloud2 msgOrig;
  sensor_msgs::PointCloud2Modifier mod(msgOrig);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(1);
  const auto& msg = msgOrig;
  
  sensor_msgs::PointCloud2 msg2;
  sensor_msgs::PointCloud2Modifier mod2(msg2);
  mod2.setPointCloud2FieldsByString(2, "xyz", "rgba");
  mod2.resize(1);

  EXPECT_TRUE(cras::hasField(msg, "x"));
  EXPECT_TRUE(cras::hasField(msg2, "x"));
  EXPECT_TRUE(cras::hasField(msg, "y"));
  EXPECT_TRUE(cras::hasField(msg2, "y"));
  EXPECT_TRUE(cras::hasField(msg, "z"));
  EXPECT_TRUE(cras::hasField(msg2, "z"));
  EXPECT_FALSE(cras::hasField(msg, "rgba"));
  EXPECT_TRUE(cras::hasField(msg2, "rgba"));
  EXPECT_FALSE(cras::hasField(msg, "test"));
  EXPECT_FALSE(cras::hasField(msg2, "test"));
  
  EXPECT_EQ("x", cras::getField(msg, "x").name);
  EXPECT_EQ(1, cras::getField(msg, "x").count);
  EXPECT_EQ(sensor_msgs::PointField::FLOAT32, cras::getField(msg, "x").datatype);
  EXPECT_EQ(0, cras::getField(msg, "x").offset);
  
  EXPECT_EQ("y", cras::getField(msg, "y").name);
  EXPECT_EQ(1, cras::getField(msg, "y").count);
  EXPECT_EQ(sensor_msgs::PointField::FLOAT32, cras::getField(msg, "y").datatype);
  EXPECT_EQ(4, cras::getField(msg, "y").offset);
  
  EXPECT_EQ("z", cras::getField(msg, "z").name);
  EXPECT_EQ(1, cras::getField(msg, "z").count);
  EXPECT_EQ(sensor_msgs::PointField::FLOAT32, cras::getField(msg, "z").datatype);
  EXPECT_EQ(8, cras::getField(msg, "z").offset);
  
  EXPECT_EQ(cras::getField(msg2, "x"), cras::getField(msg, "x"));
  EXPECT_EQ(cras::getField(msg2, "y"), cras::getField(msg, "y"));
  EXPECT_EQ(cras::getField(msg2, "z"), cras::getField(msg, "z"));

  EXPECT_EQ("rgba", cras::getField(msg2, "rgba").name);
  EXPECT_EQ(1, cras::getField(msg2, "rgba").count);
  EXPECT_EQ(sensor_msgs::PointField::FLOAT32, cras::getField(msg2, "rgba").datatype);
  EXPECT_EQ(16, cras::getField(msg2, "rgba").offset);

  EXPECT_THROW(cras::getField(msg, "rgba"), std::runtime_error);
  EXPECT_THROW(cras::getField(msg, "test"), std::runtime_error);
  EXPECT_THROW(cras::getField(msg2, "test"), std::runtime_error);
  
  // Test that getField returns a reference to the field on non-const messages.
  cras::getField(msg2, "x").offset = 42;
  EXPECT_EQ(42, msg2.fields[0].offset);
}

TEST(Cloud, SizeOfPointField)  // NOLINT
{
  using F = sensor_msgs::PointField;
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(8,
    "a", 1, F::FLOAT64, "b", 1, F::FLOAT32,
    "c", 1, F::INT8, "d", 1, F::INT16, "e", 1, F::INT32,
    "f", 1, F::UINT8, "g", 1, F::UINT16, "h", 1, F::UINT32);
  mod.resize(1);
  
  EXPECT_EQ(msg.fields[1].offset - msg.fields[0].offset, cras::sizeOfPointField(F::FLOAT64));
  EXPECT_EQ(msg.fields[2].offset - msg.fields[1].offset, cras::sizeOfPointField(F::FLOAT32));
  EXPECT_EQ(msg.fields[3].offset - msg.fields[2].offset, cras::sizeOfPointField(F::INT8));
  EXPECT_EQ(msg.fields[4].offset - msg.fields[3].offset, cras::sizeOfPointField(F::INT16));
  EXPECT_EQ(msg.fields[5].offset - msg.fields[4].offset, cras::sizeOfPointField(F::INT32));
  EXPECT_EQ(msg.fields[6].offset - msg.fields[5].offset, cras::sizeOfPointField(F::UINT8));
  EXPECT_EQ(msg.fields[7].offset - msg.fields[6].offset, cras::sizeOfPointField(F::UINT16));
  EXPECT_EQ(msg.point_step - msg.fields[7].offset, cras::sizeOfPointField(F::UINT32));
  
  EXPECT_EQ(msg.fields[1].offset - msg.fields[0].offset, cras::sizeOfPointField(msg.fields[0]));
  EXPECT_EQ(msg.fields[2].offset - msg.fields[1].offset, cras::sizeOfPointField(msg.fields[1]));
  EXPECT_EQ(msg.fields[3].offset - msg.fields[2].offset, cras::sizeOfPointField(msg.fields[2]));
  EXPECT_EQ(msg.fields[4].offset - msg.fields[3].offset, cras::sizeOfPointField(msg.fields[3]));
  EXPECT_EQ(msg.fields[5].offset - msg.fields[4].offset, cras::sizeOfPointField(msg.fields[4]));
  EXPECT_EQ(msg.fields[6].offset - msg.fields[5].offset, cras::sizeOfPointField(msg.fields[5]));
  EXPECT_EQ(msg.fields[7].offset - msg.fields[6].offset, cras::sizeOfPointField(msg.fields[6]));
  EXPECT_EQ(msg.point_step - msg.fields[7].offset, cras::sizeOfPointField(msg.fields[7]));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}