/**
 * \file
 * \brief Test for shape_shifter.h .
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <utility>
#include <vector>

#include <ros/serialization.h>

#include <cras_topic_tools/shape_shifter.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

TEST(ShapeShifter, MsgToShapeShifter)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  ASSERT_NO_THROW(shifter.instantiate<geometry_msgs::PointStamped>());
  const auto msg2 = *shifter.instantiate<geometry_msgs::PointStamped>();

  topic_tools::ShapeShifter shifter2;
  cras::msgToShapeShifter(msg2, shifter2);

  ASSERT_NO_THROW(shifter2.instantiate<geometry_msgs::PointStamped>());
  const auto msg3 = *shifter2.instantiate<geometry_msgs::PointStamped>();

  ASSERT_EQ(cras::getBufferLength(shifter), cras::getBufferLength(shifter2));

  // Compare shifter buffers
  for (size_t i = 0; i < cras::getBufferLength(shifter); ++i)
    EXPECT_EQ(cras::getBuffer(shifter2)[i], cras::getBuffer(shifter)[i]);

  // Compare messages
  EXPECT_EQ(msg, msg2);
  EXPECT_EQ(msg, msg3);
  EXPECT_EQ(msg2, msg3);
}

TEST(ShapeShifter, GetBuffer)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  const auto length = ros::serialization::serializationLength(msg);
  std::vector<uint8_t> buf;
  buf.resize(length);
  ros::serialization::OStream ostream(buf.data(), length);
  ros::serialization::serialize(ostream, msg);

  // Compare getBuffer() contents with the contents of the byte buffer
  EXPECT_EQ(length, cras::getBufferLength(shifter));
  for (size_t i = 0; i < length; ++i)
    EXPECT_EQ(buf[i], cras::getBuffer(shifter)[i]);

  // Just verify that we have loaded the buffer into the shape shifter correctly by round-tripping the message
  EXPECT_EQ(msg, *shifter.instantiate<geometry_msgs::PointStamped>());
}

TEST(ShapeShifter, GetHeaderPointStamped)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  EXPECT_TRUE(cras::hasHeader(shifter));

  auto header = cras::getHeader(shifter);
  ASSERT_TRUE(header.has_value());
  EXPECT_EQ(msg.header, header.value());
}

TEST(ShapeShifter, GetHeaderPoint)  // NOLINT
{
  // Create a message
  geometry_msgs::Point msg;
  msg.x = 1;
  msg.y = 2;
  msg.z = 3;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  EXPECT_FALSE(cras::hasHeader(shifter));

  // Do not try to run this as it could segfault
  // auto header = cras::getHeader(shifter);
  // ASSERT_TRUE(header.has_value());
  // EXPECT_EQ(msg, header.value());
}

TEST(ShapeShifter, GetHeaderFromHeader)  // NOLINT
{
  // Create a message
  std_msgs::Header msg;
  msg.stamp.sec = 1;
  msg.stamp.nsec = 2;
  msg.frame_id = "test";

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  EXPECT_FALSE(cras::hasHeader(shifter));

  auto header = cras::getHeader(shifter);
  ASSERT_TRUE(header.has_value());
  EXPECT_EQ(msg, header.value());
}

TEST(ShapeShifter, GetHeaderFromBool)  // NOLINT
{
  // Create a message
  std_msgs::Bool msg;
  msg.data = true;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  EXPECT_FALSE(cras::hasHeader(shifter));

  auto header = cras::getHeader(shifter);
  EXPECT_FALSE(header.has_value());
}

TEST(ShapeShifter, GetHeaderFromLongString)  // NOLINT
{
  // Create a message
  std_msgs::String msg;
  msg.data = "123456789012345678901234567890";

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  EXPECT_FALSE(cras::hasHeader(shifter));

  auto header = cras::getHeader(shifter);
  EXPECT_FALSE(header.has_value());
}

TEST(ShapeShifter, GetHeaderFromFakeString)  // NOLINT
{
  // Create a message
  std_msgs::String msg;
  msg.data = std::string(22, '\x00');

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  EXPECT_FALSE(cras::hasHeader(shifter));

  auto header = cras::getHeader(shifter);
  ASSERT_TRUE(header.has_value());  // The string decodes as a Header with seq == length of msg.data
  std_msgs::Header fakeHeader;
  fakeHeader.seq = msg.data.size();
  EXPECT_EQ(fakeHeader, header.value());
}

TEST(ShapeShifter, SetHeaderSameLength)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;
  const auto origMsg = msg;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  auto newHeader = msg.header;
  newHeader.frame_id = "abcd";
  EXPECT_TRUE(cras::setHeader(shifter, newHeader));

  const auto decodedHeader = cras::getHeader(shifter);
  ASSERT_TRUE(decodedHeader.has_value());
  EXPECT_EQ(newHeader, decodedHeader.value());

  const auto newMsg = shifter.instantiate<geometry_msgs::PointStamped>();
  EXPECT_EQ(newHeader, newMsg->header);
  EXPECT_EQ(origMsg.point, newMsg->point);
}

TEST(ShapeShifter, SetHeaderShorter)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;
  const auto origMsg = msg;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  auto newHeader = msg.header;
  newHeader.frame_id = "ab";
  EXPECT_TRUE(cras::setHeader(shifter, newHeader));

  const auto decodedHeader = cras::getHeader(shifter);
  ASSERT_TRUE(decodedHeader.has_value());
  EXPECT_EQ(newHeader, decodedHeader.value());

  const auto newMsg = shifter.instantiate<geometry_msgs::PointStamped>();
  EXPECT_EQ(newHeader, newMsg->header);
  EXPECT_EQ(origMsg.point, newMsg->point);
}

TEST(ShapeShifter, SetHeaderLonger)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;
  const auto origMsg = msg;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  auto newHeader = msg.header;
  newHeader.frame_id = "abcdefgh";
  EXPECT_TRUE(cras::setHeader(shifter, newHeader));

  const auto decodedHeader = cras::getHeader(shifter);
  ASSERT_TRUE(decodedHeader.has_value());
  EXPECT_EQ(newHeader, decodedHeader.value());

  const auto newMsg = shifter.instantiate<geometry_msgs::PointStamped>();
  EXPECT_EQ(newHeader, newMsg->header);
  EXPECT_EQ(origMsg.point, newMsg->point);
}

TEST(ShapeShifter, CopyShapeShifter)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;
  const auto origMsg = msg;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  topic_tools::ShapeShifter shifter2;
  // If we used shifter2 = shifter, we'd get a segfault in Melodic when this function ends
  cras::copyShapeShifter(shifter, shifter2);

  EXPECT_EQ(*shifter.instantiate<geometry_msgs::PointStamped>(), *shifter2.instantiate<geometry_msgs::PointStamped>());
  EXPECT_NE(cras::getBuffer(shifter), cras::getBuffer(shifter2));
}

TEST(ShapeShifter, CopyCrasShapeShifter)  // NOLINT
{
  // Create a message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  msg.header.frame_id = "test";
  msg.point.x = 1;
  msg.point.y = 2;
  msg.point.z = 3;
  const auto origMsg = msg;

  // Load the message into the shape shifter object
  topic_tools::ShapeShifter shifter;
  cras::msgToShapeShifter(msg, shifter);

  cras::ShapeShifter shifter2;
  // With normal ShapeShifter, we'd get a segfault in Melodic when this function ends
  shifter2 = shifter;

  EXPECT_EQ(*shifter.instantiate<geometry_msgs::PointStamped>(), *shifter2.instantiate<geometry_msgs::PointStamped>());
  EXPECT_NE(cras::getBuffer(shifter), cras::getBuffer(shifter2));

  cras::ShapeShifter shifter3;
  shifter3 = std::move(shifter2);

  EXPECT_EQ(*shifter.instantiate<geometry_msgs::PointStamped>(), *shifter3.instantiate<geometry_msgs::PointStamped>());
  EXPECT_NE(cras::getBuffer(shifter), cras::getBuffer(shifter3));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
