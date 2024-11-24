// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for tf2_utils.h
 * \author Martin Pecka
 */

// Test TfMessageFilter is mostly taken from
// https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/test/message_filter_test.cpp
// on commit 589caf083cae9d8fae7effdb910454b4681b9ec1 . Slight modifications were done to increase reliability of
// the test (circumventing real topic pub/sub) and to utilize the customized class TfMessageFilter.
// For that part, the following copyright notice is valid:

/*
 * Copyright (c) 2014, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gtest/gtest.h"

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <cras_cpp_common/tf2_utils/message_filter.hpp>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ros/callback_queue.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * \brief Test conversion of tf2 quaternion to roll, pitch and yaw.
 */
TEST(TF2Utils, TF2GetRPY)  // NOLINT
{
  tf2::Matrix3x3 m;
  tf2::Quaternion q;
  double roll, pitch, yaw;

  m.setRPY(0, 0, 0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_EQ(0, roll); EXPECT_EQ(0, pitch); EXPECT_EQ(0, yaw);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(M_PI_2, 0, 0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(M_PI_2, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0, M_PI_2, 0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(M_PI_2, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0, 0, M_PI_2); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(M_PI_2, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0.1, 0.1, 0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0.5, 0.5, 0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(1.0, 1.0, 0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0, 0.1, 0.1); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0, 0.5, 0.5); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0, 1.0, 1.0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0.1, 0, 0.1); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0.5, 0, 0.5); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(1.0, 0, 1.0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0.1, 0.1, 0.1); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(0.5, 0.5, 0.5); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));

  m.setRPY(1.0, 1.0, 1.0); m.getRotation(q); cras::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(q)); EXPECT_EQ(pitch, cras::getPitch(q)); EXPECT_EQ(yaw, cras::getYaw(q));
}

/**
 * \brief Test conversion of geometry_msgs quaternion to roll, pitch and yaw.
 */
TEST(TF2Utils, GeometryGetRPY)  // NOLINT
{
  tf2::Matrix3x3 m;
  tf2::Quaternion q;
  geometry_msgs::Quaternion gq;
  double roll, pitch, yaw;

  m.setRPY(0, 0, 0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_EQ(0, roll); EXPECT_EQ(0, pitch); EXPECT_EQ(0, yaw);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(M_PI_2, 0, 0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(M_PI_2, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0, M_PI_2, 0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(M_PI_2, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0, 0, M_PI_2); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(M_PI_2, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0.1, 0.1, 0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0.5, 0.5, 0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(1.0, 1.0, 0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0, 0.1, 0.1); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0, 0.5, 0.5); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0, 1.0, 1.0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0.1, 0, 0.1); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0.5, 0, 0.5); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(1.0, 0, 1.0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0.1, 0.1, 0.1); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(0.5, 0.5, 0.5); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));

  m.setRPY(1.0, 1.0, 1.0); m.getRotation(q); gq = tf2::toMsg(q); cras::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, cras::getRoll(gq)); EXPECT_EQ(pitch, cras::getPitch(gq)); EXPECT_EQ(yaw, cras::getYaw(gq));
}

void fillBuffer(tf2::BufferCore& buf)
{
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time(10, 0);
  tf.header.frame_id = "a";
  tf.child_frame_id = "b";
  tf.transform.rotation.w = 1;
  buf.setTransform(tf, "test");

  tf.header.frame_id = "a";
  tf.child_frame_id = "c";
  tf.transform.rotation.w = 1;
  buf.setTransform(tf, "test");
}

void fillBuffer(cras::InterruptibleTFBuffer& buf)
{
  fillBuffer(buf.getRawBuffer());
}

void testInterruptibleBuffer(
  cras::InterruptibleTFBuffer& buf, cras::InterruptibleTFBuffer& buf2, cras::InterruptibleTFBuffer& buf3,
  cras::InterruptibleTFBuffer& buf4, cras::InterruptibleTFBuffer& buf5, cras::InterruptibleTFBuffer& buf6,
  std::shared_ptr<cras::InterruptibleTFBuffer>& buf7)
{
  ros::Time::setNow({10, 0});

  std::mutex taskMutex;
  std::vector<bool> hasRun;
  auto addTask = [&]() {std::lock_guard<std::mutex> l(taskMutex); hasRun.push_back(false);};
  auto endTask = [&](const size_t i) {std::lock_guard<std::mutex> l(taskMutex); hasRun[i] = true;};

  // Test with 0 timeout, should not block

  addTask(); std::thread([&](){EXPECT_TRUE(buf.canTransform("b", "a", {10, 0}, {0, 0})); endTask(0);}).detach();
  addTask(); std::thread([&](){EXPECT_FALSE(buf.canTransform("b", "a", {11, 0}, {0, 0})); endTask(1);}).detach();
  addTask(); std::thread([&](){EXPECT_FALSE(buf.canTransform("d", "a", {10, 0}, {0, 0})); endTask(2);}).detach();
  addTask(); std::thread([&](){EXPECT_TRUE(buf.canTransform("c", "b", {10, 0}, {0, 0})); endTask(3);}).detach();

  addTask(); std::thread([&](){
    try
    {
      auto res = buf.lookupTransform("b", "a", {10, 0}, {0, 0});
      EXPECT_EQ(ros::Time(10, 0), res.header.stamp);
      EXPECT_EQ("b", res.header.frame_id);
      EXPECT_EQ("a", res.child_frame_id);
      EXPECT_EQ(1, res.transform.rotation.w);
    }
    catch (const std::runtime_error& e)
    {
      GTEST_NONFATAL_FAILURE_(e.what());
    }
  endTask(4);}).detach();

  addTask(); std::thread([&](){
    try
    {
      auto res = buf.lookupTransform("c", "b", {10, 0}, {0, 0});
      EXPECT_EQ(ros::Time(10, 0), res.header.stamp);
      EXPECT_EQ("c", res.header.frame_id);
      EXPECT_EQ("b", res.child_frame_id);
      EXPECT_EQ(1, res.transform.rotation.w);
    }
    catch (const std::runtime_error& e)
    {
      GTEST_NONFATAL_FAILURE_(e.what());
    }
  endTask(5);}).detach();

  addTask(); std::thread([&](){
    EXPECT_THROW(buf.lookupTransform("b", "a", {11, 0}, {0, 0}), tf2::ExtrapolationException);
  endTask(6);}).detach();
  addTask(); std::thread([&](){
    EXPECT_THROW(buf.lookupTransform("d", "a", {10, 0}, {0, 0}), tf2::LookupException);
  endTask(7);}).detach();

  // Test with nonzero timeout, should block if transform is not available

  addTask(); std::thread([&](){EXPECT_TRUE(buf.canTransform("b", "a", {10, 0}, {1, 0})); endTask(8);}).detach();
  addTask(); std::thread([&](){EXPECT_TRUE(buf.canTransform("c", "b", {10, 0}, {1, 0})); endTask(9);}).detach();

  // Test that lookupTransform() for known transforms with a timeout works.

  addTask(); std::thread([&](){
    try
    {
      auto res = buf.lookupTransform("b", "a", {10, 0}, {1, 0});
      EXPECT_EQ(ros::Time(10, 0), res.header.stamp);
      EXPECT_EQ("b", res.header.frame_id);
      EXPECT_EQ("a", res.child_frame_id);
      EXPECT_EQ(1, res.transform.rotation.w);
    }
    catch (const std::runtime_error& e)
    {
      GTEST_NONFATAL_FAILURE_(e.what());
    }
  endTask(10);}).detach();

  addTask(); std::thread([&](){
    try
    {
      auto res = buf.lookupTransform("c", "b", {10, 0}, {1, 0});
      EXPECT_EQ(ros::Time(10, 0), res.header.stamp);
      EXPECT_EQ("c", res.header.frame_id);
      EXPECT_EQ("b", res.child_frame_id);
      EXPECT_EQ(1, res.transform.rotation.w);
    }
    catch (const std::runtime_error& e)
    {
      GTEST_NONFATAL_FAILURE_(e.what());
    }
  endTask(11);}).detach();

  // There is no data for time 11, so the buffer waits; but we do not advance time. Normally, the canTransform() call
  // should wait infinitely (until rostime reaches 11, which it never will), but if we request the buffer to stop, the
  // canTransform() call should return false almost immediately.

  bool started = false;
  bool executed = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf2.canTransform("b", "a", {11, 0}, {1, 0}, &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  }).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.1).sleep();
  EXPECT_FALSE(executed);

  buf2.requestStop();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test that canTransform() for an unknown transform blocks and can be interrupted.

  started = false;
  executed = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf3.canTransform("d", "a", {10, 0}, {1, 0}, &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.1).sleep();
  EXPECT_FALSE(executed);

  buf3.requestStop();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test that lookupTransform() for an existing transform with a timeout works.

  started = false;
  executed = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    try
    {
      auto res = buf4.lookupTransform("c", {10, 0}, "b", {10, 0}, "a", {1, 0});
      EXPECT_EQ(ros::Time(10, 0), res.header.stamp);
      EXPECT_EQ("c", res.header.frame_id);
      EXPECT_EQ("b", res.child_frame_id);
      EXPECT_EQ(1, res.transform.rotation.w);
    }
    catch (const std::runtime_error& e)
    {
      GTEST_NONFATAL_FAILURE_(e.what());
    }
    executed = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test multiple simultaneous requests. First two should block and the third should succeed right away.

  started = false;
  executed = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf5.canTransform("d", "a", {10, 0}, {1, 0}, &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  }).detach();

  auto started2 = false;
  auto executed2 = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started2 = true;
    std::string errstr;
    EXPECT_FALSE(buf5.canTransform("e", "a", {10, 0}, {1, 0}, &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed2 = true;
  }).detach();

  auto started3 = false;
  auto executed3 = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started3 = true;
    std::string errstr;
    EXPECT_TRUE(buf5.canTransform("b", "a", {10, 0}, {1, 0}, &errstr));
    EXPECT_EQ("", errstr);
    executed3 = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while ((!started || !started2 || !started3) && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_TRUE(started2);
  EXPECT_TRUE(started3);
  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed3 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed3);

  ros::WallDuration(0.1).sleep();
  EXPECT_FALSE(executed);
  EXPECT_FALSE(executed2);

  buf5.requestStop();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while ((!executed || !executed2) && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);
  EXPECT_TRUE(executed2);

  // Test that normal timeout works when time moves on.

  started = false;
  executed = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf6.canTransform("b", "a", {11, 0}, {1, 0}, &errstr));
    EXPECT_EQ("Lookup would require extrapolation at time 11.000000000, but only time 10.000000000 is in the buffer, "
              "when looking up transform from frame [a] to frame [b] canTransform returned after 1 s, timeout was 1 s.",
              errstr);
    executed = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::Time::setNow({11, 1000});

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Test that destroying the buffer would cancel an ongoing request.

  started = false;
  executed = false;
  std::thread([&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf7->canTransform("b", {12, 0}, "a", {12, 0}, "c", {1, 0}, &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.1).sleep();

  EXPECT_FALSE(executed);

  started2 = false;
  executed2 = false;
  std::thread([&]()
  {
    started2 = true;
    buf7.reset();
    executed2 = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!started2 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started2);

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed2 && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed2);

  EXPECT_EQ(nullptr, buf7);

  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(executed);

  // Wait for finish of all the simpler tasks from the beginning (at most 1 second).

  std::vector<bool> shouldRun(hasRun.size(), true);
  end = ros::WallTime::now() + ros::WallDuration(1.0);
  while (shouldRun != hasRun && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_EQ(shouldRun, hasRun);
}

/**
 * \brief Test that interruptible buffer is working and is interruptible.
 */
TEST(TF2Utils, InterruptibleBufferDirect)  // NOLINT
{
  ros::Time::setNow({10, 0});

  cras::InterruptibleTFBuffer buf, buf2, buf3, buf4, buf5, buf6;
  auto buf7 = std::make_shared<cras::InterruptibleTFBuffer>();
  fillBuffer(buf); fillBuffer(buf2); fillBuffer(buf3); fillBuffer(buf4); fillBuffer(buf5); fillBuffer(buf6);
  fillBuffer(*buf7);

  SCOPED_TRACE("Error was in direct buffer test");
  testInterruptibleBuffer(buf, buf2, buf3, buf4, buf5, buf6, buf7);
}

/**
 * \brief Test that wrapped interruptible buffer is working and is interruptible.
 */
TEST(TF2Utils, InterruptibleBufferWrapped)  // NOLINT
{
  ros::Time::setNow({10, 0});

  using B = tf2_ros::Buffer;
  std::shared_ptr<tf2_ros::Buffer> b(new B), b2(new B), b3(new B), b4(new B), b5(new B), b6(new B), b7(new B);
  cras::InterruptibleTFBuffer buf(b), buf2(b2), buf3(b3), buf4(b4), buf5(b5), buf6(b6);
  auto buf7 = std::make_shared<cras::InterruptibleTFBuffer>(b7);

  fillBuffer(*b); fillBuffer(*b2); fillBuffer(*b3); fillBuffer(*b4); fillBuffer(*b5); fillBuffer(*b6); fillBuffer(*b7);

  SCOPED_TRACE("Error was in wrapped buffer test");
  testInterruptibleBuffer(buf, buf2, buf3, buf4, buf5, buf6, buf7);
}

/**
 * \brief Test setting polling duration on an interruptible buffer.
 */
TEST(TF2Utils, SetPollingParams)  // NOLINT
{
  ros::Time::setNow({10, 0});

  cras::InterruptibleTFBuffer buf;

  fillBuffer(buf);

  EXPECT_FALSE(buf.setMinPollingDuration({-1, 0}));
  EXPECT_TRUE(buf.setMinPollingDuration({1, 0}));

  bool started = false;
  bool executed = false;
  std::thread([&](){
    started = true;
    std::string errstr;
    buf.canTransform("d", "a", {10, 0}, ros::Duration(0.1), &errstr);
    EXPECT_EQ("canTransform: target_frame d does not exist. canTransform returned after 1.1 s, timeout was 0.1 s.",
              errstr);
    executed = true;
  }).detach();

  auto end = ros::WallTime::now() + ros::WallDuration(1.0);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.1).sleep();
  EXPECT_FALSE(executed);

  ros::Time::setNow(ros::Time(10.2));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_FALSE(executed);

  ros::Time::setNow(ros::Time(10.9));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_FALSE(executed);

  ros::Time::setNow(ros::Time(11.1));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_TRUE(executed);

  // Test setting min polling scale.

  EXPECT_TRUE(buf.setMinPollingDuration({0, 1000000}));
  EXPECT_FALSE(buf.setCanTransformPollingScale(-1.0));
  EXPECT_FALSE(buf.setCanTransformPollingScale(-0.1));
  EXPECT_FALSE(buf.setCanTransformPollingScale(1.1));
  EXPECT_TRUE(buf.setCanTransformPollingScale(2.0 / 3.0));

  ros::Time::setNow({10, 0});

  // Timeout is 1.0, polling scale is 2/3, so canTransform() will first wait until time > 10.67

  started = false;
  executed = false;
  std::thread([&](){
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf.canTransform("d", "a", {10, 0}, {1, 0}, &errstr));
    EXPECT_EQ("canTransform: target_frame d does not exist. canTransform returned after 1.6 s, timeout was 1 s.",
              errstr);
    executed = true;
  }).detach();

  end = ros::WallTime::now() + ros::WallDuration(1.0);
  while (!started && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();

  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  ros::WallDuration(0.1).sleep();
  EXPECT_FALSE(executed);

  ros::Time::setNow(ros::Time(10.2));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_FALSE(executed);

  // Time will be set > 10.67, so second round of sleep is started. 10.9 + 0.67 = 11.57, that is the new end time.
  ros::Time::setNow(ros::Time(10.9));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_FALSE(executed);

  // < 11.57, still sleeping
  ros::Time::setNow(ros::Time(11.5));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_FALSE(executed);

  // > 11.57, sleep ends
  ros::Time::setNow(ros::Time(11.6));
  end = ros::WallTime::now() + ros::WallDuration(0.1);
  while (!executed && ros::WallTime::now() < end)
    ros::WallDuration(0.01).sleep();
  EXPECT_TRUE(executed);
}

bool filter_callback_fired = false;
void filter_callback(const geometry_msgs::PointStamped& msg)
{
  filter_callback_fired = true;
}

template<class T>
class TestInput : public message_filters::SimpleFilter<T>
{
public:
  void add(const typename T::ConstPtr& msg)
  {
    // Pass a complete MessageEvent to avoid calling ros::Time::now() to determine the missing timestamp
    this->signalMessage(ros::MessageEvent<T const>(msg, msg->header.stamp));
  }
};

TEST(TF2Utils, TfMessageFilter)
{
  ros::CallbackQueue cb;
  TestInput<geometry_msgs::PointStamped> sub;
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  tf2_ros::Buffer buffer;
  cras::TfMessageFilter<geometry_msgs::PointStamped> filter(log, buffer, "map", 10, &cb);
  filter.connectInput(sub);
  filter.registerCallback(&filter_callback);
  // Register multiple target frames
  std::vector<std::string> frames;
  frames.push_back("odom");
  frames.push_back("map");
  filter.setTargetFrames(frames);
  // Set a non-zero time tolerance
  filter.setTolerance(ros::Duration(1, 0));

  // Publish static transforms so the frame transformations will always be valid
  geometry_msgs::TransformStamped map_to_odom;
  map_to_odom.header.stamp = ros::Time(0, 0);
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 0.0;
  map_to_odom.transform.translation.y = 0.0;
  map_to_odom.transform.translation.z = 0.0;
  map_to_odom.transform.rotation.x = 0.0;
  map_to_odom.transform.rotation.y = 0.0;
  map_to_odom.transform.rotation.z = 0.0;
  map_to_odom.transform.rotation.w = 1.0;
  buffer.setTransform(map_to_odom, "test", true);

  geometry_msgs::TransformStamped odom_to_base;
  odom_to_base.header.stamp = ros::Time(0, 0);
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base";
  odom_to_base.transform.translation.x = 0.0;
  odom_to_base.transform.translation.y = 0.0;
  odom_to_base.transform.translation.z = 0.0;
  odom_to_base.transform.rotation.x = 0.0;
  odom_to_base.transform.rotation.y = 0.0;
  odom_to_base.transform.rotation.z = 0.0;
  odom_to_base.transform.rotation.w = 1.0;
  buffer.setTransform(odom_to_base, "test", true);

  EXPECT_TRUE(cb.empty());

  // Publish a Point message in the "base" frame
  geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped);
  point->header.stamp = ros::Time(0, 0);
  point->header.frame_id = "base";
  sub.add(point);

  EXPECT_FALSE(cb.empty());
  cb.callAvailable();
  EXPECT_TRUE(cb.empty());

  // The filter callback should have been fired because all required transforms are available
  EXPECT_TRUE(filter_callback_fired);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_tf2_utils");
  ros::start();
  return RUN_ALL_TESTS();
}
