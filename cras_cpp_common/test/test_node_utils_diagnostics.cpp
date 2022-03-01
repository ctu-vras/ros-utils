/**
 * \file
 * \brief Unit test for node_handle_with_diagnostics.h.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"
#include <cras_cpp_common/node_utils.hpp>

#include "subscription_callbacks.inc"

using namespace cras;

TEST(NodeUtils, AdvertiseDiagnosedNoHeader)  // NOLINT
{
  cras::NodeHandle nh;
  cras::NodeHandle pnh("~");
  cras::NodeHandle tnh("test");
  cras::NodeHandle rnh("", {{"a", "d"}});
  
  pnh.deleteParam("/");
  pnh.setParam("topic/rate/min", 5.0);
  pnh.setParam("topic/rate/max", 5.0);
  pnh.setParam("topic/rate/tolerance", 0.1);
  pnh.setParam("topic/rate/window_size", 10);
  pnh.setParam("topic/delay/min", 2.0);
  pnh.setParam("topic/delay/max", 3.0);
  pnh.setParam("a/rate/min", 20.0);
  pnh.setParam("a/rate/max", 20.0);
  pnh.setParam("a/rate/tolerance", 0.1);
  pnh.setParam("a/rate/window_size", 10);
  pnh.setParam("a/delay/min", -2.0);
  pnh.setParam("a/delay/max", -1.0);
  pnh.setParam("b/rate/min", 20.0);
  pnh.setParam("b/rate/max", 20.0);
  pnh.setParam("b/rate/tolerance", 0.1);
  pnh.setParam("b/rate/window_size", 10);
  pnh.setParam("b/delay/min", -2.0);
  pnh.setParam("b/delay/max", -1.0);
  pnh.setParam("/topic/rate/min", 20.0);
  pnh.setParam("/topic/rate/max", 20.0);
  pnh.setParam("/topic/rate/tolerance", 0.1);
  pnh.setParam("/topic/rate/window_size", 10);
  pnh.setParam("/topic/delay/min", 2.0);
  pnh.setParam("/topic/delay/max", 3.0);
  pnh.setParam("/test/c/rate/min", 20.0);
  pnh.setParam("/test/c/rate/max", 20.0);
  pnh.setParam("/test/c/rate/tolerance", 0.1);
  pnh.setParam("/test/c/rate/window_size", 10);
  pnh.setParam("/test/c/delay/min", -2.0);
  pnh.setParam("/test/c/delay/max", -1.0);
  pnh.setParam("/test/topic/rate/min", 5.0);
  pnh.setParam("/test/topic/rate/max", 5.0);
  pnh.setParam("/test/topic/rate/tolerance", 0.1);
  pnh.setParam("/test/topic/rate/window_size", 10);
  pnh.setParam("/test/topic/delay/min", 2.0);
  pnh.setParam("/test/topic/delay/max", 3.0);
  pnh.setParam("/a/rate/min", 5.0);
  pnh.setParam("/a/rate/max", 5.0);
  pnh.setParam("/a/rate/tolerance", 0.1);
  pnh.setParam("/a/rate/window_size", 10);
  pnh.setParam("/a/delay/min", 2.0);
  pnh.setParam("/a/delay/max", 3.0);
  
  ros::Time::setNow({10, 0});
  diagnostic_msgs::DiagnosticArrayConstPtr msg;
  size_t numDiagCalled {0};
  auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10,
    [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});

  size_t numCalled {0};
  
  auto sub1 = nh.subscribe<std_msgs::Header>("a", 1000, [&numCalled](const std_msgs::HeaderConstPtr&){++numCalled;});
  auto sub2 = pnh.subscribe<std_msgs::Header>("b", 1000, [&numCalled](const std_msgs::HeaderConstPtr&){++numCalled;});
  auto sub3 = tnh.subscribe<std_msgs::Header>("c", 1000, [&numCalled](const std_msgs::HeaderConstPtr&){++numCalled;});
  auto sub4 = nh.subscribe<std_msgs::Header>("d", 1000, [&numCalled](const std_msgs::HeaderConstPtr&){++numCalled;});
  
  diagnostic_updater::Updater updater1(nh);  // NOLINT
  auto pub1 = nh.advertiseDiagnosed<std_msgs::Header>(updater1, "a", 10);

  diagnostic_updater::Updater updater2(nh);  // NOLINT
  auto pub2 = nh.advertiseDiagnosed<std_msgs::Header>(updater2, "topic", "a", 10);

  diagnostic_updater::Updater updater3(nh);  // NOLINT
  auto pub3 = nh.advertiseDiagnosed<std_msgs::Header>(updater3, {10.0, 10.0, 0.1, 5}, "topic", "a", 10);

  diagnostic_updater::Updater updater4(nh);  // NOLINT
  auto pub4 = nh.advertiseDiagnosed<std_msgs::Header>(updater4, "~topic", "a", 10);

  diagnostic_updater::Updater updater5(nh);  // NOLINT
  auto pub5 = nh.advertiseDiagnosed<std_msgs::Header>(updater5, {10.0, 10.0, 0.1, 5}, "~topic", "a", 10);

  diagnostic_updater::Updater updater6(nh);  // NOLINT
  auto pub6 = pnh.advertiseDiagnosed<std_msgs::Header>(updater6, "b", 10);

  diagnostic_updater::Updater updater7(nh);  // NOLINT
  auto pub7 = pnh.advertiseDiagnosed<std_msgs::Header>(updater7, "topic", "b", 10);

  diagnostic_updater::Updater updater8(nh);  // NOLINT
  auto pub8 = pnh.advertiseDiagnosed<std_msgs::Header>(updater8, {10.0, 10.0, 0.1, 5}, "topic", "b", 10);

  diagnostic_updater::Updater updater9(nh);  // NOLINT
  auto pub9 = pnh.advertiseDiagnosed<std_msgs::Header>(updater9, "~topic", "b", 10);

  diagnostic_updater::Updater updater10(nh);  // NOLINT
  auto pub10 = pnh.advertiseDiagnosed<std_msgs::Header>(updater10, {10.0, 10.0, 0.1, 5}, "~topic", "b", 10);

  diagnostic_updater::Updater updater11(nh);  // NOLINT
  auto pub11 = tnh.advertiseDiagnosed<std_msgs::Header>(updater11, "c", 10);

  diagnostic_updater::Updater updater12(nh);  // NOLINT
  auto pub12 = tnh.advertiseDiagnosed<std_msgs::Header>(updater12, "topic", "c", 10);

  diagnostic_updater::Updater updater13(nh);  // NOLINT
  auto pub13 = tnh.advertiseDiagnosed<std_msgs::Header>(updater13, {10.0, 10.0, 0.1, 5}, "topic", "c", 10);

  diagnostic_updater::Updater updater14(nh);  // NOLINT
  auto pub14 = tnh.advertiseDiagnosed<std_msgs::Header>(updater14, "~topic", "c", 10);

  diagnostic_updater::Updater updater15(nh);  // NOLINT
  auto pub15 = tnh.advertiseDiagnosed<std_msgs::Header>(updater15, {10.0, 10.0, 0.1, 5}, "~topic", "c", 10);
  
  ros::AdvertiseOptions opts16;
  opts16.init<std_msgs::Header>("a", 10);
  diagnostic_updater::Updater updater16(nh);  // NOLINT
  auto pub16 = nh.advertiseDiagnosed<std_msgs::Header>(updater16, opts16);
  
  ros::AdvertiseOptions opts17;
  opts17.init<std_msgs::Header>("a", 10);
  diagnostic_updater::Updater updater17(nh);  // NOLINT
  auto pub17 = nh.advertiseDiagnosed<std_msgs::Header>(updater17, "topic", opts17);
  
  ros::AdvertiseOptions opts18;
  opts18.init<std_msgs::Header>("a", 10);
  diagnostic_updater::Updater updater18(nh);  // NOLINT
  auto pub18 = nh.advertiseDiagnosed<std_msgs::Header>(updater18, {10.0, 10.0, 0.1, 5}, "topic", opts18);
  
  ros::AdvertiseOptions opts19;
  opts19.init<std_msgs::Header>("a", 10);
  diagnostic_updater::Updater updater19(nh);  // NOLINT
  auto pub19 = nh.advertiseDiagnosed<std_msgs::Header>(updater19, "~topic", opts19);
  
  ros::AdvertiseOptions opts20;
  opts20.init<std_msgs::Header>("a", 10);
  diagnostic_updater::Updater updater20(nh);  // NOLINT
  auto pub20 = nh.advertiseDiagnosed<std_msgs::Header>(updater20, {10.0, 10.0, 0.1, 5}, "~topic", opts20);

  // topic remapped to /d, but parameter read from ~a
  diagnostic_updater::Updater updater21(nh);  // NOLINT
  auto pub21 = rnh.advertiseDiagnosed<std_msgs::Header>(updater21, "a", 10);

  // topic remapped to /d, but parameter read from /a
  diagnostic_updater::Updater updater22(nh);  // NOLINT
  auto pub22 = rnh.advertiseDiagnosed<std_msgs::Header>(updater22, "/a", 10);

  diagnostic_updater::Updater updater23(nh);  // NOLINT
  auto pub23 = nh.advertiseDiagnosed<std_msgs::Header>(updater23, "/a", 10);

  diagnostic_updater::Updater updater24(nh);  // NOLINT
  auto pub24 = pnh.advertiseDiagnosed<std_msgs::Header>(updater24, "/a", 10);

  diagnostic_updater::Updater updater25(nh);  // NOLINT
  auto pub25 = tnh.advertiseDiagnosed<std_msgs::Header>(updater25, "/a", 10);
  
  ros::WallDuration(0.25).sleep();
  
  const size_t numCallbacks = 25;

  for (size_t i = 0; i < 100 && numDiagCalled < numCallbacks; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  std_msgs::Header pubMsg;
  
  for (size_t i = 0; i < 10; ++i)
  {
    ros::Time::setNow(ros::Time(10) + ros::Duration(0.1) * i);
    pubMsg.stamp = ros::Time(9.9) + ros::Duration(0.1) * i;
    pub1->publish(pubMsg);
    pub2->publish(pubMsg);
    pub3->publish(pubMsg);
    pub4->publish(pubMsg);
    pub5->publish(pubMsg);
    pub6->publish(pubMsg);
    pub7->publish(pubMsg);
    pub8->publish(pubMsg);
    pub9->publish(pubMsg);
    pub10->publish(pubMsg);
    pub11->publish(pubMsg);
    pub12->publish(pubMsg);
    pub13->publish(pubMsg);
    pub14->publish(pubMsg);
    pub15->publish(pubMsg);
    pub16->publish(pubMsg);
    pub17->publish(pubMsg);
    pub18->publish(pubMsg);
    pub19->publish(pubMsg);
    pub20->publish(pubMsg);
    pub21->publish(pubMsg);
    pub22->publish(pubMsg);
    pub23->publish(pubMsg);
    pub24->publish(pubMsg);
    pub25->publish(pubMsg);
  }
  
  ros::Time::setNow({11, 0});

  for (size_t i = 0; i < 100 && numCalled < numCallbacks * 10u; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(numCallbacks * 10u, numCalled);
  
  msg = nullptr;
  numDiagCalled = 0;
  updater1.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater2.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater3.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater4.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater5.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater6.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater7.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater8.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater9.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater10.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater11.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater12.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater13.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater14.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater15.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater16.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater17.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater18.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater19.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater20.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater21.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater22.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater23.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater24.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater25.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
}


TEST(NodeUtils, AdvertiseDiagnosedWithHeader)  // NOLINT
{
  cras::NodeHandle nh;
  cras::NodeHandle pnh("~");
  cras::NodeHandle tnh("test");
  cras::NodeHandle rnh("", {{"/a", "/d"}});
  
  pnh.deleteParam("/");
  pnh.setParam("topic/rate/min", 5.0);
  pnh.setParam("topic/rate/max", 5.0);
  pnh.setParam("topic/rate/tolerance", 0.1);
  pnh.setParam("topic/rate/window_size", 10);
  pnh.setParam("topic/delay/min", 2.0);
  pnh.setParam("topic/delay/max", 3.0);
  pnh.setParam("a/rate/min", 20.0);
  pnh.setParam("a/rate/max", 20.0);
  pnh.setParam("a/rate/tolerance", 0.1);
  pnh.setParam("a/rate/window_size", 10);
  pnh.setParam("a/delay/min", -2.0);
  pnh.setParam("a/delay/max", -1.0);
  pnh.setParam("b/rate/min", 20.0);
  pnh.setParam("b/rate/max", 20.0);
  pnh.setParam("b/rate/tolerance", 0.1);
  pnh.setParam("b/rate/window_size", 10);
  pnh.setParam("b/delay/min", -2.0);
  pnh.setParam("b/delay/max", -1.0);
  pnh.setParam("/topic/rate/min", 20.0);
  pnh.setParam("/topic/rate/max", 20.0);
  pnh.setParam("/topic/rate/tolerance", 0.1);
  pnh.setParam("/topic/rate/window_size", 10);
  pnh.setParam("/topic/delay/min", 2.0);
  pnh.setParam("/topic/delay/max", 3.0);
  pnh.setParam("/test/c/rate/min", 20.0);
  pnh.setParam("/test/c/rate/max", 20.0);
  pnh.setParam("/test/c/rate/tolerance", 0.1);
  pnh.setParam("/test/c/rate/window_size", 10);
  pnh.setParam("/test/c/delay/min", -2.0);
  pnh.setParam("/test/c/delay/max", -1.0);
  pnh.setParam("/test/topic/rate/min", 5.0);
  pnh.setParam("/test/topic/rate/max", 5.0);
  pnh.setParam("/test/topic/rate/tolerance", 0.1);
  pnh.setParam("/test/topic/rate/window_size", 10);
  pnh.setParam("/test/topic/delay/min", 2.0);
  pnh.setParam("/test/topic/delay/max", 3.0);
  pnh.setParam("/a/rate/min", 5.0);
  pnh.setParam("/a/rate/max", 5.0);
  pnh.setParam("/a/rate/tolerance", 0.1);
  pnh.setParam("/a/rate/window_size", 10);
  pnh.setParam("/a/delay/min", 2.0);
  pnh.setParam("/a/delay/max", 3.0);
  
  ros::Time::setNow({10, 0});
  diagnostic_msgs::DiagnosticArrayConstPtr msg;
  size_t numDiagCalled {0};
  auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10,
    [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});

  size_t numCalled {0};
  
  auto sub1 = nh.subscribe<diagnostic_msgs::DiagnosticArray>("a", 1000,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  auto sub2 = pnh.subscribe<diagnostic_msgs::DiagnosticArray>("b", 1000,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  auto sub3 = tnh.subscribe<diagnostic_msgs::DiagnosticArray>("c", 1000,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  auto sub4 = nh.subscribe<diagnostic_msgs::DiagnosticArray>("d", 1000,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  
  diagnostic_updater::Updater updater1(nh);  // NOLINT
  auto pub1 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater1, "a", 10);

  diagnostic_updater::Updater updater2(nh);  // NOLINT
  auto pub2 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater2, "topic", "a", 10);

  diagnostic_updater::Updater updater3(nh);  // NOLINT
  auto pub3 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater3, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "topic", "a", 10);

  diagnostic_updater::Updater updater4(nh);  // NOLINT
  auto pub4 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater4, "~topic", "a", 10);

  diagnostic_updater::Updater updater5(nh);  // NOLINT
  auto pub5 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater5, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "~topic", "a", 10);

  diagnostic_updater::Updater updater6(nh);  // NOLINT
  auto pub6 = pnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater6, "b", 10);

  diagnostic_updater::Updater updater7(nh);  // NOLINT
  auto pub7 = pnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater7, "topic", "b", 10);

  diagnostic_updater::Updater updater8(nh);  // NOLINT
  auto pub8 = pnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater8, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "topic", "b", 10);

  diagnostic_updater::Updater updater9(nh);  // NOLINT
  auto pub9 = pnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater9, "~topic", "b", 10);

  diagnostic_updater::Updater updater10(nh);  // NOLINT
  auto pub10 = pnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater10, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "~topic", "b", 10);

  diagnostic_updater::Updater updater11(nh);  // NOLINT
  auto pub11 = tnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater11, "c", 10);

  diagnostic_updater::Updater updater12(nh);  // NOLINT
  auto pub12 = tnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater12, "topic", "c", 10);

  diagnostic_updater::Updater updater13(nh);  // NOLINT
  auto pub13 = tnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater13, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "topic", "c", 10);

  diagnostic_updater::Updater updater14(nh);  // NOLINT
  auto pub14 = tnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater14, "~topic", "c", 10);

  diagnostic_updater::Updater updater15(nh);  // NOLINT
  auto pub15 = tnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater15, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "~topic", "c", 10);
  
  ros::AdvertiseOptions opts16;
  opts16.init<diagnostic_msgs::DiagnosticArray>("a", 10);
  diagnostic_updater::Updater updater16(nh);  // NOLINT
  auto pub16 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater16, opts16);
  
  ros::AdvertiseOptions opts17;
  opts17.init<diagnostic_msgs::DiagnosticArray>("a", 10);
  diagnostic_updater::Updater updater17(nh);  // NOLINT
  auto pub17 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater17, "topic", opts17);
  
  ros::AdvertiseOptions opts18;
  opts18.init<diagnostic_msgs::DiagnosticArray>("a", 10);
  diagnostic_updater::Updater updater18(nh);  // NOLINT
  auto pub18 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater18, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "topic", opts18);
  
  ros::AdvertiseOptions opts19;
  opts19.init<diagnostic_msgs::DiagnosticArray>("a", 10);
  diagnostic_updater::Updater updater19(nh);  // NOLINT
  auto pub19 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater19, "~topic", opts19);
  
  ros::AdvertiseOptions opts20;
  opts20.init<diagnostic_msgs::DiagnosticArray>("a", 10);
  diagnostic_updater::Updater updater20(nh);  // NOLINT
  auto pub20 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(
    updater20, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "~topic", opts20);

  // topic remapped to /d, but parameter read from ~a
  diagnostic_updater::Updater updater21(nh);  // NOLINT
  auto pub21 = rnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater21, "a", 10);

  // topic remapped to /d, but parameter read from /a
  diagnostic_updater::Updater updater22(nh);  // NOLINT
  auto pub22 = rnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater22, "/a", 10);

  diagnostic_updater::Updater updater23(nh);  // NOLINT
  auto pub23 = nh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater23, "/a", 10);

  diagnostic_updater::Updater updater24(nh);  // NOLINT
  auto pub24 = pnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater24, "/a", 10);

  diagnostic_updater::Updater updater25(nh);  // NOLINT
  auto pub25 = tnh.advertiseDiagnosed<diagnostic_msgs::DiagnosticArray>(updater25, "/a", 10);
  
  ros::WallDuration(0.25).sleep();
  
  const size_t numCallbacks = 25;

  for (size_t i = 0; i < 100 && numDiagCalled < numCallbacks; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  diagnostic_msgs::DiagnosticArray pubMsg;
  
  for (size_t i = 0; i < 10; ++i)
  {
    ros::Time::setNow(ros::Time(10) + ros::Duration(0.1) * i);
    pubMsg.header.stamp = ros::Time(9.9) + ros::Duration(0.1) * i;
    pub1->publish(pubMsg);
    pub2->publish(pubMsg);
    pub3->publish(pubMsg);
    pub4->publish(pubMsg);
    pub5->publish(pubMsg);
    pub6->publish(pubMsg);
    pub7->publish(pubMsg);
    pub8->publish(pubMsg);
    pub9->publish(pubMsg);
    pub10->publish(pubMsg);
    pub11->publish(pubMsg);
    pub12->publish(pubMsg);
    pub13->publish(pubMsg);
    pub14->publish(pubMsg);
    pub15->publish(pubMsg);
    pub16->publish(pubMsg);
    pub17->publish(pubMsg);
    pub18->publish(pubMsg);
    pub19->publish(pubMsg);
    pub20->publish(pubMsg);
    pub21->publish(pubMsg);
    pub22->publish(pubMsg);
    pub23->publish(pubMsg);
    pub24->publish(pubMsg);
    pub25->publish(pubMsg);
  }
  
  ros::Time::setNow({11, 0});

  for (size_t i = 0; i < 100 && numCalled < numCallbacks * 10u; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(numCallbacks * 10u, numCalled);
  
  msg = nullptr;
  numDiagCalled = 0;
  updater1.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater2.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater3.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater4.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater5.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater6.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater7.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater8.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater9.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater10.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater11.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater12.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater13.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater14.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater15.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater16.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater17.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater18.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater19.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater20.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater21.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater22.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater23.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater24.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater25.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(7u, msg->status[0].values.size());
}

TEST(NodeUtils, SubscribeConfiguration)  // NOLINT
{
  cras::NodeHandle nh;
  cras::NodeHandle pnh("~");
  cras::NodeHandle tnh("test");
  cras::NodeHandle rnh("", {{"/a", "/d"}});
  
  ros::Time::setNow({10, 0});
  diagnostic_msgs::DiagnosticArrayConstPtr msg;
  size_t numDiagCalled = 0;
  auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10,
    [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});
  
  pnh.deleteParam("/");
  pnh.setParam("topic/rate/min", 5.0);
  pnh.setParam("topic/rate/max", 5.0);
  pnh.setParam("topic/rate/tolerance", 0.1);
  pnh.setParam("topic/rate/window_size", 10);
  pnh.setParam("topic/delay/min", 2.0);
  pnh.setParam("topic/delay/max", 3.0);
  pnh.setParam("a/rate/min", 20.0);
  pnh.setParam("a/rate/max", 20.0);
  pnh.setParam("a/rate/tolerance", 0.1);
  pnh.setParam("a/rate/window_size", 10);
  pnh.setParam("a/delay/min", -2.0);
  pnh.setParam("a/delay/max", -1.0);
  pnh.setParam("b/rate/min", 20.0);
  pnh.setParam("b/rate/max", 20.0);
  pnh.setParam("b/rate/tolerance", 0.1);
  pnh.setParam("b/rate/window_size", 10);
  pnh.setParam("b/delay/min", -2.0);
  pnh.setParam("b/delay/max", -1.0);
  pnh.setParam("/topic/rate/min", 20.0);
  pnh.setParam("/topic/rate/max", 20.0);
  pnh.setParam("/topic/rate/tolerance", 0.1);
  pnh.setParam("/topic/rate/window_size", 10);
  pnh.setParam("/topic/delay/min", 2.0);
  pnh.setParam("/topic/delay/max", 3.0);
  pnh.setParam("/test/c/rate/min", 20.0);
  pnh.setParam("/test/c/rate/max", 20.0);
  pnh.setParam("/test/c/rate/tolerance", 0.1);
  pnh.setParam("/test/c/rate/window_size", 10);
  pnh.setParam("/test/c/delay/min", -2.0);
  pnh.setParam("/test/c/delay/max", -1.0);
  pnh.setParam("/test/topic/rate/min", 5.0);
  pnh.setParam("/test/topic/rate/max", 5.0);
  pnh.setParam("/test/topic/rate/tolerance", 0.1);
  pnh.setParam("/test/topic/rate/window_size", 10);
  pnh.setParam("/test/topic/delay/min", 2.0);
  pnh.setParam("/test/topic/delay/max", 3.0);
  pnh.setParam("/a/rate/min", 5.0);
  pnh.setParam("/a/rate/max", 5.0);
  pnh.setParam("/a/rate/tolerance", 0.1);
  pnh.setParam("/a/rate/window_size", 10);
  pnh.setParam("/a/delay/min", 2.0);
  pnh.setParam("/a/delay/max", 3.0);

  auto pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  auto pub2 = pnh.advertise<diagnostic_msgs::DiagnosticArray>("b", 10);
  auto pub3 = tnh.advertise<diagnostic_msgs::DiagnosticArray>("c", 10);
  auto pub4 = nh.advertise<diagnostic_msgs::DiagnosticArray>("d", 10);
  
  size_t numCalled {0};

  diagnostic_updater::Updater updater1(nh);  // NOLINT
  auto sub1 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater1, "a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater2(nh);  // NOLINT
  auto sub2 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater2, "topic", "a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater3(nh);  // NOLINT
  auto sub3 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater3, {10.0, 10.0, 0.1, 5, 0.5, 1.0}, "topic",
    "a", 10, [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater4(nh);  // NOLINT
  auto sub4 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater4, "~topic", "a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater5(nh);  // NOLINT
  auto sub5 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater5, {10.0, 10.0, 0.1, 5, 0.5, 1.0},
    "~topic", "a", 10, [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater6(nh);  // NOLINT
  auto sub6 = pnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater6, "b", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater7(nh);  // NOLINT
  auto sub7 = pnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater7, "topic", "b", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater8(nh);  // NOLINT
  auto sub8 = pnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater8, {10.0, 10.0, 0.1, 5, 0.5, 1.0},
    "topic", "b", 10, [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater9(nh);  // NOLINT
  auto sub9 = pnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater9, "~topic", "b", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater10(nh);  // NOLINT
  auto sub10 = pnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater10, {10.0, 10.0, 0.1, 5, 0.5, 1.0},
    "~topic", "b", 10, [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  ros::SubscribeOptions opts11;
  opts11.init<diagnostic_msgs::DiagnosticArray>("a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  diagnostic_updater::Updater updater11(nh);  // NOLINT
  auto sub11 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater11, opts11);

  ros::SubscribeOptions opts12;
  opts12.init<diagnostic_msgs::DiagnosticArray>("a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  diagnostic_updater::Updater updater12(nh);  // NOLINT
  auto sub12 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater12, "topic", opts12);

  ros::SubscribeOptions opts13;
  opts13.init<diagnostic_msgs::DiagnosticArray>("a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  diagnostic_updater::Updater updater13(nh);  // NOLINT
  auto sub13 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater13, {10.0, 10.0, 0.1, 5, 0.5, 1.0},
    "topic", opts13);

  ros::SubscribeOptions opts14;
  opts14.init<diagnostic_msgs::DiagnosticArray>("a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  diagnostic_updater::Updater updater14(nh);  // NOLINT
  auto sub14 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater14, "~topic", opts14);

  ros::SubscribeOptions opts15;
  opts15.init<diagnostic_msgs::DiagnosticArray>("a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});
  diagnostic_updater::Updater updater15(nh);  // NOLINT
  auto sub15 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater15, {10.0, 10.0, 0.1, 5, 0.5, 1.0},
    "~topic", opts15);

  diagnostic_updater::Updater updater16(nh);  // NOLINT
  auto sub16 = tnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater16, "c", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater17(nh);  // NOLINT
  auto sub17 = tnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater17, "topic", "c", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater18(nh);  // NOLINT
  auto sub18 = tnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater18, "~topic", "c", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater19(nh);  // NOLINT
  auto sub19 = rnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater19, "a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater20(nh);  // NOLINT
  auto sub20 = rnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater20, "/a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater21(nh);  // NOLINT
  auto sub21 = nh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater21, "/a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater22(nh);  // NOLINT
  auto sub22 = pnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater22, "/a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  diagnostic_updater::Updater updater23(nh);  // NOLINT
  auto sub23 = tnh.subscribeDiagnosed<diagnostic_msgs::DiagnosticArray>(updater23, "/a", 10,
    [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numCalled;});

  ros::WallDuration(0.25).sleep();
  
  const size_t numCallbacks = 23;

  for (size_t i = 0; i < 100 && numDiagCalled < numCallbacks; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  diagnostic_msgs::DiagnosticArray pubMsg;
  
  for (size_t i = 0; i < 10; ++i)
  {
    ros::Time::setNow(ros::Time(10) + ros::Duration(0.1) * i);
    pubMsg.header.stamp = ros::Time(9.9) + ros::Duration(0.1) * i;
    pub.publish(pubMsg);
    pub2.publish(pubMsg);
    pub3.publish(pubMsg);
    pub4.publish(pubMsg);
  }
  
  ros::Time::setNow({11, 0});

  for (size_t i = 0; i < 100 && numCalled < numCallbacks * 10u; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(numCallbacks * 10u, numCalled);
  
  msg = nullptr;
  numDiagCalled = 0;
  updater1.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater2.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater3.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater4.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater5.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater6.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater7.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater8.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater9.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater10.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater11.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater12.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater13.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater14.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater15.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater16.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater17.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater18.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater19.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater20.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater21.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater22.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
  
  msg = nullptr;
  numDiagCalled = 0;
  updater23.force_update();
  for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, numDiagCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
}

TEST(NodeUtils, SubscribeSignatures1ArgNoHeader)  // NOLINT
{
  using TestClass = CbTest;
  using Msg = std_msgs::Header;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd,), CRAS_TEST_SINGLE_ARG("a", 10,),,
    m.frame_id, m->frame_id, m.getConstMessage()->frame_id)
  
  auto pub = nh.advertise<Msg>("a", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures1ArgWithHeader)  // NOLINT
{
  using TestClass = CbTestHeader;
  using Msg = diagnostic_msgs::DiagnosticArray;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd,), CRAS_TEST_SINGLE_ARG("b", 10,), Header,
    m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
  
  auto pub = nh.advertise<Msg>("b", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures2ArgNoHeader)  // NOLINT
{
  using TestClass = CbTest;
  using Msg = std_msgs::Header;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, "ns",), CRAS_TEST_SINGLE_ARG("c", 10,),,
    m.frame_id, m->frame_id, m.getConstMessage()->frame_id)
  
  auto pub = nh.advertise<Msg>("c", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures2ArgWithHeader)  // NOLINT
{
  using TestClass = CbTestHeader;
  using Msg = diagnostic_msgs::DiagnosticArray;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, "ns",), CRAS_TEST_SINGLE_ARG("d", 10,), Header,
    m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
  
  auto pub = nh.advertise<Msg>("d", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures3ArgNoHeader)  // NOLINT
{
  using TestClass = CbTest;
  using Msg = std_msgs::Header;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, {}, "ns",), CRAS_TEST_SINGLE_ARG("e", 10,),,
    m.frame_id, m->frame_id, m.getConstMessage()->frame_id)
  
  auto pub = nh.advertise<Msg>("e", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures3ArgWithHeader)  // NOLINT
{
  using TestClass = CbTestHeader;
  using Msg = diagnostic_msgs::DiagnosticArray;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, {}, "ns",), CRAS_TEST_SINGLE_ARG("f", 10,), Header,
    m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
  
  auto pub = nh.advertise<Msg>("f", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures3ArgFilledNoHeader)  // NOLINT
{
  using TestClass = CbTest;
  using Msg = std_msgs::Header;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;

  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, {CRAS_TEST_SINGLE_ARG(5.0, 10.0, 0.1, 10)}, "ns",), CRAS_TEST_SINGLE_ARG("g", 10,),,
    m.frame_id, m->frame_id, m.getConstMessage()->frame_id)
  
  auto pub = nh.advertise<Msg>("g", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures3ArgFilledWithHeader)  // NOLINT
{
  using TestClass = CbTestHeader;
  using Msg = diagnostic_msgs::DiagnosticArray;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, {CRAS_TEST_SINGLE_ARG(5.0, 10.0, 0.1, 10, 0.1, 9.0)}, "nh",),
    CRAS_TEST_SINGLE_ARG("h", 10,), Header,
    m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
  
  auto pub = nh.advertise<Msg>("h", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures3ArgDesignatedInitNoHeader)  // NOLINT
{
  using TestClass = CbTest;
  using Msg = std_msgs::Header;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;

  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, {.maxRate = 10.0}, "ns",), CRAS_TEST_SINGLE_ARG("i", 10,),,
    m.frame_id, m->frame_id, m.getConstMessage()->frame_id)
  
  auto pub = nh.advertise<Msg>("i", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

TEST(NodeUtils, SubscribeSignatures3ArgDesignatedInitWithHeader)  // NOLINT
{
  using TestClass = CbTestHeader;
  using Msg = diagnostic_msgs::DiagnosticArray;
  auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

  ::cras::NodeHandle nh;
  ::diagnostic_updater::Updater upd;
  
  CRAS_TEST_SUBSCRIBE_CALLBACKS(nh.subscribeDiagnosed,  // NOLINT
    CRAS_TEST_SINGLE_ARG(upd, {CRAS_TEST_SINGLE_ARG(.maxRate = 10.0, .maxDelay = 6.0)}, "nh",),
    CRAS_TEST_SINGLE_ARG("j", 10,), Header,
    m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
  
  auto pub = nh.advertise<Msg>("j", 100);
  auto message = Msg();
  getFrame(message) = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  size_t numOk {0};
  while (ros::WallTime::now() < end && numOk < strs.size())
  {
    ros::spinOnce();
    numOk = 0;
    for (const auto str : strs)
      numOk += *str == getFrame(message);
    ros::WallDuration(0.01).sleep();
  }

  size_t i = 0;
  for (const auto str : strs)
  {
    SCOPED_TRACE("Error was in interation " + std::to_string(i));
    EXPECT_EQ(getFrame(message), *str);
    ++i;
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_node_utils");
  ros::start();

  return RUN_ALL_TESTS();
}