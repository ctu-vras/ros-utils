/**
 * \file
 * \brief Unit test for filter_chain_nodelet.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// HACK: We need to pass private FilterChain.loader_ to PreloadingClassLoader to load the locally defined filter.
#include <sstream>
#include <ros/common.h>
#define private public
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_chain.h>
#endif
#undef private
// END HACK

#include "gtest/gtest.h"

#include <thread>

#include <cras_cpp_common/filter_utils/filter_chain_nodelet.hpp>
#include <cras_cpp_common/test_utils/preloading_class_loader.hpp>
#include <cras_cpp_common/type_utils.hpp>

#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

template <typename T>
class IncFilter : public cras::FilterBase<T>
{
public:
  bool update(const T& data_in, T& data_out) override
  {
    data_out = data_in;
    data_out.data += ++this->data;
    return true;
  }
protected:
  bool configure() override
  {
    return true;
  }
  decltype(T::data) data {0};
};

template <typename T>
class VectorFilter : public cras::FilterBase<T>
{
public:
  bool update(const T& data_in, T& data_out) override
  {
    data_out = data_in;
    data_out.vector.x += ++this->data;
    return true;
  }
  
protected:
  bool configure() override
  {
    return true;
  }
  double data {0};
};

template <typename T, template<typename> typename F>
class TestChainNodelet : public cras::FilterChainNodelet<T>
{
public:
  TestChainNodelet(const std::string& chainName) : cras::FilterChainNodelet<T>(chainName)
  {
    PreloadingClassLoader<F<T>, filters::FilterBase<T>>::preload(
      "cras_cpp_common/TestFilter", this->filterChain.loader_);
    ros::param::del("/nodelet/max_age");
    ros::param::del("/nodelet/disabled_filters");
    ros::param::del("/nodelet/publish_each_filter");
    ros::param::del("/nodelet/lazy_subscription");
  }
  
  ~TestChainNodelet() override
  {
    this->filterChain.clear();
    PreloadingClassLoader<F<T>, filters::FilterBase<T>>::unPreload(
      "cras_cpp_common/TestFilter", this->filterChain.loader_);
  }
  
  ros::NodeHandle publicNodeHandle()
  {
    return this->getNodeHandle();
  }
  
  ros::NodeHandle privateNodeHandle()
  {
    return this->getPrivateNodeHandle();
  }
};

TEST(FilterChainNodelet, BasicOneFilter)
{
  TestChainNodelet<std_msgs::Float32, IncFilter> nodelet("one_filter");
  nodelet.init("/nodelet", {}, {});
  
  auto nh = nodelet.publicNodeHandle();
  
  std_msgs::Float32ConstPtr filteredMsg = nullptr;
  size_t numReceived = 0;

  auto pub = nh.advertise<std_msgs::Float32>("in", 10);
  
  // Test that the filter chain does lazy subscription by default
  for (size_t i = 0; i < 100 && pub.getNumSubscribers() == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(0u, pub.getNumSubscribers());
  
  auto sub = nh.subscribe<std_msgs::Float32>("out", 10, [&](const std_msgs::Float32ConstPtr& msg)
    {
      filteredMsg = msg;
      numReceived++;
    });
  
  for (size_t i = 0; i < 100 && pub.getNumSubscribers() == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  EXPECT_EQ(1u, pub.getNumSubscribers());
  EXPECT_EQ(1u, sub.getNumPublishers());
  
  std_msgs::Float32 msg;
  msg.data = 0;
  
  pub.publish(msg);
  
  for (size_t i = 0; i < 10 && numReceived == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  EXPECT_EQ(1u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(1.0, filteredMsg->data);
  
  pub.publish(msg);
  
  for (size_t i = 0; i < 10 && numReceived == 1; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  EXPECT_EQ(2u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(2.0, filteredMsg->data);
}

TEST(FilterChainNodelet, ThreeFiltersEnableDisable)
{
  TestChainNodelet<std_msgs::Float32, IncFilter> nodelet("three_filters");
  ros::param::set("/nodelet/lazy_subscription", false);
  nodelet.init("/nodelet", {}, {});
  
  auto nh = nodelet.publicNodeHandle();
  
  std_msgs::Float32ConstPtr filteredMsg = nullptr;
  size_t numReceived = 0;
  
  auto pub = nh.advertise<std_msgs::Float32>("in", 10);
  
  // Lazy subscription is disabled, so the nodelet should subscribe immediately
  for (size_t i = 0; i < 100 && pub.getNumSubscribers() == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, pub.getNumSubscribers());

  auto sub = nh.subscribe<std_msgs::Float32>("out", 10, [&](const std_msgs::Float32ConstPtr& msg)
  {
    filteredMsg = msg;
    numReceived++;
  });
  
  ros::WallDuration(0.1).sleep();
  
  std_msgs::Float32 msg;
  msg.data = 0;
  
  pub.publish(msg);
  
  for (size_t i = 0; i < 10 && numReceived < 1; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  // filter1.data == 1 (enabled), filter2.data == 1 (enabled), filter3.data == 1 (enabled)
  EXPECT_EQ(1u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(3.0, filteredMsg->data);
  
  nodelet.setDisabledFilters({"filter1"});
  pub.publish(msg);
  
  for (size_t i = 0; i < 10 && numReceived < 2; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 1 (disabled), filter2.data == 2 (enabled), filter3.data == 2 (enabled)
  EXPECT_EQ(2u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(4.0, filteredMsg->data);
  
  nodelet.setDisabledFilters({"filter1", "filter3"});
  pub.publish(msg);
  
  for (size_t i = 0; i < 10 && numReceived < 3; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 1 (disabled), filter2.data == 3 (enabled), filter3.data == 2 (disabled)
  EXPECT_EQ(3u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(3.0, filteredMsg->data);
  
  nodelet.setDisabledFilters({});
  pub.publish(msg);
  
  for (size_t i = 0; i < 10 && numReceived < 4; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 2 (enabled), filter2.data == 4 (enabled), filter3.data == 3 (enabled)
  EXPECT_EQ(4u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(9.0, filteredMsg->data);
}


TEST(FilterChainNodelet, PublishDiagnostics)
{
  TestChainNodelet<std_msgs::Float32, IncFilter> nodelet("one_filter");
  ros::param::set("/nodelet/publish_diagnostics", true);
  nodelet.init("/nodelet", {}, {});

  auto nh = nodelet.publicNodeHandle();

  size_t numReceived = 0;
  auto sub = nh.subscribe<std_msgs::Float32>("out", 10, [&](const std_msgs::Float32ConstPtr& msg)
    {
      numReceived++;
    });

  diagnostic_msgs::DiagnosticArrayConstPtr diagMsg = nullptr;
  auto subDiag = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10,
    [&](const diagnostic_msgs::DiagnosticArrayConstPtr& msg)
    {
      diagMsg = msg;
    });

  auto pub = nh.advertise<std_msgs::Float32>("in", 10);

  for (size_t i = 0; i < 100 && (pub.getNumSubscribers() == 0 || subDiag.getNumPublishers() == 0); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(1u, pub.getNumSubscribers());
  EXPECT_EQ(1u, subDiag.getNumPublishers());

  // receive the "starting up" diag message
  for (size_t i = 0; i < 10 && diagMsg == nullptr; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  ASSERT_NE(nullptr, diagMsg);
  ASSERT_EQ(1u, diagMsg->status.size());
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, diagMsg->status[0].level);
  diagMsg = nullptr;
  
  pub.publish(std_msgs::Float32());
  
  // diagnostics should be published every second, so waiting a little over 1 second should make sure we get it
  for (size_t i = 0; i < 120 && (numReceived == 0 || diagMsg == nullptr); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  EXPECT_EQ(1u, numReceived);
  ASSERT_NE(nullptr, diagMsg);
  ASSERT_EQ(2u, diagMsg->status.size());
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, diagMsg->status[0].level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, diagMsg->status[1].level);
  EXPECT_EQ(4u, diagMsg->status[0].values.size());
  EXPECT_EQ(4u, diagMsg->status[1].values.size());
}

void updateConfig(dynamic_reconfigure::Client<cras_cpp_common::FilterChainConfig>& client,
  const cras_cpp_common::FilterChainConfig& config)
{
  bool stop = false;
  std::thread t([stop]() {
    for (size_t i = 0; i < 100 && not stop; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
  });
  client.setConfiguration(config);
  stop = true;
  t.join();
}

void getDefaultConfig(dynamic_reconfigure::Client<cras_cpp_common::FilterChainConfig>& client,
  cras_cpp_common::FilterChainConfig& config)
{
  bool stop = false;
  std::thread t([stop]() {
    for (size_t i = 0; i < 100 && not stop; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
  });
  client.getDefaultConfiguration(config);
  stop = true;
  t.join();
}

TEST(FilterChainNodelet, ThreeFiltersDynReconf)
{
  TestChainNodelet<geometry_msgs::Vector3Stamped, VectorFilter> n("three_filters");
  n.init("/nodelet", {}, {});
  
  auto nh = n.publicNodeHandle();

  dynamic_reconfigure::Client<cras_cpp_common::FilterChainConfig> client("client", ros::NodeHandle(nh, "nodelet"));
  
  geometry_msgs::Vector3StampedConstPtr filteredMsg = nullptr;
  size_t numReceived = 0;
  auto sub = nh.subscribe<geometry_msgs::Vector3Stamped>("out", 10,
    [&](const geometry_msgs::Vector3StampedConstPtr& msg)
    {
      filteredMsg = msg;
      numReceived++;
    });
  
  size_t numReceived1 = 0, numReceived2 = 0, numReceived3 = 0;
  geometry_msgs::Vector3StampedConstPtr filterMsg1 = nullptr, filterMsg2 = nullptr, filterMsg3 = nullptr;
  auto sub1 = nh.subscribe<geometry_msgs::Vector3Stamped>("nodelet/filter0/filter1", 10,
    [&](const geometry_msgs::Vector3StampedConstPtr& msg) { filterMsg1 = msg; numReceived1++; });
  auto sub2 = nh.subscribe<geometry_msgs::Vector3Stamped>("nodelet/filter1/filter2", 10,
    [&](const geometry_msgs::Vector3StampedConstPtr& msg) { filterMsg2 = msg; numReceived2++; });
  auto sub3 = nh.subscribe<geometry_msgs::Vector3Stamped>("nodelet/filter2/filter3", 10,
    [&](const geometry_msgs::Vector3StampedConstPtr& msg) { filterMsg3 = msg; numReceived3++; });
  
  auto pub = nh.advertise<geometry_msgs::Vector3Stamped>("in", 10);

  for (size_t i = 0; i < 100 && pub.getNumSubscribers() == 0; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  geometry_msgs::Vector3Stamped msg;
  msg.vector.x = 0;

  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived < 1; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  // filter1.data == 1 (enabled), filter2.data == 1 (enabled), filter3.data == 1 (enabled)
  EXPECT_EQ(1u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(3.0, filteredMsg->vector.x);
  
  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);
  
  cras_cpp_common::FilterChainConfig config;
  getDefaultConfig(client, config);
  config.disabled_filters = "filter1";
  updateConfig(client, config);

  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived < 2; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 1 (disabled), filter2.data == 2 (enabled), filter3.data == 2 (enabled)
  EXPECT_EQ(2u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(4.0, filteredMsg->vector.x);

  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);

  config.disabled_filters = "filter1,filter3";
  updateConfig(client, config);

  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived < 3; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 1 (disabled), filter2.data == 3 (enabled), filter3.data == 2 (disabled)
  EXPECT_EQ(3u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(3.0, filteredMsg->vector.x);

  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);

  config.disabled_filters = "";
  updateConfig(client, config);

  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived < 4; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 2 (enabled), filter2.data == 4 (enabled), filter3.data == 3 (enabled)
  EXPECT_EQ(4u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(9.0, filteredMsg->vector.x);

  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);

  const auto defaultMaxAge = config.max_age;

  // test that an old message does not pass through the filter
  
  msg.header.stamp = ros::Time::now() - ros::Duration(defaultMaxAge + 1);
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived == 4; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // check that no message was received
  EXPECT_EQ(4u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(9.0, filteredMsg->vector.x);

  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);
  
  // test that an old message does not pass through the filter with a custom max age

  config.max_age = defaultMaxAge / 2;
  updateConfig(client, config);
  
  msg.header.stamp = ros::Time::now() - ros::Duration(defaultMaxAge / 2 + 1);
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived == 4; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // check that no message was received
  EXPECT_EQ(4u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(9.0, filteredMsg->vector.x);

  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);

  // test that after changing maxAge, a not-old message does pass through the filter

  msg.header.stamp = ros::Time::now() - ros::Duration(defaultMaxAge / 2 - 1);
  pub.publish(msg);
  
  for (size_t i = 0; i < 40 && numReceived < 5; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 3 (enabled), filter2.data == 5 (enabled), filter3.data == 4 (enabled)
  EXPECT_EQ(5u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(12.0, filteredMsg->vector.x);

  EXPECT_EQ(0u, numReceived1);
  EXPECT_EQ(nullptr, filterMsg1);
  EXPECT_EQ(0u, numReceived2);
  EXPECT_EQ(nullptr, filterMsg2);
  EXPECT_EQ(0u, numReceived3);
  EXPECT_EQ(nullptr, filterMsg3);

  // test that individual filter publishers are working

  config.publish_each_filter = true;
  updateConfig(client, config);
  
  for (size_t i = 0;
    i < 10 && (sub1.getNumPublishers() == 0 || sub2.getNumPublishers() == 0 || sub3.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
  }

  msg.header.stamp = ros::Time::now();
  pub.publish(msg);

  for (size_t i = 0; i < 40 && (numReceived < 6 || numReceived1 < 1 || numReceived2 < 1 || numReceived3 < 1); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 4 (enabled), filter2.data == 6 (enabled), filter3.data == 5 (enabled)
  EXPECT_EQ(6u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(15.0, filteredMsg->vector.x);
  
  EXPECT_EQ(1u, numReceived1);
  ASSERT_NE(nullptr, filterMsg1);
  EXPECT_EQ(4.0, filterMsg1->vector.x);
  
  EXPECT_EQ(1u, numReceived2);
  ASSERT_NE(nullptr, filterMsg2);
  EXPECT_EQ(10.0, filterMsg2->vector.x);
  
  EXPECT_EQ(1u, numReceived3);
  ASSERT_NE(nullptr, filterMsg3);
  EXPECT_EQ(15.0, filterMsg3->vector.x);

  // test that individual filter publishers are not active when the functionality gets disabled again

  config.publish_each_filter = false;
  updateConfig(client, config);
  
  msg.header.stamp = ros::Time::now();
  pub.publish(msg);

  for (size_t i = 0; i < 40 && numReceived < 7; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  // filter1.data == 5 (enabled), filter2.data == 7 (enabled), filter3.data == 6 (enabled)
  EXPECT_EQ(7u, numReceived);
  ASSERT_NE(nullptr, filteredMsg);
  EXPECT_EQ(18.0, filteredMsg->vector.x);
  
  EXPECT_EQ(1u, numReceived1);
  ASSERT_NE(nullptr, filterMsg1);
  EXPECT_EQ(4.0, filterMsg1->vector.x);
  
  EXPECT_EQ(1u, numReceived2);
  ASSERT_NE(nullptr, filterMsg2);
  EXPECT_EQ(10.0, filterMsg2->vector.x);
  
  EXPECT_EQ(1u, numReceived3);
  ASSERT_NE(nullptr, filterMsg3);
  EXPECT_EQ(15.0, filterMsg3->vector.x);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_filter_chain_nodelet");
  ros::start();
  return RUN_ALL_TESTS();
}
