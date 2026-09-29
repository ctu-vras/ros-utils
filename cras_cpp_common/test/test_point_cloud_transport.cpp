// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for cras::PointCloudTransport
 * \author Martin Pecka
 */

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <cras_cpp_common/point_cloud_transport.hpp>
#include <cras_cpp_common/test_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using CompressedPC2 = point_cloud_interfaces::msg::CompressedPointCloud2;
using PC2 = sensor_msgs::msg::PointCloud2;

using namespace std::chrono_literals;

struct PointCloudTransportNode : rclcpp::Node {
  explicit PointCloudTransportNode(
      const rclcpp::NodeOptions node_options, const point_cloud_transport::Subscriber::Callback& cb)
      : rclcpp::Node("point_cloud_transport_test", node_options), point_cloud_transport_(*this) {
    point_cloud_pub_ = point_cloud_transport_.advertise("pcl", rclcpp::QoS(1), {});
    const cras::PointCloudTransportHints hints(*this, "zstd");
    point_cloud_sub_ = point_cloud_transport_.subscribe("pcl", rclcpp::QoS(1), cb, nullptr, &hints);
  }

  cras::PointCloudTransport point_cloud_transport_;
  point_cloud_transport::Publisher point_cloud_pub_;
  point_cloud_transport::Subscriber point_cloud_sub_;
};

std::shared_ptr<PointCloudTransportNode> createNode(
    const point_cloud_transport::Subscriber::Callback cb, rclcpp::NodeOptions node_options = rclcpp::NodeOptions()) {
  return std::make_shared<PointCloudTransportNode>(node_options, cb);
}

class PointCloudTransport : public cras::RclcppTestFixture {};

TEST_F(PointCloudTransport, Basic)  // NOLINT
{
  std::optional<CompressedPC2> last_compressed_msg;
  auto pc_compressed_cb =
    [&last_compressed_msg](const CompressedPC2::ConstSharedPtr& msg) {
      last_compressed_msg = *msg;
    };

  std::optional<PC2> last_pc;
  auto pc_cb =
    [&last_pc](const PC2::ConstSharedPtr& msg) {
      last_pc = *msg;
    };

  auto node = createNode(pc_cb);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  std::list<point_cloud_transport::Publisher> pubs;
  pubs.push_back(node->point_cloud_pub_);

  std::list<rclcpp::Subscription<CompressedPC2>::ConstSharedPtr> subs;
  std::list<point_cloud_transport::Subscriber> pc_subs;

  auto pc_compressed_sub = node->create_subscription<CompressedPC2>(
    "pcl/zstd", rclcpp::SensorDataQoS(sub_qos), pc_compressed_cb);
  subs.push_back(pc_compressed_sub);

  pc_subs.push_back(node->point_cloud_sub_);

  const auto pub_test = [](const point_cloud_transport::Publisher& p) {return p.getNumSubscribers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto sub_test = [](const rclcpp::Subscription<CompressedPC2>::ConstSharedPtr& p) {
    return p->get_publisher_count() == 0;
  };
  const auto pc_sub_test = [](const point_cloud_transport::Subscriber& p) {return p.getNumPublishers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), sub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }
  for (size_t i = 0; i < 1000 && std::any_of(pc_subs.begin(), pc_subs.end(), pc_sub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pub_test));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), sub_test));
  ASSERT_FALSE(std::any_of(pc_subs.begin(), pc_subs.end(), pc_sub_test));

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  PC2 pc;
  pc.header.stamp = time;
  pc.header.frame_id = "pcl";
  pc.width = 2;
  pc.height = 2;
  pc.point_step = 12;
  pc.row_step = pc.point_step * pc.width;

  sensor_msgs::msg::PointField pf;
  pf.name = "x";
  pf.count = 1;
  pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc.fields.push_back(pf);

  pf.name = "y";
  pf.offset = 4;
  pc.fields.push_back(pf);

  pf.name = "z";
  pf.offset = 8;
  pc.fields.push_back(pf);

  std::vector<float> data = {
    0.0, 1.0, 2.0,
    3.0, 4.0, 5.0,
    6.0, 7.0, 8.0,
    9.0, 10.0, 11.0,
  };

  union {
    float f;
    uint8_t b[4];
  } FloatConv;

  for (size_t i = 0; i < data.size(); ++i) {
    FloatConv.f = data[i];
    pc.data.push_back(FloatConv.b[0]);
    pc.data.push_back(FloatConv.b[1]);
    pc.data.push_back(FloatConv.b[2]);
    pc.data.push_back(FloatConv.b[3]);
  }

  node->point_cloud_pub_.publish(pc);

  for (size_t i = 0; i < 5 && !last_compressed_msg.has_value() && rclcpp::ok(); ++i) {
    executor.spin_all(100ms);
  }

  ASSERT_TRUE(last_compressed_msg.has_value());
  ASSERT_TRUE(last_pc.has_value());

  EXPECT_EQ(time, last_compressed_msg->header.stamp);
  EXPECT_EQ("pcl", last_compressed_msg->header.frame_id);
  EXPECT_EQ("zstd", last_compressed_msg->format);
  std::vector<uint8_t> expected_data = {
    40, 181, 47, 253, 32, 48, 129, 1, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 128, 64, 0, 0, 160,
    64, 0, 0, 192, 64, 0, 0, 224, 64, 0, 0, 0, 65, 0, 0, 16, 65, 0, 0, 32, 65, 0, 0, 48, 65,
  };
  EXPECT_EQ(expected_data.size(), last_compressed_msg->compressed_data.size());
  EXPECT_EQ(expected_data, last_compressed_msg->compressed_data);

  EXPECT_EQ(time, last_pc->header.stamp);
  EXPECT_EQ("pcl", last_pc->header.frame_id);
  EXPECT_EQ(pc.data.size(), last_pc->data.size());
  for (size_t i = 0; i < last_pc->data.size(); ++i) {
    EXPECT_EQ(pc.data[i], last_pc->data[i]);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
