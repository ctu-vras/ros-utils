// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for cras::ImageTransport and cras::PointCloudTransport in a single node. This tests mainly that the
 *        libraries can be loaded side-by-side (without any conflicts like ODR).
 * \author Martin Pecka
 */

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <cras_cpp_common/image_transport.hpp>
#include <cras_cpp_common/point_cloud_transport.hpp>
#include <cras_cpp_common/test_utils.hpp>
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using CompressedImage = sensor_msgs::msg::CompressedImage;
using CompressedPC2 = point_cloud_interfaces::msg::CompressedPointCloud2;
using Image = sensor_msgs::msg::Image;
using PC2 = sensor_msgs::msg::PointCloud2;

using namespace std::chrono_literals;

struct TransportNode : rclcpp::Node {
  explicit TransportNode(
      const rclcpp::NodeOptions node_options, const image_transport::Subscriber::Callback& cb_img,
      const point_cloud_transport::Subscriber::Callback& cb_pc2)
      : rclcpp::Node("transport_test", node_options), image_transport_(*this), point_cloud_transport_(*this) {
    image_pub_ = image_transport_.advertise("image_raw", rclcpp::QoS(1), {});
    const cras::ImageTransportHints hints_img(*this, "compressed");
    image_sub_ = image_transport_.subscribe("image_raw", rclcpp::QoS(1), cb_img, nullptr, &hints_img);

    point_cloud_pub_ = point_cloud_transport_.advertise("pcl", rclcpp::QoS(1), {});
    const cras::PointCloudTransportHints hints_pc2(*this, "zstd");
    point_cloud_sub_ = point_cloud_transport_.subscribe("pcl", rclcpp::QoS(1), cb_pc2, nullptr, &hints_pc2);
  }

  cras::ImageTransport image_transport_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;

  cras::PointCloudTransport point_cloud_transport_;
  point_cloud_transport::Publisher point_cloud_pub_;
  point_cloud_transport::Subscriber point_cloud_sub_;
};

std::shared_ptr<TransportNode> createNode(
    const image_transport::Subscriber::Callback cb_img, const point_cloud_transport::Subscriber::Callback cb_pc2,
    rclcpp::NodeOptions node_options = rclcpp::NodeOptions()) {
  return std::make_shared<TransportNode>(node_options, cb_img, cb_pc2);
}

class AllTransports : public cras::RclcppTestFixture {};

TEST_F(AllTransports, Basic)  // NOLINT
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

  std::optional<CompressedImage> last_compressed_img;
  auto img_compressed_cb =
    [&last_compressed_img](const CompressedImage::ConstSharedPtr& msg) {
      last_compressed_img = *msg;
  };

  std::optional<Image> last_img;
  auto img_cb =
    [&last_img](const Image::ConstSharedPtr& msg) {
      last_img = *msg;
  };

  auto node = createNode(img_cb, pc_cb);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  std::list<image_transport::Publisher> pubs_im;
  pubs_im.push_back(node->image_pub_);

  std::list<rclcpp::Subscription<CompressedImage>::ConstSharedPtr> subs_im;
  std::list<image_transport::Subscriber> im_subs_im;

  auto img_compressed_sub = node->create_subscription<CompressedImage>(
    "image_raw/compressed", rclcpp::SensorDataQoS(sub_qos), img_compressed_cb);
  subs_im.push_back(img_compressed_sub);

  im_subs_im.push_back(node->image_sub_);

  std::list<point_cloud_transport::Publisher> pubs_pc;
  pubs_pc.push_back(node->point_cloud_pub_);

  std::list<rclcpp::Subscription<CompressedPC2>::ConstSharedPtr> subs_pc;
  std::list<point_cloud_transport::Subscriber> pc_subs_pc;

  auto pc_compressed_sub = node->create_subscription<CompressedPC2>(
    "pcl/zstd", rclcpp::SensorDataQoS(sub_qos), pc_compressed_cb);
  subs_pc.push_back(pc_compressed_sub);

  pc_subs_pc.push_back(node->point_cloud_sub_);

  const auto pub_test_im = [](const image_transport::Publisher& p) {return p.getNumSubscribers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs_im.begin(), pubs_im.end(), pub_test_im); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto sub_test_im = [](const rclcpp::Subscription<CompressedImage>::ConstSharedPtr& p) {
    return p->get_publisher_count() == 0;
  };
  const auto im_sub_test = [](const image_transport::Subscriber& p) {return p.getNumPublishers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(subs_im.begin(), subs_im.end(), sub_test_im); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }
  for (size_t i = 0; i < 1000 && std::any_of(im_subs_im.begin(), im_subs_im.end(), im_sub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs_im.begin(), pubs_im.end(), pub_test_im));
  ASSERT_FALSE(std::any_of(subs_im.begin(), subs_im.end(), sub_test_im));
  ASSERT_FALSE(std::any_of(im_subs_im.begin(), im_subs_im.end(), im_sub_test));

  const auto pub_test_pc = [](const point_cloud_transport::Publisher& p) {return p.getNumSubscribers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs_pc.begin(), pubs_pc.end(), pub_test_pc); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto sub_test_pc = [](const rclcpp::Subscription<CompressedPC2>::ConstSharedPtr& p) {
    return p->get_publisher_count() == 0;
  };
  const auto pc_sub_test_pc = [](const point_cloud_transport::Subscriber& p) {return p.getNumPublishers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(subs_pc.begin(), subs_pc.end(), sub_test_pc); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }
  for (size_t i = 0; i < 1000 && std::any_of(pc_subs_pc.begin(), pc_subs_pc.end(), pc_sub_test_pc); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs_pc.begin(), pubs_pc.end(), pub_test_pc));
  ASSERT_FALSE(std::any_of(subs_pc.begin(), subs_pc.end(), sub_test_pc));
  ASSERT_FALSE(std::any_of(pc_subs_pc.begin(), pc_subs_pc.end(), pc_sub_test_pc));

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  Image img;
  img.header.stamp = time;
  img.header.frame_id = "image";
  img.encoding = "bgr8";
  img.width = 2;
  img.height = 2;
  img.step = img.width * 3;
  img.data = {
    0, 0, 0,
    100, 100, 100,
    200, 200, 200,
    255, 255, 255,
  };
  node->image_pub_.publish(img);

  for (size_t i = 0; i < 5 && !last_compressed_img.has_value() && rclcpp::ok(); ++i) {
    executor.spin_all(100ms);
  }

  ASSERT_TRUE(last_compressed_img.has_value());
  ASSERT_TRUE(last_img.has_value());

  EXPECT_EQ(time, last_compressed_img->header.stamp);
  EXPECT_EQ("image", last_compressed_img->header.frame_id);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", last_compressed_img->format);

  EXPECT_EQ(time, last_img->header.stamp);
  EXPECT_EQ("image", last_img->header.frame_id);
  EXPECT_EQ("bgr8", last_img->encoding);

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

  EXPECT_EQ(time, last_pc->header.stamp);
  EXPECT_EQ("pcl", last_pc->header.frame_id);
  EXPECT_EQ(pc.data.size(), last_pc->data.size());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
