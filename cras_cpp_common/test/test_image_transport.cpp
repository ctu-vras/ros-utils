// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for cras::ImageTransport
 * \author Martin Pecka
 */

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <cras_cpp_common/image_transport.hpp>
#include <cras_cpp_common/test_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using CompressedImage = sensor_msgs::msg::CompressedImage;
using Image = sensor_msgs::msg::Image;

using namespace std::chrono_literals;

struct ImageTransportNode : rclcpp::Node {
  explicit ImageTransportNode(const rclcpp::NodeOptions node_options, const image_transport::Subscriber::Callback& cb)
      : rclcpp::Node("image_transport_test", node_options), image_transport_(*this) {
    image_pub_ = image_transport_.advertise("image_raw", rclcpp::QoS(1), {});
    const cras::ImageTransportHints hints(*this, "compressed");
    image_sub_ = image_transport_.subscribe(
      "image_raw", rclcpp::QoS(1), cb, nullptr, &hints, rclcpp::SubscriptionOptions());
  }

  cras::ImageTransport image_transport_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;
};

std::shared_ptr<ImageTransportNode> createNode(
    const image_transport::Subscriber::Callback cb, rclcpp::NodeOptions node_options = rclcpp::NodeOptions()) {
  rclcpp::NodeOptions options = node_options;
  options.arguments({
      "--ros-args",
      "-r", "image_raw:=image_remapped",
      // TODO double remaps should be handled correctly, but so far they're not
      // "-r", "image_remapped:=image_wrong",  // test for double remaps
  });
  return std::make_shared<ImageTransportNode>(options, cb);
}

class ImageTransport : public cras::RclcppTestFixture {};

TEST_F(ImageTransport, Basic)  // NOLINT
{
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

  auto node = createNode(img_cb);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  std::list<image_transport::Publisher> pubs;
  pubs.push_back(node->image_pub_);

  std::list<rclcpp::Subscription<CompressedImage>::ConstSharedPtr> subs;
  std::list<image_transport::Subscriber> im_subs;

  auto img_compressed_sub = node->create_subscription<CompressedImage>(
    "image_remapped/compressed", rclcpp::SensorDataQoS(sub_qos), img_compressed_cb);
  subs.push_back(img_compressed_sub);

  im_subs.push_back(node->image_sub_);

  EXPECT_EQ("/image_remapped", node->image_pub_.getTopic());
  EXPECT_EQ("/image_remapped/compressed", node->image_sub_.getTopic());
  EXPECT_STREQ("/image_remapped/compressed", img_compressed_sub->get_topic_name());

  for (const auto& [topic, types] : node->get_node_graph_interface()->get_publisher_names_and_types_by_node(
      node->get_name(), node->get_namespace())) {
    EXPECT_EQ(topic.find("wrong"), std::string::npos);
  }
  for (const auto& [topic, types] : node->get_node_graph_interface()->get_subscriber_names_and_types_by_node(
      node->get_name(), node->get_namespace())) {
    EXPECT_EQ(topic.find("wrong"), std::string::npos);
  }

  const auto pub_test = [](const image_transport::Publisher& p) {return p.getNumSubscribers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto sub_test = [](const rclcpp::Subscription<CompressedImage>::ConstSharedPtr& p) {
    return p->get_publisher_count() == 0;
  };
  const auto im_sub_test = [](const image_transport::Subscriber& p) {return p.getNumPublishers() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), sub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }
  for (size_t i = 0; i < 1000 && std::any_of(im_subs.begin(), im_subs.end(), im_sub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pub_test));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), sub_test));
  ASSERT_FALSE(std::any_of(im_subs.begin(), im_subs.end(), im_sub_test));

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
  std::vector<uint8_t> expected_data_preamble = {
    255, 216,
  };
  ASSERT_LT(expected_data_preamble.size(), last_compressed_img->data.size());
  for (size_t i = 0; i < expected_data_preamble.size(); ++i) {
    EXPECT_EQ(expected_data_preamble[i], last_compressed_img->data[i]);
  }

  EXPECT_EQ(time, last_img->header.stamp);
  EXPECT_EQ("image", last_img->header.frame_id);
  EXPECT_EQ("bgr8", last_img->encoding);
  ASSERT_EQ(img.data.size(), last_img->data.size());
  for (size_t i = 0; i < last_img->data.size(); ++i) {
    // the color changes a bit after compression and decompression
    EXPECT_LT(std::abs(last_img->data[i] - img.data[i]), 20);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
