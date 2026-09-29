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
  return std::make_shared<ImageTransportNode>(node_options, cb);
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
    "image_raw/compressed", rclcpp::SensorDataQoS(sub_qos), img_compressed_cb);
  subs.push_back(img_compressed_sub);

  im_subs.push_back(node->image_sub_);

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
  std::vector<uint8_t> expected_data = {
    255, 216, 255, 224, 0, 16, 74, 70, 73, 70, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 255, 219, 0, 67, 0, 2, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 2, 2, 2, 2, 2, 4, 3, 2, 2, 2, 2, 5, 4, 4, 3, 4, 6, 5, 6, 6, 6, 5, 6, 6, 6, 7, 9, 8, 6, 7, 9, 7, 6, 6, 8, 11,
    8, 9, 10, 10, 10, 10, 10, 6, 8, 11, 12, 11, 10, 12, 9, 10, 10, 10, 255, 219, 0, 67, 1, 2, 2, 2, 2, 2, 2, 5, 3, 3, 5,
    10, 7, 6, 7, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 255, 192, 0, 17, 8,
    0, 2, 0, 2, 3, 1, 34, 0, 2, 17, 1, 3, 17, 1, 255, 196, 0, 31, 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 255, 196, 0, 181, 16, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125, 1, 2, 3,
    0, 4, 17, 5, 18, 33, 49, 65, 6, 19, 81, 97, 7, 34, 113, 20, 50, 129, 145, 161, 8, 35, 66, 177, 193, 21, 82, 209,
    240, 36, 51, 98, 114, 130, 9, 10, 22, 23, 24, 25, 26, 37, 38, 39, 40, 41, 42, 52, 53, 54, 55, 56, 57, 58, 67, 68,
    69, 70, 71, 72, 73, 74, 83, 84, 85, 86, 87, 88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118,
    119, 120, 121, 122, 131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162, 163,
    164, 165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194, 195, 196, 197, 198, 199, 200,
    201, 202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 241, 242,
    243, 244, 245, 246, 247, 248, 249, 250, 255, 196, 0, 31, 1, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 2,
    3, 4, 5, 6, 7, 8, 9, 10, 11, 255, 196, 0, 181, 17, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119, 0, 1, 2, 3, 17,
    4, 5, 33, 49, 6, 18, 65, 81, 7, 97, 113, 19, 34, 50, 129, 8, 20, 66, 145, 161, 177, 193, 9, 35, 51, 82, 240, 21, 98,
    114, 209, 10, 22, 36, 52, 225, 37, 241, 23, 24, 25, 26, 38, 39, 40, 41, 42, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70,
    71, 72, 73, 74, 83, 84, 85, 86, 87, 88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119, 120,
    121, 122, 130, 131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162, 163, 164,
    165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194, 195, 196, 197, 198, 199, 200, 201,
    202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 226, 227, 228, 229, 230, 231, 232, 233, 234, 242, 243, 244, 245,
    246, 247, 248, 249, 250, 255, 218, 0, 12, 3, 1, 0, 2, 17, 3, 17, 0, 63, 0, 253, 144, 248, 87, 251, 16, 126, 197,
    182, 159, 12, 60, 55, 107, 107, 251, 33, 124, 47, 138, 40, 180, 27, 52, 142, 56, 252, 1, 167, 42, 162, 136, 16, 0,
    0, 135, 0, 1, 218, 138, 40, 160, 15, 255, 217,
  };
  EXPECT_EQ(expected_data.size(), last_compressed_img->data.size());
  EXPECT_EQ(expected_data, last_compressed_img->data);

  EXPECT_EQ(time, last_img->header.stamp);
  EXPECT_EQ("image", last_img->header.frame_id);
  EXPECT_EQ("bgr8", last_img->encoding);
  EXPECT_EQ(img.data.size(), last_img->data.size());
  for (size_t i = 0; i < last_img->data.size(); ++i) {
    // the color changes a bit after compression and decompression
    EXPECT_LT(std::abs(last_img->data[i] - img.data[i]), 20);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
