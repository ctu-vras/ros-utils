/**
 * \file
 * \brief Unit test for image_transport_codecs.cpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
// If OpenCV4
#if CV_VERSION_MAJOR > 3
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif

#include <compressed_image_transport/CompressedPublisherConfig.h>
#include <compressed_depth_image_transport/CompressedDepthPublisherConfig.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <image_transport_codecs/image_transport_codecs.h>
#include <image_transport_codecs/parse_compressed_format.h>

using namespace image_transport_codecs;  // NOLINT(build/namespaces)

TEST(ImageTransportCodecs, Raw)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;

  const auto compressedShifter = codecs.encode(raw, "raw");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::Image>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::Image>();
  EXPECT_EQ(*compressed, raw);

  const auto raw2 = codecs.decode(compressedShifter.value(), "raw");
  ASSERT_TRUE(raw2);
  EXPECT_EQ(raw2.value(), raw);
}

TEST(ImageTransportCodecs, CompressedJPEG)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "bgr8";
  raw.width = raw.height = 2;
  raw.step = 6;
  raw.data = {0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255};

  const auto compressedShifter = codecs.encode(raw, "compressed");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", compressed->format);

  const auto raw2 = codecs.decode(compressedShifter.value(), "compressed");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  for (size_t i = 0; i < raw.data.size(); ++i)
  {
    // the color changes a bit after compression and decompression
    EXPECT_LT(fabs(raw2->data[i] - raw.data[i]), 20);
  }

  auto conf = compressed_image_transport::CompressedPublisherConfig::__getDefault__();
  conf.jpeg_quality = 50;
  const auto compressedShifter2 = codecs.encode(raw, "compressed", conf);
  ASSERT_TRUE(compressedShifter2);
  ASSERT_NE("", compressedShifter2->getDataType());
  ASSERT_NO_THROW(compressedShifter2->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed2 = compressedShifter2->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed2->header, raw.header);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", compressed2->format);
  EXPECT_NE(compressed->data, compressed2->data);

#if COMPRESSED_HAS_JPEG_OPTIONS == 1
  conf.jpeg_progressive = true;
  const auto compressedShifter3 = codecs.encode(raw, "compressed", conf);
  ASSERT_TRUE(compressedShifter3);
  ASSERT_NE("", compressedShifter3->getDataType());
  ASSERT_NO_THROW(compressedShifter3->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed3 = compressedShifter2->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed3->header, raw.header);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", compressed3->format);
  EXPECT_NE(compressed->data, compressed3->data);
  EXPECT_NE(compressed2->data, compressed3->data);
#endif
}

TEST(ImageTransportCodecs, CompressedPNG)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "bgr8";
  raw.width = raw.height = 2;
  raw.step = 6;
  raw.data = {0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255};

  auto config = compressed_image_transport::CompressedPublisherConfig::__getDefault__();
  config.format = compressed_image_transport::CompressedPublisher_png;
  const auto compressedShifter = codecs.encode(raw, "compressed", config);
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("bgr8; png compressed bgr8", compressed->format);

  const auto raw2 = codecs.decode(compressedShifter.value(), "compressed");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  for (size_t i = 0; i < raw.data.size(); ++i)
  {
    // PNG compression is lossless
    EXPECT_EQ(raw2->data[i], raw.data[i]);
  }
}

TEST(ImageTransportCodecs, CompressedDepthInv)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "32FC1";
  raw.width = raw.height = 2;
  raw.step = 8;
  float floats[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  auto bytes = reinterpret_cast<uint8_t*>(floats);
  raw.data.resize(16);
  memcpy(&raw.data[0], bytes, 16);

  const auto compressedShifter = codecs.encode(raw, "compressedDepth");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
#if COMPRESSED_DEPTH_HAS_RVL == 1
  EXPECT_EQ("32FC1; compressedDepth png", compressed->format);
#else
  EXPECT_EQ("32FC1; compressedDepth", compressed->format);
#endif

  auto raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  ASSERT_EQ(raw2->data.size(), raw.data.size());
  for (size_t i = 0; i < raw.data.size(); i += 4)
  {
    // the depth changes a bit after lossy compression and decompression
    const auto val = *reinterpret_cast<float*>(&raw.data[i]);
    const auto val2 = *reinterpret_cast<float*>(&raw2->data[i]);
    EXPECT_LT(fabs(val2 - val), 1e-3);
  }
}

TEST(ImageTransportCodecs, CompressedDepthUC)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "16UC1";
  raw.width = raw.height = 2;
  raw.step = 4;
  uint16_t shorts[4] = {1, 2, 3, 4};
  auto bytes = reinterpret_cast<uint8_t*>(shorts);
  raw.data.resize(8);
  memcpy(&raw.data[0], bytes, 8);

  const auto compressedShifter = codecs.encode(raw, "compressedDepth");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
#if COMPRESSED_DEPTH_HAS_RVL == 1
  EXPECT_EQ("16UC1; compressedDepth png", compressed->format);
#else
  EXPECT_EQ("16UC1; compressedDepth", compressed->format);
#endif

  auto raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  ASSERT_EQ(raw2->data.size(), raw.data.size());
  for (size_t i = 0; i < raw.data.size(); i += 2)
  {
    // 16UC1 compression is lossless
    const auto val = *reinterpret_cast<uint16_t*>(&raw.data[i]);
    const auto val2 = *reinterpret_cast<uint16_t*>(&raw2->data[i]);
    EXPECT_EQ(val2, val);
  }
}

#if COMPRESSED_DEPTH_HAS_RVL == 1

TEST(ImageTransportCodecs, CompressedDepthInvRvl)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "32FC1";
  raw.width = raw.height = 2;
  raw.step = 8;
  float floats[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  auto bytes = reinterpret_cast<uint8_t*>(floats);
  raw.data.resize(16);
  memcpy(&raw.data[0], bytes, 16);

  auto config = compressed_depth_image_transport::CompressedDepthPublisherConfig::__getDefault__();
  config.format = compressed_depth_image_transport::CompressedDepthPublisher_rvl;
  const auto compressedShifter = codecs.encode(raw, "compressedDepth", config);
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("32FC1; compressedDepth rvl", compressed->format);

  auto raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  ASSERT_EQ(raw2->data.size(), raw.data.size());
  for (size_t i = 0; i < raw.data.size(); i += 4)
  {
    // the depth changes a bit after lossy compression and decompression
    const auto val = *reinterpret_cast<float*>(&raw.data[i]);
    const auto val2 = *reinterpret_cast<float*>(&raw2->data[i]);
    EXPECT_LT(fabs(val2 - val), 1e-3);
  }
}

TEST(ImageTransportCodecs, CompressedDepthUCRvl)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "16UC1";
  raw.width = raw.height = 2;
  raw.step = 4;
  uint16_t shorts[4] = {1, 2, 3, 4};
  auto bytes = reinterpret_cast<uint8_t*>(shorts);
  raw.data.resize(8);
  memcpy(&raw.data[0], bytes, 8);

  auto config = compressed_depth_image_transport::CompressedDepthPublisherConfig::__getDefault__();
  const auto compressedShifter = codecs.encode(raw, "compressedDepth", config);
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("16UC1; compressedDepth rvl", compressed->format);

  auto raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  ASSERT_EQ(raw2->data.size(), raw.data.size());
  for (size_t i = 0; i < raw.data.size(); i += 2)
  {
    // 16UC1 compression is lossless
    const auto val = *reinterpret_cast<uint16_t*>(&raw.data[i]);
    const auto val2 = *reinterpret_cast<uint16_t*>(&raw2->data[i]);
    EXPECT_EQ(val2, val);
  }
}

#endif

TEST(ImageTransportCodecs, CompressedWrongType)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "32FC1";
  raw.width = raw.height = 2;
  raw.step = 8;
  float floats[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  auto bytes = reinterpret_cast<uint8_t*>(floats);
  raw.data.resize(16);
  memcpy(&raw.data[0], bytes, 16);

  const auto compressedShifter = codecs.encode(raw, "compressedDepth");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);

  auto raw2 = codecs.decode(compressedShifter.value(), "wrong");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

  raw2 = codecs.decode(compressedShifter.value(), "compressed");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

  compressed->format = "bgr8; jpeg compressed bgr8";
  raw2 = codecs.decode(compressedShifter.value(), "compressed");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

  compressed->format = "bgr8; png compressed bgr8";
  raw2 = codecs.decode(compressedShifter.value(), "compressed");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

  compressed->format = "bgr8; compressed bgr8";
  raw2 = codecs.decode(compressedShifter.value(), "compressed");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());
}

TEST(ImageTransportCodecs, CompressedWrongType2)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "bgr8";
  raw.width = raw.height = 2;
  raw.step = 6;
  raw.data = {0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255};

  const auto compressedShifter = codecs.encode(raw, "compressed");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);

  auto raw2 = codecs.decode(compressedShifter.value(), "wrong");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

  raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

#if COMPRESSED_DEPTH_HAS_RVL == 1
  compressed->format = "32FC1; compressedDepth png";
#else
  compressed->format = "32FC1; compressedDepth";
#endif
  raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

#if COMPRESSED_DEPTH_HAS_RVL == 1
  compressed->format = "16UC1; compressedDepth png";
#else
  compressed->format = "16UC1; compressedDepth";
#endif
  raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

#if COMPRESSED_DEPTH_HAS_RVL == 1
  compressed->format = "32FC1; compressedDepth rvl";
#else
  compressed->format = "32FC1; compressedDepth";
#endif
  raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());

#if COMPRESSED_DEPTH_HAS_RVL == 1
  compressed->format = "16UC1; compressedDepth rvl";
#else
  compressed->format = "16UC1; compressedDepth";
#endif
  raw2 = codecs.decode(compressedShifter.value(), "compressedDepth");
  ASSERT_FALSE(raw2);
  EXPECT_NE("", raw2.error());
}

TEST(ImageTransportCodecs, Bag)
{
  ImageTransportCodecs codecs;

  rosbag::Bag rawBag(std::string(TEST_DATA_DIR) + "/raw.bag");
  rosbag::Bag compressedBag(std::string(TEST_DATA_DIR) + "/compressed.bag");
  rosbag::Bag compressedDepthBag(std::string(TEST_DATA_DIR) + "/compressedDepth.bag");

  sensor_msgs::Image bodyMonoRaw;
  sensor_msgs::Image bodyDepthRaw;
  sensor_msgs::Image handColorRaw;
  sensor_msgs::Image handMonoRaw;

  sensor_msgs::CompressedImage bodyMonoCompressed;
  sensor_msgs::CompressedImage bodyDepthCompressed;
  sensor_msgs::CompressedImage handColorCompressed;
  sensor_msgs::CompressedImage handMonoCompressed;

  sensor_msgs::CompressedImage bodyDepthCompressedDepth;

  for (const auto& data : rosbag::View(rawBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::Image>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    msgPtr->is_bigendian = 0;  // Spot driver sets some images to 1 for some reason
    if (data.getTopic() == "/spot/camera/frontleft/image")
      bodyMonoRaw = *msgPtr;
    else if (data.getTopic() == "/spot/camera/hand_color/image")
      handColorRaw = *msgPtr;
    else if (data.getTopic() == "/spot/camera/hand_mono/image")
      handMonoRaw = *msgPtr;
    else if (data.getTopic() == "/spot/depth/frontleft/image")
      bodyDepthRaw = *msgPtr;
  }

  for (const auto& data : rosbag::View(compressedBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::CompressedImage>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    if (data.getTopic() == "/spot/camera/frontleft/image/compressed")
      bodyMonoCompressed = *msgPtr;
    else if (data.getTopic() == "/spot/camera/hand_color/image/compressed")
      handColorCompressed = *msgPtr;
    else if (data.getTopic() == "/spot/camera/hand_mono/image/compressed")
      handMonoCompressed = *msgPtr;
    else if (data.getTopic() == "/spot/depth/frontleft/image/compressed")
      bodyDepthCompressed = *msgPtr;
  }

  for (const auto& data : rosbag::View(compressedDepthBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::CompressedImage>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    // The official encoder passes uninitialized memory to depthParam in case it encodes 16UC1 images, so we zero it out
    if (parseCompressedDepthTransportFormat(msgPtr->format).value().rawEncoding == "16UC1")
    {
      auto depthParam = reinterpret_cast<compressed_depth_image_transport::ConfigHeader*>(
        msgPtr->data.data())->depthParam;
      depthParam[0] = 0;
      depthParam[1] = 0;
    }
    if (data.getTopic() == "/spot/depth/frontleft/image/compressedDepth")
      bodyDepthCompressedDepth = *msgPtr;
  }

  {
    const auto compressedShifter = std::move(codecs.encode(bodyMonoRaw, "compressed"));
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    const auto compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(bodyMonoCompressed, *compressed);

    const auto rawImg = codecs.decodeTyped(bodyMonoCompressed, "compressed");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(bodyMonoRaw.header, rawImg->header);
    EXPECT_EQ(bodyMonoRaw.step, rawImg->step);
    EXPECT_EQ(bodyMonoRaw.width, rawImg->width);
    EXPECT_EQ(bodyMonoRaw.height, rawImg->height);
    EXPECT_EQ(bodyMonoRaw.encoding, rawImg->encoding);
    EXPECT_EQ(bodyMonoRaw.is_bigendian, rawImg->is_bigendian);
    ASSERT_EQ(bodyMonoRaw.data.size(), rawImg->data.size());
    for (size_t i = 0; i < rawImg->data.size(); ++i)
    {
      // the color changes a bit after compression and decompression
      EXPECT_LT(fabs(bodyMonoRaw.data[i] - rawImg->data[i]), 20);
    }
  }

  {
    dynamic_reconfigure::Config config;
    dynamic_reconfigure::StrParameter formatParam;
    formatParam.name = "format";
    formatParam.value = compressed_image_transport::CompressedPublisher_png;
    config.strs.push_back(formatParam);
    dynamic_reconfigure::IntParameter pngLevelParam;
    pngLevelParam.name = "png_level";
    pngLevelParam.value = 6;
    config.ints.push_back(pngLevelParam);
    auto compressedShifter = codecs.encode(bodyDepthRaw, "compressed", config);
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    auto compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(bodyDepthCompressed, *compressed);

    auto rawImg = codecs.decodeTyped(bodyDepthCompressed, "compressed");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(bodyDepthRaw, *rawImg);

    compressedShifter = codecs.encode(bodyDepthRaw, "compressedDepth");
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(bodyDepthCompressedDepth, *compressed);

    rawImg = codecs.decodeTyped(bodyDepthCompressedDepth, "compressedDepth");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(bodyDepthRaw, *rawImg);
  }

  for (size_t i = 0; i < 3; ++i)  // test several iterations
  {
    const auto compressedShifter = codecs.encode(handColorRaw, "compressed");
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    const auto compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(handColorCompressed, *compressed);

    const auto rawImg = codecs.decodeTyped(handColorCompressed, "compressed");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(handColorRaw.header, rawImg->header);
    EXPECT_EQ(handColorRaw.step, rawImg->step);
    EXPECT_EQ(handColorRaw.width, rawImg->width);
    EXPECT_EQ(handColorRaw.height, rawImg->height);
    EXPECT_EQ(handColorRaw.encoding, rawImg->encoding);
    EXPECT_EQ(handColorRaw.is_bigendian, rawImg->is_bigendian);
    ASSERT_EQ(handColorRaw.data.size(), rawImg->data.size());

    // JPEG compression changes this image quite a lot, so we examine the error histogram.
    // It is a 1920x1080 image, so having 7000 pixels with color difference of 20-30 is quite okay.

    size_t err20 = 0, err30 = 0, err40 = 0, err50 = 0, err80 = 0, err = 0;
    for (size_t j = 0; j < rawImg->data.size(); ++j)
    {
      const auto e = fabs(handColorRaw.data[j] - rawImg->data[j]);
      if (e < 20) err20++;
      else if (e < 30) err30++;
      else if (e < 40) err40++;
      else if (e < 50) err50++;
      else if (e < 80) err80++;
      else err++;
    }
    // err20 is ok in any amount
    EXPECT_LT(err30, 7000);
    EXPECT_LT(err40, 500);
    EXPECT_LT(err50, 100);
    EXPECT_LT(err80, 30);
    EXPECT_EQ(0, err);
  }

  {
    const auto compressedShifter = codecs.encode(handMonoRaw, "compressed");
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    const auto compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(handMonoCompressed, *compressed);

    const auto rawImg = codecs.decodeTyped(handMonoCompressed, "compressed");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(handMonoRaw.header, rawImg->header);
    EXPECT_EQ(handMonoRaw.step, rawImg->step);
    EXPECT_EQ(handMonoRaw.width, rawImg->width);
    EXPECT_EQ(handMonoRaw.height, rawImg->height);
    EXPECT_EQ(handMonoRaw.encoding, rawImg->encoding);
    EXPECT_EQ(handMonoRaw.is_bigendian, rawImg->is_bigendian);
    ASSERT_EQ(handMonoRaw.data.size(), rawImg->data.size());
    for (size_t i = 0; i < rawImg->data.size(); ++i)
    {
      // the color changes a bit after compression and decompression
      EXPECT_LT(fabs(handMonoRaw.data[i] - rawImg->data[i]), 30);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
