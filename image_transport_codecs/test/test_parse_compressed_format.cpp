/**
 * \file
 * \brief Unit test for parse_compressed_format.cpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <image_transport_codecs/parse_compressed_format.h>

using namespace image_transport_codecs;  // NOLINT(build/namespaces)

TEST(ParseCompressedFormat, JPEG)
{
  CompressedTransportFormat format {CompressedTransportCompressionFormat::JPEG, "jpeg", "bgr8", "bgr8", 3, 8, true};

  sensor_msgs::Image img;
  img.encoding = "bgr8";

  std::string formatStr = "bgr8; jpeg compressed bgr8";

  EXPECT_EQ(format, extractCompressedTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedTransportFormat(format));
}

TEST(ParseCompressedFormat, PNG_Color)
{
  CompressedTransportFormat format {CompressedTransportCompressionFormat::PNG, "png", "rgb8", "bgr8", 3, 8, true};

  sensor_msgs::Image img;
  img.encoding = "rgb8";

  std::string formatStr = "rgb8; png compressed bgr8";

  EXPECT_EQ(format, extractCompressedTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedTransportFormat(format));
}

TEST(ParseCompressedFormat, PNG_Mono8)
{
  CompressedTransportFormat format {CompressedTransportCompressionFormat::PNG, "png", "mono8", "mono8", 1, 8, false};

  sensor_msgs::Image img;
  img.encoding = "mono8";

  std::string formatStr = "mono8; png compressed ";

  EXPECT_EQ(format, extractCompressedTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedTransportFormat(format));
}

TEST(ParseCompressedFormat, PNG_Mono16)
{
  CompressedTransportFormat format {CompressedTransportCompressionFormat::PNG, "png", "mono16", "mono16", 1, 16, false};

  sensor_msgs::Image img;
  img.encoding = "mono16";

  std::string formatStr = "mono16; png compressed ";

  EXPECT_EQ(format, extractCompressedTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedTransportFormat(format));
}

TEST(ParseCompressedFormat, PNG_16UC1)
{
  CompressedTransportFormat format {CompressedTransportCompressionFormat::PNG, "png", "16UC1", "16UC1", 1, 16, false};

  sensor_msgs::Image img;
  img.encoding = "16UC1";

  std::string formatStr = "16UC1; png compressed ";

  EXPECT_EQ(format, extractCompressedTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedTransportFormat(format));
}

TEST(ParseCompressedFormat, JPEG_Short)
{
  std::string formatStr = "jpeg";
  CompressedTransportFormat format {CompressedTransportCompressionFormat::JPEG, "jpeg", "bgr8", "bgr8", 3, 8, true};
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
}

TEST(ParseCompressedFormat, PNG_Short)
{
  std::string formatStr = "png";
  CompressedTransportFormat format {CompressedTransportCompressionFormat::PNG, "png", "bgr8", "bgr8", 3, 8, true};
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
}

TEST(ParseCompressedFormat, JPEG_C_API)
{
  CompressedTransportFormat format {CompressedTransportCompressionFormat::JPEG, "jpeg", "bgr8", "bgr8", 3, 8, true};

  sensor_msgs::Image img;
  img.encoding = "bgr8";

  std::string formatStr = "bgr8; jpeg compressed bgr8";

  static char* compressedEncoding;
  static char* errorString;
  int numChannels;
  int bitDepth;
  bool isColor;
  auto success = ::extractCompressedTransportFormat(img.encoding.c_str(), format.formatString.c_str(),
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor,
    [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_TRUE(success);
  EXPECT_EQ(format.compressedEncoding, compressedEncoding);
  EXPECT_EQ(format.numChannels, numChannels);
  EXPECT_EQ(format.bitDepth, bitDepth);
  EXPECT_EQ(format.isColor, isColor);
  EXPECT_EQ(nullptr, errorString);
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;

  static char* compressionFormat;
  static char* rawEncoding;
  bitDepth = numChannels = 0;
  isColor = false;
  success = ::parseCompressedTransportFormat(formatStr.c_str(),
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor, [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_TRUE(success);
  EXPECT_EQ(format.formatString, compressionFormat);
  EXPECT_EQ(format.rawEncoding, rawEncoding);
  EXPECT_EQ(format.compressedEncoding, compressedEncoding);
  EXPECT_EQ(format.numChannels, numChannels);
  EXPECT_EQ(format.bitDepth, bitDepth);
  EXPECT_EQ(format.isColor, isColor);
  EXPECT_EQ(nullptr, errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;

  static char* formatOutput;
  success = ::makeCompressedTransportFormat(format.formatString.c_str(), format.rawEncoding.c_str(),
    format.compressedEncoding.c_str(), format.numChannels, format.bitDepth, format.isColor,
    [](size_t s) -> void* {return formatOutput = new char[s];},
    [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_TRUE(success);
  EXPECT_EQ(formatStr, formatOutput);
  EXPECT_EQ(nullptr, errorString);
  delete formatOutput; formatOutput = nullptr;
  delete errorString; errorString = nullptr;

  bitDepth = numChannels = 0;
  isColor = false;
  success = ::parseCompressedTransportFormat("invalid",
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor, [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_FALSE(success);
  EXPECT_EQ(nullptr, compressionFormat);
  EXPECT_EQ(nullptr, rawEncoding);
  EXPECT_EQ(nullptr, compressedEncoding);
  EXPECT_EQ(0, numChannels);
  EXPECT_EQ(0, bitDepth);
  EXPECT_EQ(false, isColor);
  EXPECT_STREQ("compressed transport format 'invalid' is invalid.", errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;
}

TEST(ParseCompressedFormat, Empty)
{
  std::string formatStr = "";
  CompressedTransportFormat format {CompressedTransportCompressionFormat::JPEG, "jpeg", "bgr8", "bgr8", 3, 8, true};
  EXPECT_EQ(format, parseCompressedTransportFormat(formatStr).value());
}

TEST(ParseCompressedFormat, Invalid)
{
  std::string formatStr = "invalid";
  EXPECT_FALSE(parseCompressedTransportFormat(formatStr));
}

TEST(ParseCompressedDepthFormat, PNG_16UC1)
{
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::PNG, "png", "16UC1", 16};

  sensor_msgs::Image img;
  img.encoding = "16UC1";

#if COMPRESSED_DEPTH_HAS_RVL == 1
  std::string formatStr = "16UC1; compressedDepth png";
#else
  std::string formatStr = "16UC1; compressedDepth";
#endif

  EXPECT_EQ(format, extractCompressedDepthTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedDepthTransportFormat(format));
}

TEST(ParseCompressedDepthFormat, PNG_32FC1)
{
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::PNG, "png", "32FC1", 32};

  sensor_msgs::Image img;
  img.encoding = "32FC1";

#if COMPRESSED_DEPTH_HAS_RVL == 1
  std::string formatStr = "32FC1; compressedDepth png";
#else
  std::string formatStr = "32FC1; compressedDepth";
#endif

  EXPECT_EQ(format, extractCompressedDepthTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
  EXPECT_EQ(formatStr, makeCompressedDepthTransportFormat(format));
}

TEST(ParseCompressedDepthFormat, RVL_16UC1)
{
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::RVL, "rvl", "16UC1", 16};

  sensor_msgs::Image img;
  img.encoding = "16UC1";

  std::string formatStr = "16UC1; compressedDepth rvl";

  EXPECT_EQ(format, extractCompressedDepthTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
#if COMPRESSED_DEPTH_HAS_RVL == 1
  EXPECT_EQ(formatStr, makeCompressedDepthTransportFormat(format));
#endif
}

TEST(ParseCompressedDepthFormat, RVL_32FC1)
{
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::RVL, "rvl", "32FC1", 32};

  sensor_msgs::Image img;
  img.encoding = "32FC1";

  std::string formatStr = "32FC1; compressedDepth rvl";

  EXPECT_EQ(format, extractCompressedDepthTransportFormat(img, format.format));
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
#if COMPRESSED_DEPTH_HAS_RVL == 1
  EXPECT_EQ(formatStr, makeCompressedDepthTransportFormat(format));
#endif
}

TEST(ParseCompressedDepthFormat, PNG_Short)
{
  std::string formatStr = "png";
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::PNG, "png", "16UC1", 16};
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
}

TEST(ParseCompressedDepthFormat, RVL_Short)
{
  std::string formatStr = "rvl";
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::RVL, "rvl", "16UC1", 16};
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
}

TEST(ParseCompressedDepthFormat, PNG_16UC1_C_API)
{
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::PNG, "png", "16UC1", 16};

  sensor_msgs::Image img;
  img.encoding = "16UC1";

#if COMPRESSED_DEPTH_HAS_RVL == 1
  std::string formatStr = "16UC1; compressedDepth png";
#else
  std::string formatStr = "16UC1; compressedDepth";
#endif

  static char* errorString;
  int bitDepth;
  auto success = ::extractCompressedDepthTransportFormat(img.encoding.c_str(), format.formatString.c_str(),
    bitDepth, [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_TRUE(success);
  EXPECT_EQ(format.bitDepth, bitDepth);
  EXPECT_EQ(nullptr, errorString);
  delete errorString; errorString = nullptr;

  static char* compressionFormat;
  static char* rawEncoding;
  bitDepth = 0;
  success = ::parseCompressedDepthTransportFormat(formatStr.c_str(),
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    bitDepth, [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_TRUE(success);
  EXPECT_EQ(format.formatString, compressionFormat);
  EXPECT_EQ(format.rawEncoding, rawEncoding);
  EXPECT_EQ(format.bitDepth, bitDepth);
  EXPECT_EQ(nullptr, errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete errorString; errorString = nullptr;

  static char* formatOutput;
  success = ::makeCompressedDepthTransportFormat(format.formatString.c_str(), format.rawEncoding.c_str(),
    format.bitDepth, [](size_t s) -> void* {return formatOutput = new char[s];},
    [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_TRUE(success);
  EXPECT_EQ(formatStr, formatOutput);
  EXPECT_EQ(nullptr, errorString);
  delete formatOutput; formatOutput = nullptr;
  delete errorString; errorString = nullptr;

  bitDepth = 0;
  success = ::parseCompressedDepthTransportFormat("invalid",
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    bitDepth, [](size_t s) -> void* {return errorString = new char[s];});
  ASSERT_FALSE(success);
  EXPECT_EQ(nullptr, compressionFormat);
  EXPECT_EQ(nullptr, rawEncoding);
  EXPECT_EQ(0, bitDepth);
  EXPECT_STREQ("compressedDepth transport format 'invalid' is invalid.", errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete errorString; errorString = nullptr;
}

TEST(ParseCompressedDepthFormat, Empty)
{
  std::string formatStr = "";
  CompressedDepthTransportFormat format {CompressedDepthTransportCompressionFormat::PNG, "png", "16UC1", 16};
  EXPECT_EQ(format, parseCompressedDepthTransportFormat(formatStr).value());
}

TEST(ParseCompressedDepthFormat, Invalid)
{
  std::string formatStr = "invalid";
  EXPECT_FALSE(parseCompressedDepthTransportFormat(formatStr));
}

TEST(GuessAnyCompressedFormat, Bag)
{
  rosbag::Bag compressedBag(std::string(TEST_DATA_DIR) + "/compressed.bag");
  rosbag::Bag compressedDepthBag(std::string(TEST_DATA_DIR) + "/compressedDepth.bag");

  sensor_msgs::CompressedImage bodyMonoCompressed;
  sensor_msgs::CompressedImage bodyDepthCompressed;
  sensor_msgs::CompressedImage handColorCompressed;
  sensor_msgs::CompressedImage handMonoCompressed;

  sensor_msgs::CompressedImage bodyDepthCompressedDepth;

  for (const auto& data : rosbag::View(compressedBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::CompressedImage>();
    ASSERT_NE(nullptr, msgPtr);
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
    if (data.getTopic() == "/spot/depth/frontleft/image/compressedDepth")
      bodyDepthCompressedDepth = *msgPtr;
  }

  auto result = image_transport_codecs::guessAnyCompressedImageTransportFormat(bodyMonoCompressed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->first);
  EXPECT_FALSE(result->second);
  EXPECT_EQ(image_transport_codecs::CompressedTransportCompressionFormat::JPEG, result->first->format);
  EXPECT_EQ(8u, result->first->bitDepth);
  EXPECT_FALSE(result->first->isColor);
  EXPECT_EQ(1u, result->first->numChannels);
  EXPECT_EQ("mono8", result->first->compressedEncoding);
  EXPECT_EQ("mono8", result->first->rawEncoding);

  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(handColorCompressed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->first);
  EXPECT_FALSE(result->second);
  EXPECT_EQ(image_transport_codecs::CompressedTransportCompressionFormat::JPEG, result->first->format);
  EXPECT_EQ(8u, result->first->bitDepth);
  EXPECT_TRUE(result->first->isColor);
  EXPECT_EQ(3u, result->first->numChannels);
  EXPECT_EQ("bgr8", result->first->compressedEncoding);
  EXPECT_EQ("rgb8", result->first->rawEncoding);

  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(handMonoCompressed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->first);
  EXPECT_FALSE(result->second);
  EXPECT_EQ(image_transport_codecs::CompressedTransportCompressionFormat::JPEG, result->first->format);
  EXPECT_EQ(8u, result->first->bitDepth);
  EXPECT_FALSE(result->first->isColor);
  EXPECT_EQ(1u, result->first->numChannels);
  EXPECT_EQ("mono8", result->first->compressedEncoding);
  EXPECT_EQ("mono8", result->first->rawEncoding);

  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(bodyDepthCompressed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->first);
  EXPECT_FALSE(result->second);
  EXPECT_EQ(image_transport_codecs::CompressedTransportCompressionFormat::PNG, result->first->format);
  EXPECT_EQ(16u, result->first->bitDepth);
  EXPECT_FALSE(result->first->isColor);
  EXPECT_EQ(1u, result->first->numChannels);
  EXPECT_EQ("16UC1", result->first->compressedEncoding);
  EXPECT_EQ("16UC1", result->first->rawEncoding);

  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(bodyDepthCompressedDepth);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->second);
  EXPECT_FALSE(result->first);
  EXPECT_EQ(image_transport_codecs::CompressedDepthTransportCompressionFormat::PNG, result->second->format);
  EXPECT_EQ(16u, result->second->bitDepth);
  EXPECT_EQ("16UC1", result->second->rawEncoding);
}

TEST(GuessAnyCompressedFormat, EmptyImage)
{
  sensor_msgs::CompressedImage image;

  image.format = "jpeg";
  auto result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->first);
  EXPECT_FALSE(result->second);
  EXPECT_EQ(image_transport_codecs::CompressedTransportCompressionFormat::JPEG, result->first->format);

  image.format = "rvl";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->second);
  EXPECT_FALSE(result->first);
  EXPECT_EQ(image_transport_codecs::CompressedDepthTransportCompressionFormat::RVL, result->second->format);

  image.format = "png";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  // without an actual image content, PNG cannot be decided
  EXPECT_FALSE(result);

  image.format = "png";
  image.data.resize(64);
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  // if image data do not start with PNG header, it is compressedDepth
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->second);
  EXPECT_FALSE(result->first);
  EXPECT_EQ(image_transport_codecs::CompressedDepthTransportCompressionFormat::PNG, result->second->format);

  image.format = "png";
  image.data = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
  image.data.resize(64);
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  // if image data do start with PNG header, it is compressed
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->first);
  EXPECT_FALSE(result->second);
  EXPECT_EQ(image_transport_codecs::CompressedTransportCompressionFormat::PNG, result->first->format);
}

TEST(GuessAnyCompressedFormat, Wrong)
{
  sensor_msgs::CompressedImage image;

  image.format = "";
  auto result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_FALSE(result);
  EXPECT_NE("", result.error());

  image.format = "wrong";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_FALSE(result);
  EXPECT_NE("", result.error());

  image.format = "jpeg compressed wrong";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_FALSE(result);
  EXPECT_NE("", result.error());

  image.format = "png compressed wrong";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_FALSE(result);
  EXPECT_NE("", result.error());

  image.format = "png compressedDepth wrong";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_FALSE(result);
  EXPECT_NE("", result.error());

  image.format = "rvl compressedDepth wrong";
  result = image_transport_codecs::guessAnyCompressedImageTransportFormat(image);
  ASSERT_FALSE(result);
  EXPECT_NE("", result.error());
}

TEST(GuessAnyCompressedFormat, C_API)
{
  std::vector<uint8_t> data;
  static char* compressionFormat;
  static char* rawEncoding;
  static char* compressedEncoding;
  static char* errorString;
  bool isCompressedDepth = false;
  int numChannels = 0;
  int bitDepth = 0;
  bool isColor = false;
  bool success;

  success = guessAnyCompressedImageTransportFormat("jpeg", nullptr, isCompressedDepth,
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor,
    [](size_t s) -> void* {return errorString = new char[s];});

  ASSERT_TRUE(success);
  EXPECT_FALSE(isCompressedDepth);
  EXPECT_STREQ("jpeg", compressionFormat);
  EXPECT_STREQ("bgr8", rawEncoding);
  EXPECT_STREQ("bgr8", compressedEncoding);
  EXPECT_EQ(3, numChannels);
  EXPECT_EQ(8, bitDepth);
  EXPECT_TRUE(isColor);
  EXPECT_EQ(nullptr, errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;

  success = guessAnyCompressedImageTransportFormat("rvl", nullptr, isCompressedDepth,
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor,
    [](size_t s) -> void* {return errorString = new char[s];});

  ASSERT_TRUE(success);
  EXPECT_TRUE(isCompressedDepth);
  EXPECT_STREQ("rvl", compressionFormat);
  EXPECT_STREQ("16UC1", rawEncoding);
  EXPECT_EQ(nullptr, compressedEncoding);
  EXPECT_EQ(1, numChannels);
  EXPECT_EQ(16, bitDepth);
  EXPECT_FALSE(isColor);
  EXPECT_EQ(nullptr, errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;

  // without an actual image content, PNG cannot be decided

  success = guessAnyCompressedImageTransportFormat("png", nullptr, isCompressedDepth,
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor,
    [](size_t s) -> void* {return errorString = new char[s];});

  ASSERT_FALSE(success);
  ASSERT_NE(nullptr, errorString);
  EXPECT_STRNE("", errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;

  // 12 zeros at the beginning are for the compressedDepth config header, then the PNG starts
  data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
  data.resize(64);

  success = guessAnyCompressedImageTransportFormat("png", data.data(), isCompressedDepth,
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor,
    [](size_t s) -> void* {return errorString = new char[s];});

  ASSERT_TRUE(success);
  EXPECT_TRUE(isCompressedDepth);
  EXPECT_STREQ("png", compressionFormat);
  EXPECT_STREQ("16UC1", rawEncoding);
  EXPECT_EQ(nullptr, compressedEncoding);
  EXPECT_EQ(1, numChannels);
  EXPECT_EQ(16, bitDepth);
  EXPECT_FALSE(isColor);
  EXPECT_EQ(nullptr, errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;

  // directly start with PNG header
  data = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
  data.resize(64);

  success = guessAnyCompressedImageTransportFormat("png", data.data(), isCompressedDepth,
    [](size_t s) -> void* {return compressionFormat = new char[s];},
    [](size_t s) -> void* {return rawEncoding = new char[s];},
    [](size_t s) -> void* {return compressedEncoding = new char[s];},
    numChannels, bitDepth, isColor,
    [](size_t s) -> void* {return errorString = new char[s];});

  ASSERT_TRUE(success);
  EXPECT_FALSE(isCompressedDepth);
  EXPECT_STREQ("png", compressionFormat);
  EXPECT_STREQ("bgr8", rawEncoding);
  EXPECT_STREQ("bgr8", compressedEncoding);
  EXPECT_EQ(3, numChannels);
  EXPECT_EQ(8, bitDepth);
  EXPECT_TRUE(isColor);
  EXPECT_EQ(nullptr, errorString);
  delete compressionFormat; compressionFormat = nullptr;
  delete rawEncoding; rawEncoding = nullptr;
  delete compressedEncoding; compressedEncoding = nullptr;
  delete errorString; errorString = nullptr;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
