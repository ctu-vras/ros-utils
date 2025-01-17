/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// This file is heavily based on the BSD-3-licensed compressed_depth_image_transport package:
// https://github.com/ros-perception/image_transport_plugins/tree/noetic-devel/compressed_depth_image_transport/src
// The basic algorithms from upstream are unchanged, although the code has undergone cosmetic and API-design changes.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>

// If OpenCV3
#ifndef CV_VERSION_EPOCH
#include <opencv2/imgcodecs.hpp>

// If OpenCV4
#if CV_VERSION_MAJOR > 3
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif
#endif


#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_depth_image_transport/CompressedDepthPublisherConfig.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <image_transport_codecs/parse_compressed_format.h>
#include <image_transport_codecs/codecs/compressed_depth_codec.h>

namespace image_transport_codecs
{

namespace enc = sensor_msgs::image_encodings;

class RvlCodec
{
public:
  RvlCodec() = default;
  void ReadSize(const unsigned char* compressed, uint32_t& rows, uint32_t& cols);
  // Compress input data into output. The size of output can be equal to
  // (1.5 * numPixels + 4) in the worst case.
  int CompressRVL(const unsigned short* input, unsigned char* output, int numPixels);  // NOLINT(runtime/int)
  // Decompress input data into output. The size of output must be
  // equal to numPixels.
  void DecompressRVL(const unsigned char* input, unsigned short* output, int numPixels);  // NOLINT(runtime/int)

private:
  void EncodeVLE(int value);
  int DecodeVLE();

  int *buffer_;
  int *pBuffer_;
  int word_;
  int nibblesWritten_;
};


CompressedDepthCodec::CompressedDepthCodec(const cras::LogHelperPtr& logHelper) : ImageTransportCodec(logHelper)
{
}

CompressedDepthCodec::~CompressedDepthCodec() = default;

CompressedDepthCodec::EncodeResult CompressedDepthCodec::encode(const sensor_msgs::Image& raw,
  const compressed_depth_image_transport::CompressedDepthPublisherConfig& config) const
{
  std::string format = "png";
#if COMPRESSED_DEPTH_HAS_RVL == 1
  format = config.format;
#endif

  return encodeCompressedDepthImage(
    raw, format, config.depth_max, config.depth_quantization, config.png_level);
}

ImageTransportCodec::DecodeResult CompressedDepthCodec::decode(const sensor_msgs::CompressedImage& compressed) const
{
  return decodeCompressedDepthImage(compressed);
}

std::string CompressedDepthCodec::getTransportName() const
{
  return "compressedDepth";
}

ImageTransportCodec::EncodeResult CompressedDepthCodec::encode(const sensor_msgs::Image& raw,
                                                               const dynamic_reconfigure::Config& config) const
{
  auto codecConfig = compressed_depth_image_transport::CompressedDepthPublisherConfig::__getDefault__();
  if (!codecConfig.__fromMessage__(*const_cast<dynamic_reconfigure::Config*>(&config)))
    return cras::make_unexpected("Invalid config passed to compressed transport encoder.");

  const auto compressedImage = this->encode(raw, codecConfig);
  if (!compressedImage)
    return cras::make_unexpected(compressedImage.error());

  cras::ShapeShifter compressed;
  cras::msgToShapeShifter(compressedImage.value(), compressed);
  return compressed;
}

ImageTransportCodec::DecodeResult CompressedDepthCodec::decode(const topic_tools::ShapeShifter& compressed,
                                                               const dynamic_reconfigure::Config& config) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressedDepth decoder: %s.", e.what()));
  }

  return this->decode(*compressedImage);
}

compressed_depth_image_transport::ConfigHeader
CompressedDepthCodec::getCompressionConfig(const sensor_msgs::CompressedImage& compressed) const
{
  compressed_depth_image_transport::ConfigHeader compressionConfig {};
  compressionConfig.format = compressed_depth_image_transport::UNDEFINED;

  if (compressed.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
  {
    // Read compression type from stream
    memcpy(&compressionConfig, &compressed.data[0], sizeof(compressionConfig));
  }

  return compressionConfig;
}

ImageTransportCodec::DecodeResult CompressedDepthCodec::decodeCompressedDepthImage(
  const sensor_msgs::CompressedImage& compressed) const
{
  if (compressed.data.size() <= sizeof(compressed_depth_image_transport::ConfigHeader))
    return cras::make_unexpected("The data passed to compressedDepth decoder are too small to represent an image.");

  // Parse format field
  const auto format = parseCompressedDepthTransportFormat(compressed.format);
  if (!format)
    return cras::make_unexpected("Invalid compressed decoder config: " + format.error());

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = compressed.header;
  cv_ptr->encoding = format->rawEncoding;

  // Decode message data
  const auto compressionConfig = this->getCompressionConfig(compressed);
  if (compressionConfig.format != compressed_depth_image_transport::UNDEFINED)
  {
    // Get compressed image data
    const std::vector<uint8_t> imageData(compressed.data.begin() + sizeof(compressionConfig), compressed.data.end());

    if (format->bitDepth == 32)
    {
      cv::Mat decompressed;
      if (format->format == CompressedDepthTransportCompressionFormat::PNG) {
        try
        {
          // Decode image data
          decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        }
        catch (cv::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "OpenCV error occurred while decoding '%s' image: %s (%s).",
            compressed.format.c_str(), cvErrorStr(e.code), e.err.c_str()));
        }
      }
      else if (format->format == CompressedDepthTransportCompressionFormat::RVL)
      {
        decompressed = this->decodeRVL(imageData);
      }
      else
      {
        return cras::make_unexpected(cras::format("Invalid encoding format %i.", static_cast<int>(format->format)));
      }

      if (decompressed.rows > 0 && decompressed.cols > 0)
      {
        cv_ptr->image = this->fromInvDepth(decompressed, compressionConfig);
        return *cv_ptr->toImageMsg();
      }
      return cras::make_unexpected("Decoding compressedDepth image yielded a zero-size result.");
    }
    else
    {
      // Decode raw image
      if (format->format == CompressedDepthTransportCompressionFormat::PNG) {
        try
        {
          cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        }
        catch (cv::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "OpenCV error occurred while decoding '%s' image: %s (%s).",
            compressed.format.c_str(), cvErrorStr(e.code), e.err.c_str()));
        }
      }
      else if (format->format == CompressedDepthTransportCompressionFormat::RVL)
      {
        cv_ptr->image = this->decodeRVL(imageData);
      }
      else
      {
        return cras::make_unexpected(cras::format("Invalid encoding format %i.", static_cast<int>(format->format)));
      }

      if (cv_ptr->image.rows > 0 && cv_ptr->image.cols > 0)
        return *cv_ptr->toImageMsg();

      return cras::make_unexpected("Decoding compressedDepth image yielded a zero-size result.");
    }
  }
  return cras::make_unexpected("compressedDepth decoder has not found compression config in the image.");
}

CompressedDepthCodec::EncodeResult CompressedDepthCodec::encodeCompressedDepthImage(const sensor_msgs::Image& raw,
  const std::string& compression_format, double depth_max, double depth_quantization, int png_level) const
{
  const auto format = extractCompressedDepthTransportFormat(raw, compression_format);
  if (!format)
    return cras::make_unexpected("Invalid compressedDepth encoder config: " + format.error());

  sensor_msgs::CompressedImage compressed;
  compressed.header = raw.header;
  compressed.format = makeCompressedDepthTransportFormat(format.value());

  // Compression settings
  std::vector<int> params;
  params.resize(3, 0);

  int numChannels = enc::numChannels(raw.encoding);

  // Image compression configuration
  compressed_depth_image_transport::ConfigHeader compressionConfig {};

  // Check input format
  params[0] = cv::IMWRITE_PNG_COMPRESSION;
  params[1] = png_level;

  // OpenCV-ROS bridge
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(raw);
  }
  catch (cv_bridge::Exception& e)
  {
    return cras::make_unexpected(cras::format(
      "cv_bridge error occurred while encoding %ix%i %s image as %s: %s.",
      raw.width, raw.height, raw.encoding.c_str(), compression_format.c_str(), e.what()));
  }

  const cv::Mat& depthImg = cv_ptr->image;

  if ((format->bitDepth == 32) && (numChannels == 1))
  {
    if (depthImg.rows > 0 && depthImg.cols > 0)
    {
      // Allocate matrix for inverse depth (disparity) coding
      cv::Mat invDepthImg = this->toInvDepth(depthImg, depth_max, depth_quantization, compressionConfig);

      // Compress quantized disparity image
      if (format->format == CompressedDepthTransportCompressionFormat::PNG) {
        try
        {
          if (!cv::imencode(".png", invDepthImg, compressed.data, params))
            // The bool return value of imencode is undocumented and it seems it cannot happen, but for completeness...
            return cras::make_unexpected(cras::format(
              "Unknown OpenCV error occurred while encoding %ix%i %s image as %s.",
              raw.width, raw.height, raw.encoding.c_str(), compression_format.c_str()));
        }
        catch (cv::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "OpenCV error occurred while encoding %ix%i %s image as %s: %s (%s).",
            raw.width, raw.height, raw.encoding.c_str(), compression_format.c_str(),
            cvErrorStr(e.code), e.err.c_str()));
        }
      }
      else if (format->format == CompressedDepthTransportCompressionFormat::RVL)
      {
        this->encodeRVL(invDepthImg, compressed.data);
      }
      else
      {
        return cras::make_unexpected(cras::format("Invalid encoding format %i.", static_cast<int>(format->format)));
      }
    }
  }
  // Raw depth map compression
  else if ((format->bitDepth == 16) && (numChannels == 1))
  {
    if (depthImg.rows > 0 && depthImg.cols > 0)
    {
      unsigned short depthMaxUShort = static_cast<unsigned short>(depth_max * 1000.0f);  // NOLINT(runtime/int)

      // Matrix iterators
      cv::MatIterator_<unsigned short> itDepthImg = cv_ptr->image.begin<unsigned short>();  // NOLINT(runtime/int)
      cv::MatIterator_<unsigned short> itDepthImg_end = cv_ptr->image.end<unsigned short>();  // NOLINT(runtime/int)

      // Max depth filter
      for (; itDepthImg != itDepthImg_end; ++itDepthImg)
      {
        if (*itDepthImg > depthMaxUShort)
          *itDepthImg = 0;
      }

      // Compress raw depth image
      if (format->format == CompressedDepthTransportCompressionFormat::PNG) {
        try
        {
          if (!cv::imencode(".png", cv_ptr->image, compressed.data, params))
            // The bool return value of imencode is undocumented and it seems it cannot happen, but for completeness...
            return cras::make_unexpected(cras::format(
              "Unknown OpenCV error occurred while encoding %ix%i %s image as %s.",
              raw.width, raw.height, raw.encoding.c_str(), compression_format.c_str()));
        }
        catch (cv::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "OpenCV error occurred while encoding %ix%i %s image as %s: %s (%s).",
            raw.width, raw.height, raw.encoding.c_str(), compression_format.c_str(),
            cvErrorStr(e.code), e.err.c_str()));
        }
      }
      else if (format->format == CompressedDepthTransportCompressionFormat::RVL)
      {
        this->encodeRVL(cv_ptr->image, compressed.data);
      }
    }
  }
  else
  {
    return cras::make_unexpected("Error encoding " + raw.encoding +
      " image as compressedDepth: only 16-bit and 32-bit images are supported.");
  }

  if (!compressed.data.empty())
  {
    // Add configuration to binary output
    compressed.data.insert(compressed.data.begin(), sizeof(compressed_depth_image_transport::ConfigHeader), 0);
    memcpy(&compressed.data[0], &compressionConfig, sizeof(compressed_depth_image_transport::ConfigHeader));

    return compressed;
  }

  return cras::make_unexpected("Unknown error in compressedDepth encoder.");
}

cv::Mat CompressedDepthCodec::decodeRVL(const std::vector<uint8_t>& compressed) const
{
  const unsigned char *buffer = compressed.data();
  uint32_t cols, rows;
  RvlCodec rvl;

  rvl.ReadSize(buffer, rows, cols);
  if (rows == 0 || cols == 0)
  {
    CRAS_ERROR_THROTTLE(1.0, "Received malformed RVL-encoded image. Size %ix%i contains zero.", cols, rows);
    return cv::Mat(0, 0, CV_16UC1);
  }

  // Sanity check - the best compression ratio is 4x; we leave some buffer, so we check whether the output image would
  // not be more than 100x larger than the compressed one. If it is, we probably received corrupted data.
  // The condition should be "numPixels * 2 > compressed.size() * 100" (because each pixel is 2 bytes), but to prevent
  // overflow, we have canceled out the *2 from both sides of the inequality.
  const auto numPixels = static_cast<uint64_t>(rows) * cols;
  if (numPixels > std::numeric_limits<int>::max() || numPixels > static_cast<uint64_t>(compressed.size()) * 50)
  {
    CRAS_ERROR_THROTTLE(1.0, "Received malformed RVL-encoded image. It reports size %ux%u.", cols, rows);
    return cv::Mat(0, 0, CV_16UC1);
  }

  cv::Mat decompressed(rows, cols, CV_16UC1);
  rvl.DecompressRVL(
    &buffer[8], decompressed.ptr<unsigned short>(), static_cast<int>(numPixels));  // NOLINT(runtime/int)
  return decompressed;
}

cv::Mat CompressedDepthCodec::fromInvDepth(
  const cv::Mat& invDepthImg, const compressed_depth_image_transport::ConfigHeader& compressionConfig) const
{
  cv::Mat depthImg = cv::Mat(invDepthImg.rows, invDepthImg.cols, CV_32FC1);

  // Depth map decoding
  float depthQuantA, depthQuantB;

  // Read quantization parameters
  depthQuantA = compressionConfig.depthParam[0];
  depthQuantB = compressionConfig.depthParam[1];

  // Depth conversion
  cv::MatIterator_<float> itDepthImg = depthImg.begin<float>();
  cv::MatIterator_<float> itDepthImg_end = depthImg.end<float>();
  cv::MatConstIterator_<unsigned short> itInvDepthImg = invDepthImg.begin<unsigned short>();  // NOLINT(runtime/int)
  cv::MatConstIterator_<unsigned short> itInvDepthImg_end = invDepthImg.end<unsigned short>();  // NOLINT(runtime/int)

  for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
  {
    // check for NaN & max depth
    if (*itInvDepthImg)
    {
      *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);  // NOLINT(readability/casting)
    }
    else
    {
      *itDepthImg = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return depthImg;
}

cv::Mat CompressedDepthCodec::toInvDepth(const cv::Mat& depthImg, double depth_max, double depth_quantization,
  compressed_depth_image_transport::ConfigHeader& compressionConfig) const
{
  // Allocate matrix for inverse depth (disparity) coding
  cv::Mat invDepthImg(depthImg.rows, depthImg.cols, CV_16UC1);

  const auto depthZ0 = static_cast<float>(depth_quantization);
  const auto depthMax = static_cast<float>(depth_max);

  // Inverse depth quantization parameters
  float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
  float depthQuantB = 1.0f - depthQuantA / depthMax;

  // Matrix iterators
  cv::MatConstIterator_<float> itDepthImg = depthImg.begin<float>();
  cv::MatConstIterator_<float> itDepthImg_end = depthImg.end<float>();
  cv::MatIterator_<unsigned short> itInvDepthImg = invDepthImg.begin<unsigned short>();  // NOLINT(runtime/int)
  cv::MatIterator_<unsigned short> itInvDepthImg_end = invDepthImg.end<unsigned short>();  // NOLINT(runtime/int)

  // Quantization
  for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
  {
    // check for NaN & max depth
    if (*itDepthImg < depthMax)
    {
      *itInvDepthImg = static_cast<unsigned short>(depthQuantA / *itDepthImg + depthQuantB);  // NOLINT(runtime/int)
    }
    else
    {
      *itInvDepthImg = 0;
    }
  }

  // Add coding parameters to header
  compressionConfig.format = compressed_depth_image_transport::INV_DEPTH;
  compressionConfig.depthParam[0] = depthQuantA;
  compressionConfig.depthParam[1] = depthQuantB;

  return invDepthImg;
}

ImageTransportCodec::GetCompressedContentResult CompressedDepthCodec::getCompressedImageContent(
  const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressedDepth decoder: %s.", e.what()));
  }
  return this->getCompressedImageContent(*compressedImage, matchFormat);
}

ImageTransportCodec::GetCompressedContentResult CompressedDepthCodec::getCompressedImageContent(
  const sensor_msgs::CompressedImage& compressed, const std::string& matchFormat) const
{
  const auto format = parseCompressedDepthTransportFormat(compressed.format);
  if (!format)
    return cras::make_unexpected("Invalid compressedDepth format: " + format.error());

  if (!matchFormat.empty() && cras::toLower(format->formatString) != cras::toLower(matchFormat))
    return cras::nullopt;

  const auto headerLen = sizeof(compressed_depth_image_transport::ConfigHeader);

  if (compressed.data.size() < headerLen)
    return cras::nullopt;

  return CompressedImageContent{format->formatString, {compressed.data.begin() + headerLen, compressed.data.end()}};
}

void CompressedDepthCodec::encodeRVL(const cv::Mat& depthImg16UC1, std::vector<uint8_t>& compressed) const
{
  int numPixels = depthImg16UC1.rows * depthImg16UC1.cols;
  // In the worst case, RVL compression results in ~1.5x larger data.
  compressed.resize(3 * numPixels + 12);
  uint32_t cols = depthImg16UC1.cols;
  uint32_t rows = depthImg16UC1.rows;
  memcpy(&compressed[0], &cols, 4);
  memcpy(&compressed[4], &rows, 4);
  RvlCodec rvl;
  int compressedSize =
    rvl.CompressRVL(depthImg16UC1.ptr<unsigned short>(), &compressed[8], numPixels);  // NOLINT(runtime/int)
  compressed.resize(8 + compressedSize);
}

void RvlCodec::ReadSize(const unsigned char* compressed, uint32_t& rows, uint32_t & cols)
{
  memcpy(&cols, &compressed[0], 4);
  memcpy(&rows, &compressed[4], 4);
}

void RvlCodec::EncodeVLE(int value)
{
  do {
    int nibble = value & 0x7;        // lower 3 bits
    if (value >>= 3) nibble |= 0x8;  // more to come
    word_ <<= 4;
    word_ |= nibble;
    if (++nibblesWritten_ == 8)  // output word
    {
      *pBuffer_++ = word_;
      nibblesWritten_ = 0;
      word_ = 0;
    }
  } while (value);
}

int RvlCodec::DecodeVLE()
{
  unsigned int nibble;
  int value = 0, bits = 29;
  do {
    if (!nibblesWritten_) {
      word_ = *pBuffer_++;  // load word
      nibblesWritten_ = 8;
    }
    nibble = word_ & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word_ <<= 4;
    nibblesWritten_--;
    bits -= 3;
  } while (nibble & 0x80000000);
  return value;
}

int RvlCodec::CompressRVL(const unsigned short* input, unsigned char* output, int numPixels)  // NOLINT(runtime/int)
{
  buffer_ = pBuffer_ = (int*)output;  // NOLINT(readability/casting)
  nibblesWritten_ = 0;
  const unsigned short* end = input + numPixels;  // NOLINT(runtime/int)
  unsigned short previous = 0;  // NOLINT(runtime/int)
  while (input != end) {
    int zeros = 0, nonzeros = 0;
    for (; (input != end) && !*input; input++, zeros++) {}
    EncodeVLE(zeros);  // number of zeros
    for (const unsigned short* p = input; (p != end) && *p++; nonzeros++) {}  // NOLINT(runtime/int)
    EncodeVLE(nonzeros);  // number of nonzeros
    for (int i = 0; i < nonzeros; i++) {
      unsigned short current = *input++;  // NOLINT(runtime/int)
      int delta = current - previous;
      int positive = (delta << 1) ^ (delta >> 31);  // NOLINT
      EncodeVLE(positive);  // nonzero value
      previous = current;
    }
  }
  if (nibblesWritten_)  // last few values
    *pBuffer_++ = word_ << 4 * (8 - nibblesWritten_);
  return int((unsigned char*)pBuffer_ - (unsigned char*)buffer_);  // num bytes NOLINT
}

void RvlCodec::DecompressRVL(const unsigned char* input, unsigned short* output, int numPixels)  // NOLINT(runtime/int)
{
  buffer_ = pBuffer_ = const_cast<int*>(reinterpret_cast<const int*>(input));
  nibblesWritten_ = 0;
  unsigned short current, previous = 0;  // NOLINT(runtime/int)
  int numPixelsToDecode = numPixels;
  while (numPixelsToDecode) {
    int zeros = DecodeVLE();  // number of zeros
    numPixelsToDecode -= zeros;
    for (; zeros; zeros--) *output++ = 0;
    int nonzeros = DecodeVLE();  // number of nonzeros
    numPixelsToDecode -= nonzeros;
    for (; nonzeros; nonzeros--) {
      int positive = DecodeVLE();  // nonzero value
      int delta = (positive >> 1) ^ -(positive & 1);
      current = previous + delta;
      *output++ = current;
      previous = current;
    }
  }
}

thread_local auto globalLogger = std::make_shared<cras::MemoryLogHelper>();
thread_local CompressedDepthCodec compressed_depth_codec_instance(globalLogger);

}

bool compressedDepthCodecEncode(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
#if COMPRESSED_DEPTH_HAS_RVL == 1
  const char* configFormat,
#endif
  double configDepthMax,
  double configDepthQuantization,
  int configPngLevel,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
)
{
  sensor_msgs::Image raw;
  raw.height = rawHeight;
  raw.width = rawWidth;
  raw.encoding = rawEncoding;
  raw.is_bigendian = rawIsBigEndian;
  raw.step = rawStep;
  raw.data.resize(rawDataLength);
  memcpy(raw.data.data(), rawData, rawDataLength);

  compressed_depth_image_transport::CompressedDepthPublisherConfig config;
#if COMPRESSED_DEPTH_HAS_RVL == 1
  config.format = configFormat;
#endif
  config.depth_max = configDepthMax;
  config.depth_quantization = configDepthQuantization;
  config.png_level = configPngLevel;

  image_transport_codecs::globalLogger->clear();

  const auto compressed = image_transport_codecs::compressed_depth_codec_instance.encode(raw, config);

  for (const auto& msg : image_transport_codecs::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  image_transport_codecs::globalLogger->clear();

  if (!compressed)
  {
    cras::outputString(errorStringAllocator, compressed.error());
    return false;
  }

  cras::outputString(compressedFormatAllocator, compressed->format);
  cras::outputByteBuffer(compressedDataAllocator, compressed->data);

  return true;
}

bool compressedDepthCodecDecode(
  const char* compressedFormat,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  sensor_msgs::Image::_height_type& rawHeight,
  sensor_msgs::Image::_width_type& rawWidth,
  cras::allocator_t rawEncodingAllocator,
  sensor_msgs::Image::_is_bigendian_type& rawIsBigEndian,
  sensor_msgs::Image::_step_type& rawStep,
  cras::allocator_t rawDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
)
{
  sensor_msgs::CompressedImage compressed;
  compressed.format = compressedFormat;
  compressed.data.resize(compressedDataLength);
  memcpy(compressed.data.data(), compressedData, compressedDataLength);

  image_transport_codecs::globalLogger->clear();

  const auto raw = image_transport_codecs::compressed_depth_codec_instance.decode(compressed);

  for (const auto& msg : image_transport_codecs::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  image_transport_codecs::globalLogger->clear();

  if (!raw)
  {
    cras::outputString(errorStringAllocator, raw.error());
    return false;
  }

  rawHeight = raw->height;
  rawWidth = raw->width;
  rawIsBigEndian = raw->is_bigendian;
  rawStep = raw->step;
  cras::outputString(rawEncodingAllocator, raw->encoding);
  cras::outputByteBuffer(rawDataAllocator, raw->data);

  return true;
}

bool compressed_depth_codec_has_rvl()
{
  return COMPRESSED_DEPTH_HAS_RVL;
}
