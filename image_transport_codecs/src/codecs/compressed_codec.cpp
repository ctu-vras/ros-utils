/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// This file is heavily based on the BSD-3-licensed compressed_image_transport package:
// https://github.com/ros-perception/image_transport_plugins/tree/noetic-devel/compressed_image_transport/src
// The basic algorithms from upstream are unchanged, although the code has undergone cosmetic and API-design changes.

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <boost/endian/arithmetic.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
// If OpenCV4
#if CV_VERSION_MAJOR > 3
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif
#include <turbojpeg.h>

#include <compressed_image_transport/CompressedPublisherConfig.h>
#include <compressed_image_transport/CompressedSubscriberConfig.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <image_transport_codecs/parse_compressed_format.h>
#include <image_transport_codecs/codecs/compressed_codec.h>

namespace image_transport_codecs
{

namespace enc = sensor_msgs::image_encodings;

struct CompressedCodecPrivate
{
  tjhandle tj_ {nullptr};
};

CompressedCodec::CompressedCodec(const ::cras::LogHelperPtr& logHelper) : ImageTransportCodec(logHelper),
  data(new CompressedCodecPrivate)
{
}

CompressedCodec::~CompressedCodec()
{
  if (this->data->tj_)
  {
    tjDestroy(this->data->tj_);
    this->data->tj_ = nullptr;
  }
}

CompressedCodec::EncodeResult CompressedCodec::encode(const sensor_msgs::Image& raw,
  const compressed_image_transport::CompressedPublisherConfig& config) const
{
  const auto format = extractCompressedTransportFormat(raw, config.format);
  if (!format)
    return cras::make_unexpected("Invalid compressed encoder config: " + format.error());

  sensor_msgs::CompressedImage compressed;
  compressed.header = raw.header;
  compressed.format = makeCompressedTransportFormat(format.value());

  // Compression settings
  std::vector<int> params;

  switch (format->format)
  {
    // JPEG Compression
    case CompressedTransportCompressionFormat::JPEG:
    {
#if COMPRESSED_HAS_JPEG_OPTIONS == 1
      params.resize(9, 0);
#else
      params.resize(3, 0);
#endif
      params[0] = cv::IMWRITE_JPEG_QUALITY;
      params[1] = config.jpeg_quality;
#if COMPRESSED_HAS_JPEG_OPTIONS == 1
      params[2] = cv::IMWRITE_JPEG_PROGRESSIVE;
      params[3] = config.jpeg_progressive ? 1 : 0;
      params[4] = cv::IMWRITE_JPEG_OPTIMIZE;
      params[5] = config.jpeg_optimize ? 1 : 0;
      params[6] = cv::IMWRITE_JPEG_RST_INTERVAL;
      params[7] = config.jpeg_restart_interval;
#endif

      // Check input format
      if ((format->bitDepth == 8) || (format->bitDepth == 16))
      {
        // OpenCV-ros bridge
        try
        {
          auto cv_ptr = cv_bridge::toCvShare(raw, nullptr, format->compressedEncoding);

          // Compress image
          if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
            return compressed;
          else
            // The bool return value of imencode is undocumented and it seems it cannot happen, but for completeness...
            return cras::make_unexpected(cras::format(
              "Unknown OpenCV error occurred while encoding %ix%i %s image as %s.",
              raw.width, raw.height, raw.encoding.c_str(), config.format.c_str()));
        }
        catch (cv_bridge::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "cv_bridge error occurred while encoding %ix%i %s image as %s: %s.",
            raw.width, raw.height, raw.encoding.c_str(), config.format.c_str(), e.what()));
        }
        catch (cv::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "OpenCV error occurred while encoding %ix%i %s image as %s: %s (%s).",
            raw.width, raw.height, raw.encoding.c_str(), config.format.c_str(), cvErrorStr(e.code), e.err.c_str()));
        }
      }
      else
        return cras::make_unexpected("Error encoding " + raw.encoding +
          " image as jpeg: only 8-bit and 16-bit JPEGs are supported.");
    }

    // PNG Compression
    case CompressedTransportCompressionFormat::PNG:
    {
      params.resize(3, 0);
      params[0] = cv::IMWRITE_PNG_COMPRESSION;
      params[1] = config.png_level;

      // Check input format
      if ((format->bitDepth == 8) || (format->bitDepth == 16))
      {
        // OpenCV-ros bridge
        try
        {
          auto cv_ptr = cv_bridge::toCvShare(raw, nullptr, format->compressedEncoding);

          // Compress image
          if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
            return compressed;
          else
            // The bool return value of imencode is undocumented and it seems it cannot happen, but for completeness...
            return cras::make_unexpected(cras::format(
              "Unknown OpenCV error occurred while encoding %ix%i %s image as %s.",
              raw.width, raw.height, raw.encoding.c_str(), config.format.c_str()));
        }
        catch (cv_bridge::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "cv_bridge error occurred while encoding %ix%i %s image as %s: %s.",
            raw.width, raw.height, raw.encoding.c_str(), config.format.c_str(), e.what()));
        }
        catch (cv::Exception& e)
        {
          return cras::make_unexpected(cras::format(
            "OpenCV error occurred while encoding %ix%i %s image as %s: %s (%s).",
            raw.width, raw.height, raw.encoding.c_str(), config.format.c_str(), cvErrorStr(e.code), e.err.c_str()));
        }
      }
      else
        return cras::make_unexpected("Error encoding " + raw.encoding +
          " image as png: only 8-bit and 16-bit PNGs are supported.");
    }

    default:
    {
      return cras::make_unexpected(cras::format("Invalid encoding format %i.", static_cast<int>(format->format)));
    }
  }
}

ImageTransportCodec::DecodeResult CompressedCodec::decompressJPEG(
  const std::vector<uint8_t>& data, const std::string& source_encoding, const std_msgs::Header& header) const
{
  if (!this->data->tj_)
    this->data->tj_ = tjInitDecompress();

  if (!this->data->tj_)
    return cras::make_unexpected("Could not initialize TurboJPEG decoder.");

  int width, height, jpegSub, jpegColor;

  // Old TurboJPEG require a const_cast here. This was fixed in TurboJPEG 1.5.
  auto src = const_cast<uint8_t*>(data.data());

  if (tjDecompressHeader3(this->data->tj_, src, data.size(), &width, &height, &jpegSub, &jpegColor) != 0)
    // If we cannot decode the JPEG header, silently fall back to OpenCV
    return cras::make_unexpected(cras::format("TurboJPEG could not process image: %s.", tjGetErrorStr()));

  sensor_msgs::Image img;
  img.header = header;
  img.width = width;
  img.height = height;
  img.encoding = source_encoding;
  // consistent with cv_bridge
  img.is_bigendian = (boost::endian::order::native == boost::endian::order::big);  // NOLINT

  int pixelFormat;

  if (source_encoding == enc::MONO8)
  {
    img.data.resize(height*width);
    img.step = img.width;
    pixelFormat = TJPF_GRAY;
  }
  else if (source_encoding == enc::RGB8)
  {
    img.data.resize(height*width*3);
    img.step = width*3;
    pixelFormat = TJPF_RGB;
  }
  else if (source_encoding == enc::BGR8)
  {
    img.data.resize(height*width*3);
    img.step = width*3;
    pixelFormat = TJPF_BGR;
  }
  else if (source_encoding == enc::RGBA8)
  {
    img.data.resize(height*width*4);
    img.step = width*4;
    pixelFormat = TJPF_RGBA;
  }
  else if (source_encoding == enc::BGRA8)
  {
    img.data.resize(height*width*4);
    img.step = width*4;
    pixelFormat = TJPF_BGRA;
  }
  else if (source_encoding.empty())
  {
    // Autodetect based on image
    if (jpegColor == TJCS_GRAY)
    {
      img.data.resize(height*width);
      img.step = width;
      img.encoding = enc::MONO8;
      pixelFormat = TJPF_GRAY;
    }
    else
    {
      img.data.resize(height*width*3);
      img.step = width*3;
      img.encoding = enc::RGB8;
      pixelFormat = TJPF_RGB;
    }
  }
  else
  {
    return cras::make_unexpected("Unsupported image encoding " + source_encoding + ".");
  }

  if (tjDecompress2(this->data->tj_, src, data.size(), img.data.data(), width, 0, height, pixelFormat, 0) != 0)
  {
    return cras::make_unexpected(cras::format("TurboJPEG failed to decode image: %s.", tjGetErrorStr()));
  }

  return img;
}

ImageTransportCodec::DecodeResult CompressedCodec::decode(const sensor_msgs::CompressedImage& compressed) const
{
  return this->decode(compressed, compressed_image_transport::CompressedSubscriberConfig::__getDefault__());
}

ImageTransportCodec::DecodeResult CompressedCodec::decode(const sensor_msgs::CompressedImage& compressed,
  const compressed_image_transport::CompressedSubscriberConfig& config) const
{
  // Parse format field
  const auto format = parseCompressedTransportFormat(compressed.format);
  if (!format)
    return cras::make_unexpected("Invalid compressed decoder config: " + format.error());

  // Try TurboJPEG first (if the first bytes look like JPEG)
  if (compressed.data.size() > 4 && compressed.data[0] == 0xFF && compressed.data[1] == 0xD8)
  {
    auto decoded = decompressJPEG(compressed.data, format->rawEncoding, compressed.header);
    if (decoded)
      return decoded;
  }

  // Otherwise, try our luck with OpenCV.
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = compressed.header;

  // Decode color/mono image
  try
  {
    int imdecode_flag;
    if (config.mode == compressed_image_transport::CompressedSubscriber_gray) {
      imdecode_flag = cv::IMREAD_GRAYSCALE;
    } else if (config.mode == compressed_image_transport::CompressedSubscriber_color) {
      imdecode_flag = cv::IMREAD_COLOR;
    } else /*if (config_.mode == compressed_image_transport::CompressedSubscriber_unchanged)*/ {
      imdecode_flag = cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH;
    }

    cv_ptr->image = cv::imdecode(cv::Mat(compressed.data), imdecode_flag);

    // Assign image encoding string
    {
      cv_ptr->encoding = format->rawEncoding;

      if (format->isColor)
      {
        // Revert color transformation
        if (cras::startsWith(format->compressedEncoding, "bgr"))
        {
          // if necessary convert colors from bgr to rgb
          if ((format->rawEncoding == enc::RGB8) || (format->rawEncoding == enc::RGB16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

          if ((format->rawEncoding == enc::RGBA8) || (format->rawEncoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

          if ((format->rawEncoding == enc::BGRA8) || (format->rawEncoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
        }
        else
        {
          // if necessary convert colors from rgb to bgr
          if ((format->rawEncoding == enc::BGR8) || (format->rawEncoding == enc::BGR16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

          if ((format->rawEncoding == enc::BGRA8) || (format->rawEncoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

          if ((format->rawEncoding == enc::RGBA8) || (format->rawEncoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
        }
      }
    }
  }
  catch (cv::Exception& e)
  {
    return cras::make_unexpected(cras::format(
      "OpenCV error occurred while decoding '%s' image: %s (%s).",
      compressed.format.c_str(), cvErrorStr(e.code), e.err.c_str()));
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
    return *cv_ptr->toImageMsg();

  return cras::make_unexpected("Decoding compressed image yielded a zero-size result.");
}

std::string CompressedCodec::getTransportName() const
{
  return "compressed";
}

ImageTransportCodec::EncodeResult CompressedCodec::encode(
  const sensor_msgs::Image& raw, const dynamic_reconfigure::Config& config) const
{
  auto codecConfig = compressed_image_transport::CompressedPublisherConfig::__getDefault__();
  if (!codecConfig.__fromMessage__(*const_cast<dynamic_reconfigure::Config*>(&config)))
    return cras::make_unexpected("Invalid config passed to compressed transport encoder.");

  const auto compressedImage = this->encode(raw, codecConfig);
  if (!compressedImage)
    return cras::make_unexpected(compressedImage.error());

  cras::ShapeShifter compressed;
  cras::msgToShapeShifter(compressedImage.value(), compressed);
  return compressed;
}

ImageTransportCodec::DecodeResult CompressedCodec::decode(
  const topic_tools::ShapeShifter& compressed, const dynamic_reconfigure::Config& config) const
{
  auto codecConfig = compressed_image_transport::CompressedSubscriberConfig::__getDefault__();
  if (!codecConfig.__fromMessage__(*const_cast<dynamic_reconfigure::Config*>(&config)))
    return cras::make_unexpected("Invalid config passed to compressed transport decoder.");

  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressed decoder: %s.", e.what()));
  }

  return this->decode(*compressedImage, codecConfig);
}

ImageTransportCodec::GetCompressedContentResult CompressedCodec::getCompressedImageContent(
  const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressed decoder: %s.", e.what()));
  }
  return this->getCompressedImageContent(*compressedImage, matchFormat);
}

ImageTransportCodec::GetCompressedContentResult CompressedCodec::getCompressedImageContent(
  const sensor_msgs::CompressedImage& compressed, const std::string& matchFormat) const
{
  const auto format = parseCompressedTransportFormat(compressed.format);
  if (!format)
    return cras::make_unexpected("Invalid compressed format: " + format.error());

  const auto targetFormat = (cras::toLower(matchFormat) == "jpg") ? "jpeg" : cras::toLower(matchFormat);
  if (!targetFormat.empty() && cras::toLower(format->formatString) != targetFormat)
    return cras::nullopt;

  return CompressedImageContent{format->formatString, compressed.data};
}

thread_local auto globalLogger = std::make_shared<cras::MemoryLogHelper>();
thread_local CompressedCodec compressed_codec_instance(globalLogger);
}

bool compressedCodecEncode(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
  const char* configFormat,
  int configJpegQuality,
#if COMPRESSED_HAS_JPEG_OPTIONS == 1
  bool configJpegProgressive,
  bool configJpegOptimize,
  int configJpegRestartInterval,
#endif
  int configPngLevel,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator)
{
  sensor_msgs::Image raw;
  raw.height = rawHeight;
  raw.width = rawWidth;
  raw.encoding = rawEncoding;
  raw.is_bigendian = rawIsBigEndian;
  raw.step = rawStep;
  raw.data.resize(rawDataLength);
  memcpy(raw.data.data(), rawData, rawDataLength);

  compressed_image_transport::CompressedPublisherConfig config;
  config.format = configFormat;
  config.jpeg_quality = configJpegQuality;
#if COMPRESSED_HAS_JPEG_OPTIONS == 1
  config.jpeg_progressive = configJpegProgressive;
  config.jpeg_optimize = configJpegOptimize;
  config.jpeg_restart_interval = configJpegRestartInterval;
#endif
  config.png_level = configPngLevel;

  image_transport_codecs::globalLogger->clear();

  const auto compressed = image_transport_codecs::compressed_codec_instance.encode(raw, config);

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

bool compressedCodecDecode(
  const char* compressedFormat,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  sensor_msgs::Image::_height_type& rawHeight,
  sensor_msgs::Image::_width_type& rawWidth,
  cras::allocator_t rawEncodingAllocator,
  sensor_msgs::Image::_is_bigendian_type& rawIsBigEndian,
  sensor_msgs::Image::_step_type& rawStep,
  cras::allocator_t rawDataAllocator,
  const char* configMode,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
)
{
  sensor_msgs::CompressedImage compressed;
  compressed.format = compressedFormat;
  compressed.data.resize(compressedDataLength);
  memcpy(compressed.data.data(), compressedData, compressedDataLength);

  compressed_image_transport::CompressedSubscriberConfig config;
  config.mode = configMode;

  image_transport_codecs::globalLogger->clear();

  const auto raw = image_transport_codecs::compressed_codec_instance.decode(compressed, config);

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

bool compressed_codec_has_extra_jpeg_options()
{
  return COMPRESSED_HAS_JPEG_OPTIONS;
}
