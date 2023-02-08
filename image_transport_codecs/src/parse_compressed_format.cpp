/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <unordered_map>

#include <compressed_depth_image_transport/CompressedDepthPublisherConfig.h>
#include <compressed_image_transport/CompressedPublisherConfig.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <sensor_msgs/image_encodings.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <image_transport_codecs/parse_compressed_format.h>

namespace image_transport_codecs
{

std::unordered_map<std::string, CompressedTransportCompressionFormat> compressedFormatTypes =
{
  {compressed_image_transport::CompressedPublisher_jpeg, CompressedTransportCompressionFormat::JPEG},
  {compressed_image_transport::CompressedPublisher_png, CompressedTransportCompressionFormat::PNG},
};

std::unordered_map<CompressedTransportCompressionFormat, std::string> compressedFormatNames =
{
  {CompressedTransportCompressionFormat::JPEG, compressed_image_transport::CompressedPublisher_jpeg},
  {CompressedTransportCompressionFormat::PNG, compressed_image_transport::CompressedPublisher_png},
};

cras::expected<CompressedTransportFormat, std::string> parseCompressedTransportFormat(const std::string& format)
{
  CompressedTransportFormat result;

  const auto& parts = cras::split(format, ";", 1);
  if (parts.size() == 1)
  {
    // old format
    // if the format field is nonempty, it might contain just "jpeg" or "png"
    const auto& f = cras::strip(format);
    if (f.empty())
      result.format = CompressedTransportCompressionFormat::JPEG;
    else if (compressedFormatTypes.find(f) != compressedFormatTypes.end())
      result.format = compressedFormatTypes[f];
    else
      return cras::make_unexpected("compressed transport format '" + format + "' is invalid.");
    result.formatString = compressedFormatNames[result.format];
    result.numChannels = 3;
    result.bitDepth = 8;
    result.isColor = true;
    result.rawEncoding = result.compressedEncoding = sensor_msgs::image_encodings::BGR8;
    return result;
  }

  // RAW_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]

  result.rawEncoding = result.compressedEncoding = cras::strip(parts[0]);
  const auto parts2 = cras::split(cras::strip(parts[1]), " ");

  result.format = CompressedTransportCompressionFormat::JPEG;
  if (parts2.size() >= 2 && cras::strip(parts2[1]) == "compressed")
  {
    const auto& f = cras::strip(parts2[0]);
    if (compressedFormatTypes.find(f) != compressedFormatTypes.end())
      result.format = compressedFormatTypes[f];

    if (parts2.size() > 2)
    {
      const auto &enc = cras::strip(parts2[2]);
      if (!enc.empty())
        result.compressedEncoding = enc;
    }
  }
  else
    return cras::make_unexpected("compressed transport format '" + format + "' is invalid.");

  result.formatString = compressedFormatNames[result.format];
  result.isColor = sensor_msgs::image_encodings::isColor(result.rawEncoding);
  try
  {
    result.bitDepth = sensor_msgs::image_encodings::bitDepth(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.bitDepth = result.format == CompressedTransportCompressionFormat::PNG ? 16 : 8;
  }
  try
  {
    result.numChannels = sensor_msgs::image_encodings::numChannels(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.numChannels = (result.format == CompressedTransportCompressionFormat::PNG && result.bitDepth == 16) ? 1 : 3;
  }

  return result;
}

std::string makeCompressedTransportFormat(const CompressedTransportFormat& format)
{
  if (format.formatString.empty() || format.bitDepth <= 0 || format.numChannels <= 0 || format.rawEncoding.empty())
    return "";

  return cras::format("%s; %s compressed %s", format.rawEncoding.c_str(), format.formatString.c_str(),
                      (format.isColor ? format.compressedEncoding.c_str() : ""));
}

std::unordered_map<std::string, CompressedDepthTransportCompressionFormat> compressedDepthFormatTypes =
{
#if COMPRESSED_DEPTH_HAS_RVL == 1
  {compressed_depth_image_transport::CompressedDepthPublisher_png, CompressedDepthTransportCompressionFormat::PNG},
  {compressed_depth_image_transport::CompressedDepthPublisher_rvl, CompressedDepthTransportCompressionFormat::RVL},
#else
  {"png", CompressedDepthTransportCompressionFormat::PNG},
  {"rvl", CompressedDepthTransportCompressionFormat::RVL},
#endif
};

std::unordered_map<CompressedDepthTransportCompressionFormat, std::string> compressedDepthFormatNames =
{
#if COMPRESSED_DEPTH_HAS_RVL == 1
  {CompressedDepthTransportCompressionFormat::PNG, compressed_depth_image_transport::CompressedDepthPublisher_png},
  {CompressedDepthTransportCompressionFormat::RVL, compressed_depth_image_transport::CompressedDepthPublisher_rvl},
#else
  {CompressedDepthTransportCompressionFormat::PNG, "png"},
  {CompressedDepthTransportCompressionFormat::RVL, "rvl"},
#endif
};

CompressedTransportFormat extractCompressedTransportFormat(
  const std::string& imageEncoding, const CompressedTransportCompressionFormat& compressionFormat)
{
  CompressedTransportFormat result;
  result.format = compressionFormat;
  result.formatString = compressedFormatNames[result.format];
  result.rawEncoding = result.compressedEncoding = imageEncoding;
  result.isColor = sensor_msgs::image_encodings::isColor(result.rawEncoding);

  try
  {
    result.bitDepth = sensor_msgs::image_encodings::bitDepth(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.bitDepth = (result.isColor || result.format == CompressedTransportCompressionFormat::JPEG) ? 8 : 16;
  }

  try
  {
    result.numChannels = sensor_msgs::image_encodings::numChannels(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.numChannels = result.isColor ? 3 : 1;
  }

  if (result.isColor)
  {
    result.compressedEncoding =
      result.bitDepth == 8 ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::BGR16;
  }

  return result;
}

CompressedTransportFormat extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const CompressedTransportCompressionFormat& compressionFormat)
{
  return extractCompressedTransportFormat(image.encoding, compressionFormat);
}

cras::expected<CompressedDepthTransportFormat, std::string> parseCompressedDepthTransportFormat(
  const std::string& format)
{
  CompressedDepthTransportFormat result;

  const auto& parts = cras::split(format, ";", 1);
  if (parts.size() == 1)
  {
    // old format
    // if the format field is nonempty, it might contain just "png" or "rvl"
    const auto& f = cras::strip(format);
    if (f.empty())
      result.format = CompressedDepthTransportCompressionFormat::PNG;
    else if (compressedDepthFormatTypes.find(f) != compressedDepthFormatTypes.end())
      result.format = compressedDepthFormatTypes[f];
    else
      return cras::make_unexpected("compressedDepth transport format '" + format + "' is invalid.");
    result.formatString = compressedDepthFormatNames[result.format];
    result.bitDepth = 16;
    result.rawEncoding = sensor_msgs::image_encodings::TYPE_16UC1;
    return result;
  }

  // RAW_PIXFMT; compressedDepth [CODEC]

  result.rawEncoding = cras::strip(parts[0]);
  const auto parts2 = cras::split(cras::strip(parts[1]), " ");

  result.format = CompressedDepthTransportCompressionFormat::PNG;
  if (!parts2.empty() && cras::strip(parts2[0]) == "compressedDepth")
  {
    if (parts2.size() > 1)
    {
      // Noetic version with RVL
      const auto& f = cras::strip(parts2[1]);
      if (compressedDepthFormatTypes.find(f) != compressedDepthFormatTypes.end())
        result.format = compressedDepthFormatTypes[f];
    }
    else
    {
      // Melodic version
      result.format = CompressedDepthTransportCompressionFormat::PNG;
    }
  }
  else
    return cras::make_unexpected("compressedDepth transport format '" + format + "' is invalid.");

  result.formatString = compressedDepthFormatNames[result.format];
  try
  {
    result.bitDepth = sensor_msgs::image_encodings::bitDepth(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.bitDepth = 16;
  }

  return result;
}

std::string makeCompressedDepthTransportFormat(const CompressedDepthTransportFormat& format)
{
  if (format.formatString.empty() || format.bitDepth <= 0 || format.rawEncoding.empty())
    return "";

#if COMPRESSED_DEPTH_HAS_RVL == 1
  return cras::format("%s; compressedDepth %s", format.rawEncoding.c_str(), format.formatString.c_str());
#else
  return cras::format("%s; compressedDepth", format.rawEncoding.c_str());
#endif
}

CompressedDepthTransportFormat extractCompressedDepthTransportFormat(
  const std::string& imageEncoding, const CompressedDepthTransportCompressionFormat& compressionFormat)
{
  CompressedDepthTransportFormat result;
  result.format = compressionFormat;
  result.formatString = compressedDepthFormatNames[result.format];
  result.rawEncoding = imageEncoding;

  try
  {
    result.bitDepth = sensor_msgs::image_encodings::bitDepth(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.bitDepth = 16;
  }

  return result;
}

CompressedDepthTransportFormat extractCompressedDepthTransportFormat(
  const sensor_msgs::Image& image, const CompressedDepthTransportCompressionFormat& compressionFormat)
{
  return extractCompressedDepthTransportFormat(image.encoding, compressionFormat);
}

cras::expected<CompressedTransportFormat, std::string> extractCompressedTransportFormat(
  const std::string& imageEncoding, const std::string& compressionFormat)
{
  if (compressedFormatTypes.find(compressionFormat) == compressedFormatTypes.end())
    return cras::make_unexpected("Unknown compressed transport format '" + compressionFormat + "'.");
  const auto& format2 = compressedFormatTypes[compressionFormat];
  return extractCompressedTransportFormat(imageEncoding, format2);
}

cras::expected<CompressedTransportFormat, std::string> extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const std::string& compressionFormat)
{
  return extractCompressedTransportFormat(image.encoding, compressionFormat);
}

cras::expected<CompressedDepthTransportFormat, std::string> extractCompressedDepthTransportFormat(
  const std::string& imageEncoding, const std::string& compressionFormat)
{
  if (compressedDepthFormatTypes.find(compressionFormat) == compressedDepthFormatTypes.end())
    return cras::make_unexpected("Unknown compressedDepth transport format '" + compressionFormat + "'.");
  const auto& format2 = compressedDepthFormatTypes[compressionFormat];
  return extractCompressedDepthTransportFormat(imageEncoding, format2);
}

cras::expected<CompressedDepthTransportFormat, std::string> extractCompressedDepthTransportFormat(
  const sensor_msgs::Image& image, const std::string& compressionFormat)
{
  return extractCompressedDepthTransportFormat(image.encoding, compressionFormat);
}

bool CompressedTransportFormat::operator==(const CompressedTransportFormat& other) const
{
  return
    this->format == other.format &&
    this->formatString == other.formatString &&
    this->rawEncoding == other.rawEncoding &&
    this->compressedEncoding == other.compressedEncoding &&
    this->numChannels == other.numChannels &&
    this->bitDepth == other.bitDepth &&
    this->isColor == other.isColor;
}

bool CompressedDepthTransportFormat::operator==(const CompressedDepthTransportFormat& other) const
{
  return
    this->format == other.format &&
    this->formatString == other.formatString &&
    this->rawEncoding == other.rawEncoding &&
    this->bitDepth == other.bitDepth;
}

}

bool parseCompressedTransportFormat(
  const char* format,
  cras::allocator_t compressionFormatAllocator,
  cras::allocator_t rawEncodingAllocator,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  cras::allocator_t errorStringAllocator)
{
  const auto parsed = image_transport_codecs::parseCompressedTransportFormat(format);
  if (!parsed)
  {
    cras::outputString(errorStringAllocator, parsed.error());
    return false;
  }

  cras::outputString(compressionFormatAllocator, parsed->formatString);
  cras::outputString(rawEncodingAllocator, parsed->rawEncoding);
  cras::outputString(compressedEncodingAllocator, parsed->compressedEncoding);

  numChannels = parsed->numChannels;
  bitDepth = parsed->bitDepth;
  isColor = parsed->isColor;

  return true;
}

bool makeCompressedTransportFormat(
  const char* compressionFormat,
  const char* rawEncoding,
  const char* compressedEncoding,
  int numChannels,
  int bitDepth,
  bool isColor,
  cras::allocator_t formatAllocator,
  cras::allocator_t errorStringAllocator)
{
  namespace itc = image_transport_codecs;
  if (itc::compressedFormatTypes.find(compressionFormat) == itc::compressedFormatTypes.end())
  {
    cras::outputString(errorStringAllocator,
      cras::format("Unknown compressed transport format '%s'.", compressionFormat));
    return false;
  }
  itc::CompressedTransportFormat format {itc::compressedFormatTypes[compressionFormat], compressionFormat,
    rawEncoding, compressedEncoding, numChannels, bitDepth, isColor};
  cras::outputString(formatAllocator, itc::makeCompressedTransportFormat(format));
  return true;
}

bool extractCompressedTransportFormat(
  const char* imageEncoding,
  const char* compressionFormat,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  cras::allocator_t errorStringAllocator)
{
  const auto format = image_transport_codecs::extractCompressedTransportFormat(imageEncoding, compressionFormat);
  if (!format)
  {
    cras::outputString(errorStringAllocator, format.error());
    return false;
  }
  cras::outputString(compressedEncodingAllocator, format->compressedEncoding);
  numChannels = format->numChannels;
  bitDepth = format->bitDepth;
  isColor = format->isColor;

  return true;
}

bool parseCompressedDepthTransportFormat(
  const char* format,
  cras::allocator_t compressionFormatAllocator,
  cras::allocator_t rawEncodingAllocator,
  int& bitDepth,
  cras::allocator_t errorStringAllocator)
{
  const auto parsed = image_transport_codecs::parseCompressedDepthTransportFormat(format);
  if (!parsed)
  {
    cras::outputString(errorStringAllocator, parsed.error());
    return false;
  }

  cras::outputString(compressionFormatAllocator, parsed->formatString);
  cras::outputString(rawEncodingAllocator, parsed->rawEncoding);
  bitDepth = parsed->bitDepth;

  return true;
}

bool makeCompressedDepthTransportFormat(
  const char* compressionFormat,
  const char* rawEncoding,
  int bitDepth,
  cras::allocator_t formatAllocator,
  cras::allocator_t errorStringAllocator)
{
  namespace itc = image_transport_codecs;
  if (itc::compressedDepthFormatTypes.find(compressionFormat) == itc::compressedDepthFormatTypes.end())
  {
    cras::outputString(errorStringAllocator,
      cras::format("Unknown compressedDepth transport format '%s'.", compressionFormat));
    return false;
  }
  itc::CompressedDepthTransportFormat format {
    itc::compressedDepthFormatTypes[compressionFormat], compressionFormat, rawEncoding, bitDepth};
  cras::outputString(formatAllocator, itc::makeCompressedDepthTransportFormat(format));
  return true;
}

bool extractCompressedDepthTransportFormat(
  const char* imageEncoding,
  const char* compressionFormat,
  int& bitDepth,
  cras::allocator_t errorStringAllocator)
{
  const auto format = image_transport_codecs::extractCompressedDepthTransportFormat(imageEncoding, compressionFormat);
  if (!format)
  {
    cras::outputString(errorStringAllocator, format.error());
    return false;
  }
  bitDepth = format->bitDepth;
  return true;
}
