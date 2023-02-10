#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Methods for parsing the values of field `sensor_msgs::CompressedImage::format` for `compressed` and
 *        `compressedDepth` codecs.
 * \author Martin Pecka
 */

#include <string>
#include <utility>

#include <sensor_msgs/Image.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>

namespace image_transport_codecs
{

/**
 * \brief Compression format of `compressed` codec (JPEG/PNG).
 */
enum class CompressedTransportCompressionFormat
{
  JPEG,  //!< \brief JPEG compression format.
  PNG,  //!< \brief PNG compression format.
};

/**
 * \brief Decoded meaning of field `sensor_msgs::CompressedImage::format` for `compressed` transport.
 */
struct CompressedTransportFormat
{
  CompressedTransportCompressionFormat format;  //!< \brief The compression format (JPEG/PNG).
  std::string formatString;  //!< \brief Text version of the compression format ("jpeg"/"png").
  std::string rawEncoding;  //!< \brief Encoding of the raw image (before compression, after decompression).
  std::string compressedEncoding;  //!< \brief Encoding of the compressed image (i.e. `bgr8` for JPEG).
  int numChannels;  //!< \brief Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
  int bitDepth;  //!< \brief Number of bits used for encoding one raw channel value.
  bool isColor;  //!< \brief Whether the image is a color image or not.

  bool operator==(const CompressedTransportFormat& other) const;
};

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using `compressed` transport into
 *        `CompressedTransportFormat` structure.
 * \param[in] format The `format` field text.
 * \return The parsed structure or error string.
 */
cras::expected<CompressedTransportFormat, std::string> parseCompressedTransportFormat(const std::string& format);

/**
 * \brief Convert the `CompressedTransportFormat` structure into a string to be filled in field
 *        `sensor_msgs::CompressedImage::format` of `compressed` transport image.
 * \param[in] format The format to convert.
 * \return The string for the `format` field.
 */
std::string makeCompressedTransportFormat(const CompressedTransportFormat& format);

/**
 * \brief Create the `CompressedTransportFormat` structure for the given raw image compressed with the given method. 
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The target compression method.
 * \return The `CompressedTransportFormat` structure corresponding to the given image and compression method.
 */
CompressedTransportFormat extractCompressedTransportFormat(
  const std::string& imageEncoding, const CompressedTransportCompressionFormat& compressionFormat);

/**
 * \brief Create the `CompressedTransportFormat` structure for the given raw image compressed with the given method. 
 * \param[in] image The raw image.
 * \param[in] compressionFormat The target compression method.
 * \return The `CompressedTransportFormat` structure corresponding to the given image and compression method.
 */
CompressedTransportFormat extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const CompressedTransportCompressionFormat& compressionFormat);

/**
 * \brief Create the `CompressedTransportFormat` structure for the given raw image compressed with the given method. 
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The string version of target compression method (either "jpeg" or "png").
 * \return The `CompressedTransportFormat` structure corresponding to the given image and compression method, or error
 *         string if `format` is invalid.
 */
cras::expected<CompressedTransportFormat, std::string> extractCompressedTransportFormat(
  const std::string& imageEncoding, const std::string& compressionFormat);

/**
 * \brief Create the `CompressedTransportFormat` structure for the given raw image compressed with the given method. 
 * \param[in] image The raw image.
 * \param[in] compressionFormat The string version of target compression method (either "jpeg" or "png").
 * \return The `CompressedTransportFormat` structure corresponding to the given image and compression method, or error
 *         string if `format` is invalid.
 */
cras::expected<CompressedTransportFormat, std::string> extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const std::string& compressionFormat);

/**
 * \brief Compression format of `compressedDepth` codec (PNG/RVL). RVL is only usable in Noetic.
 */
enum class CompressedDepthTransportCompressionFormat
{
  PNG,  //!< \brief PNG compression format.
  RVL,  //!< \brief RVL compression format (only usable in Noetic).
};

/**
 * \brief Decoded meaning of field `sensor_msgs::CompressedImage::format` for `compressedDepth` transport.
 */
struct CompressedDepthTransportFormat
{
  CompressedDepthTransportCompressionFormat format;  //!< \brief The compression format (PNG/RVL).
  std::string formatString;  //!< \brief Text version of the compression format ("png"/"rvl").
  std::string rawEncoding;   //!< \brief Encoding of the raw image (before compression, after decompression).
  int bitDepth;  //!< \brief Number of bits used for encoding one depth value.

  bool operator==(const CompressedDepthTransportFormat& other) const;
};

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using `compressedDepth` transport into
 *        `CompressedDepthTransportFormat` structure.
 * \param[in] format The `format` field text.
 * \return The parsed `CompressedDepthTransportFormat` structure or error string.
 */
cras::expected<CompressedDepthTransportFormat, std::string> parseCompressedDepthTransportFormat(
  const std::string& format);

/**
 * \brief Convert the `CompressedDepthTransportFormat` structure into a string to be filled in field
 *        `sensor_msgs::CompressedImage::format` using `compressedDepth` transport.
 * \param[in] format The format to convert.
 * \return The string for the `format` field.
 */
std::string makeCompressedDepthTransportFormat(const CompressedDepthTransportFormat& format);

/**
 * \brief Create the `CompressedDepthTransportFormat` structure for the given raw image compressed with the given
 *        method. 
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The target compression method.
 * \return The `CompressedDepthTransportFormat` structure corresponding to the given image and compression method.
 */
CompressedDepthTransportFormat extractCompressedDepthTransportFormat(
  const std::string& imageEncoding, const CompressedDepthTransportCompressionFormat& compressionFormat);

/**
 * \brief Create the `CompressedDepthTransportFormat` structure for the given raw image compressed with the given
 *        method. 
 * \param[in] image The raw image.
 * \param[in] compressionFormat The target compression method.
 * \return The `CompressedDepthTransportFormat` structure corresponding to the given image and compression method.
 */
CompressedDepthTransportFormat extractCompressedDepthTransportFormat(
  const sensor_msgs::Image& image, const CompressedDepthTransportCompressionFormat& compressionFormat);

/**
 * \brief Create the `CompressedDepthTransportFormat` structure for the given raw image compressed with the given
 *        method. 
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The string version of target compression method (either "png" or "rvl" (only on Noetic)).
 * \return The `CompressedDepthTransportFormat` structure corresponding to the given image and compression method, or
 *         error string if `format` is invalid.
 */
cras::expected<CompressedDepthTransportFormat, std::string> extractCompressedDepthTransportFormat(
  const std::string& imageEncoding, const std::string& compressionFormat);

/**
 * \brief Create the `CompressedDepthTransportFormat` structure for the given raw image compressed with the given
 *        method. 
 * \param[in] image The raw image.
 * \param[in] compressionFormat The string version of target compression method (either "png" or "rvl" (only on Noetic)).
 * \return The `CompressedDepthTransportFormat` structure corresponding to the given image and compression method, or
 *         error string if `format` is invalid.
 */
cras::expected<CompressedDepthTransportFormat, std::string> extractCompressedDepthTransportFormat(
  const sensor_msgs::Image& image, const std::string& compressionFormat);

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using either `compressed` or
 *        `compressedDepth` transport. Selection between the two formats is done automatically and it might peek into
 *        the first 64 bytes of the compressed byte stream in the image.
 * \param[in] image The image whose transport format should be parsed.
 * \return The parsed `CompressedTransportFormat` or `CompressedDepthTransportFormat` (exactly one of these two will
 *         be valid on success) or error string.
 */
cras::expected<
  std::pair<cras::optional<CompressedTransportFormat>, cras::optional<CompressedDepthTransportFormat>>, std::string>
guessAnyCompressedImageTransportFormat(const sensor_msgs::CompressedImage& image);

}

/////////////
/// C API ///
/////////////

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using `compressed` transport into
 *        individual components.
 * \param[in] format The `format` field text.
 * \param[in,out] compressionFormatAllocator Allocator for the compression format ("jpeg" or "png").
 * \param[in,out] rawEncodingAllocator Allocator for encoding of the raw image (before compression, after
 *                                     decompression).
 * \param[in,out] compressedEncodingAllocator Allocator for encoding of the compressed image (i.e. `bgr8` for JPEG).
 * \param[out] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[out] isColor Whether the image is a color image or not.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the parsing succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the parsing failed.
 */
extern "C" bool parseCompressedTransportFormat(
  const char* format,
  cras::allocator_t compressionFormatAllocator,
  cras::allocator_t rawEncodingAllocator,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Convert the `compressed` transport parameters into a string to be filled in field
 *        `sensor_msgs::CompressedImage::format`.
 * \param[in] compressionFormat The compression format ("jpeg" or "png").
 * \param[in] rawEncoding Encoding of the raw image (before compression, after decompression).
 * \param[in] compressedEncoding Encoding of the compressed image (i.e. `bgr8` for JPEG).
 * \param[in] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[in] bitDepth Number of bits used for encoding one raw channel value.
 * \param[in] isColor Whether the image is a color image or not.
 * \param[in,out] formatAllocator Allocator for the string for the `format` field.
 * \param[in,out] errorStringAllocator Allocator for explanation what failed (used only in case of failure).
 * \return Whether the conversion succeeded. If not, `formatAllocator` is not used, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the conversion failed.
 */
extern "C" bool makeCompressedTransportFormat(
  const char* compressionFormat,
  const char* rawEncoding,
  const char* compressedEncoding,
  int numChannels,
  int bitDepth,
  bool isColor,
  cras::allocator_t formatAllocator,
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Create the `compressed` transport parameters for the given raw image compressed with the given method. 
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The compression format ("jpeg" or "png").
 * \param[in,out] compressedEncodingAllocator Allocator for encoding of the compressed image (i.e. `bgr8` for JPEG).
 * \param[out] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[out] isColor Whether the image is a color image or not.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the extraction succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the extraction failed.
 */
extern "C" bool extractCompressedTransportFormat(
  const char* imageEncoding,
  const char* compressionFormat,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using `compressedDepth` transport into
 *        individual components.
 * \param[in] format The `format` field text.
 * \param[in,out] compressionFormatAllocator Allocator for the compression format ("png" or "rvl").
 * \param[in,out] rawEncodingAllocator Allocator for encoding of the raw image (before compression, after
 *                                     decompression).
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the parsing succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the parsing failed.
 */
extern "C" bool parseCompressedDepthTransportFormat(
  const char* format,
  cras::allocator_t compressionFormatAllocator,
  cras::allocator_t rawEncodingAllocator,
  int& bitDepth,
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Convert the `compressedDepth` transport parameters into a string to be filled in field
 *        `sensor_msgs::CompressedImage::format`.
 * \param[in] compressionFormat The compression format ("png" or "rvl").
 * \param[in] rawEncoding Encoding of the raw image (before compression, after decompression).
 * \param[in] bitDepth Number of bits used for encoding one raw channel value.
 * \param[in,out] formatAllocator Allocator for the string for the `format` field.
 * \param[in,out] errorStringAllocator Allocator for explanation what failed (used only in case of failure).
 * \return Whether the conversion succeeded. If not, `formatAllocator` is not used, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the conversion failed.
 */
extern "C" bool makeCompressedDepthTransportFormat(
  const char* compressionFormat,
  const char* rawEncoding,
  int bitDepth,
  cras::allocator_t formatAllocator,
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Create the `compressedDepth` transport parameters for the given raw image compressed with the given method. 
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The compression format ("png" or "rvl").
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the extraction succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the extraction failed.
 */
extern "C" bool extractCompressedDepthTransportFormat(
  const char* imageEncoding,
  const char* compressionFormat,
  int& bitDepth,
  cras::allocator_t errorStringAllocator
);

/**
 *
 * \param[in] image The image whose transport format should be parsed.
 * \return The parsed `CompressedTransportFormat` or `CompressedDepthTransportFormat` (exactly one of these two will
 *         be valid on success) or error string.
 */
/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using either `compressed` or
 *        `compressedDepth` transport. Selection between the two formats is done automatically and it might peek into
 *        the first 64 bytes of the compressed byte stream in the image.
 * \param[in] format The `format` field text.
 * \param[in] imageHeader First 64 bytes of the image's `data` field.
 * \param[out] isCompressedDepth True if the image should be decoded using `compressedDepth` transport.
 * \param[in,out] compressionFormatAllocator Allocator for the compression format ("jpeg", "png" or "rvl").
 * \param[in,out] rawEncodingAllocator Allocator for encoding of the raw image (before compression).
 * \param[in,out] compressedEncodingAllocator Allocator for encoding of the compressed image if the format is from
 *                                            `compressed` codec (i.e. `bgr8` for JPEG).
 * \param[out] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[out] isColor Whether the image is a color image or not.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the parsing succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the parsing failed.
 */
extern "C" bool guessAnyCompressedImageTransportFormat(
  const char* format,
  const uint8_t imageHeader[64],
  bool& isCompressedDepth,
  cras::allocator_t compressionFormatAllocator,
  cras::allocator_t rawEncodingAllocator,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  cras::allocator_t errorStringAllocator
);
