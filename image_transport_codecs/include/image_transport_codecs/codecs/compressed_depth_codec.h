#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport codec corresponding to
 *        [compressed_depth_image_transport](https://wiki.ros.org/compressed_depth_image_transport).
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <vector>

#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <compressed_depth_image_transport/CompressedDepthPublisherConfig.h>
#include <compressed_depth_image_transport/compression_common.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <image_transport_codecs/image_transport_codec.h>

namespace cv
{
class Mat;
}

namespace image_transport_codecs
{

/**
 * \brief Image transport codec corresponding to
 *        [compressed_depth_image_transport](https://wiki.ros.org/compressed_depth_image_transport).
 *
 * This codec exposes the functionality of `compressed_depth_image_transport` so that it can be used directly without
 * the need to go through running a node and publishing on a topic. E.g.
 * 
 * ```
 * sensor_msgs::Image raw = ...;
 * image_transport_codecs::CompressedDepthCodec codec;
 * auto result = codec.encode(raw);
 * if (!result)
 * {
 *   ROS_ERROR_STREAM("Compression failed: " << result.error());
 *   return false;
 * }
 * sensor_msgs::CompressedImage compressed = result.value();
 * ```
 */
class CompressedDepthCodec : public image_transport_codecs::ImageTransportCodec
{
public:
  //! \brief Result of image encoding. Either a `sensor_msgs::CompressedImage` message, or error message.
  typedef cras::expected<sensor_msgs::CompressedImage, std::string> EncodeResult;

  /**
   * \brief Create an instance of the codec.
   * \param[in] logHelper The logger to use for error messages not directly related to the currently processed image.
   */
  explicit CompressedDepthCodec(const cras::LogHelperPtr& logHelper = std::make_shared<cras::NodeLogHelper>());

  ~CompressedDepthCodec() override;

  /**
   * \brief Encode the given raw image using the given publisher config.
   * \param[in] raw The raw image to encode.
   * \param[in] config Configuration of the encoder (corresponds to the dynamic_reconfigure parameters of publisher).
   * \return The encoded image, or an error.
   * \sa Corresponding C API function: `compressedDepthCodecEncode()`.
   */
  EncodeResult encode(const sensor_msgs::Image& raw,
                      const compressed_depth_image_transport::CompressedDepthPublisherConfig& config) const;

  /**
   * \brief Decode the given compressed image.
   * \param[in] compressed The image to decode.
   * \return The decoded raw image, or an error.
   * \sa Corresponding C API function: `compressedDepthCodecDecode()`.
   */
  ImageTransportCodec::DecodeResult decode(const sensor_msgs::CompressedImage& compressed) const;

  std::string getTransportName() const override;

  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                           const dynamic_reconfigure::Config& config) const override;

  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const dynamic_reconfigure::Config& config) const override;

  /**
   * \brief Get the depth quantization parameters used by compression.
   * \param[in] compressed The compressed image to read the parameters from.
   * \return The depth quantization parameters. If an invalid image (i.e. from a different codec) is passed, the result
   *         is undefined.
   */
  compressed_depth_image_transport::ConfigHeader getCompressionConfig(
    const sensor_msgs::CompressedImage& compressed) const;

  /**
   * \brief Decode the given compressed image.
   * \param[in] compressed The image to decode.
   * \return The decoded raw image, or an error.
   */
  ImageTransportCodec::DecodeResult decodeCompressedDepthImage(const sensor_msgs::CompressedImage& compressed) const;

  /**
   * \brief Encode the given raw image using the given quantization and compression config.
   * \param[in] raw The raw image to encode.
   * \param[in] compression_format The compression format (png or rvl).
   * \param[in] depth_max Maximum depth value in meters (1-100).
   * \param[in] depth_quantization Depth value at which the sensor accuracy is 1 m (Kinect: >75) (1-150).
   * \param[in] png_level PNG compression level (if PNG is used) (1-9).
   * \return The encoded image, or an error.
   */
  EncodeResult encodeCompressedDepthImage(const sensor_msgs::Image& raw, const std::string& compression_format,
    double depth_max, double depth_quantization, int png_level) const;

  /**
   * \brief Decode a RVL-encoded image into a 16UC1 `cv::Mat`.
   * \param[in] compressed Bytes of a RVL-encoded image.
   * \return 2D matrix representing the decoded raw image.
   */
  cv::Mat decodeRVL(const std::vector<uint8_t>& compressed) const;

  /**
   * \brief Encode the given 16UC1 image (i.e. transformed to inverse depth) using RVL.
   * \param[in] depthImg16UC1 2D `cv::Mat` with the image in inverse depth coding.
   * \param[out] compressed Bytes of the compressed image.
   */
  void encodeRVL(const cv::Mat& depthImg16UC1, std::vector<uint8_t>& compressed) const;

  /**
   * \brief Convert a 16UC1 inverse-depth-coded image into a float direct-coded image.
   * \param[in] invDepthImg The raw image coded with inverse depth.
   * \param[in] compressionConfig Quantization settings.
   * \return The raw image coded with direct depth in floats.
   */
  cv::Mat fromInvDepth(const cv::Mat& invDepthImg,
    const compressed_depth_image_transport::ConfigHeader& compressionConfig) const;

  /**
   * \brief Convert a float direct-coded image to 16UC1 inverse-depth-coded image.
   * \param[in] depthImg The raw direct-coded image.
   * \param[in] depth_max Maximum depth value in meters (1-100).
   * \param[in] depth_quantization Depth value at which the sensor accuracy is 1 m (Kinect: >75) (1-150).
   * \param[out] compressionConfig The quantization parameters struct to be written to the beginning of the compressed
   *                               image.
   * \return The 16UC1 inverse-depth-coded raw image.
   */
  cv::Mat toInvDepth(const cv::Mat& depthImg, double depth_max, double depth_quantization,
    compressed_depth_image_transport::ConfigHeader& compressionConfig) const;
};

}

// //////////
//  C API ///
// //////////

/**
 * \brief Encode the given raw image using `compressedDepth` codec with the given config.
 * 
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 * 
 * \param[in] rawHeight Raw image height, that is, number of rows.
 * \param[in] rawWidth Raw image width, that is, number of columns.
 * \param[in] rawEncoding Raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[in] rawIsBigEndian Is raw image bigendian?
 * \param[in] rawStep Raw image full row length in bytes.
 * \param[in] rawDataLength Length of raw image data in bytes, should be `step * rows`.
 * \param[in] rawData The raw image bytes.
 * \param[in,out] compressedFormatAllocator Allocator for the `format` field of the compressed image.
 * \param[in,out] compressedDataAllocator Allocator for the byte data of the compressed image.
 * \param[in] configFormat Compression format (png or rvl) (Noetic only; Melodic autoselects PNG).
 * \param[in] configDepthMax Maximum depth value in meters (1-100).
 * \param[in] configDepthQuantization Depth value at which the sensor accuracy is 1 m (Kinect: >75) (1-150).
 * \param[in] configPngLevel PNG compression level (1-9).
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, `compressedFormatAllocator` and `compressedDataAllocator`
 *         allocate their buffers and write the output to them. If not, `errorStringAllocator` allocates its buffer
 *         and stores the error string in it.
 * \sa Corresponding C++ API function: `image_transport_codecs::CompressedDepthCodec::encode()`.
 */
extern "C" bool compressedDepthCodecEncode(
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
);

/**
 * \brief Decode the given compressed image using `compressedDepth` codec.
 * 
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] compressedFormat The `format` field of the compressed image.
 * \param[in] compressedDataLength Length of the compressed image data in bytes.
 * \param[in] compressedData Bytes of the compressed image.
 * \param[out] rawHeight Raw image height, that is, number of rows.
 * \param[out] rawWidth Raw image width, that is, number of columns.
 * \param[in,out] rawEncodingAllocator Allocator for raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[out] rawIsBigEndian Is raw image bigendian?
 * \param[out] rawStep Raw image full row length in bytes.
 * \param[in,out] rawDataAllocator Allocator for raw image bytes.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, output parameters are set, `rawEncodingAllocator` and
 *         `rawDataAllocator` allocate their buffers and write the output to them. If not, `errorStringAllocator`
 *         allocates its buffer and stores the error string in it.
 * \sa Corresponding C++ API function: `image_transport_codecs::CompressedDepthCodec::decode()`.
 */
extern "C" bool compressedDepthCodecDecode(
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
);

/**
 * \brief Whether the RVL encoding is available. This should be false on Melodic and true on Noetic.
 * \return Whether the RVL encoding is available.
 */
extern "C" bool compressed_depth_codec_has_rvl();
