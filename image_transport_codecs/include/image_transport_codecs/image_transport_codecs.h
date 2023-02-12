#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin-based interface for compressing and decompressing images using codec plugins.
 * \author Martin Pecka
 */

#include <list>
#include <memory>
#include <string>
#include <unordered_map>

#include <dynamic_reconfigure/Config.h>
#include <pluginlib/class_loader.hpp>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <image_transport_codecs/image_transport_codec.h>
#include <image_transport_codecs/image_transport_codec_plugin.h>

namespace image_transport_codecs
{

/**
 * \brief Plugin-based interface for compressing and decompressing images using codec plugins.
 * 
 * Example usage:
 * ```
 * image_transport_codecs::ImageTransportCodecs codecs;
 * sensor_msgs::Image raw = ...;
 * auto result = codecs.encode(raw, "compressed");
 * if (!result)
 * {
 *   ROS_ERROR_STREAM("Error encoding image: " << result.error();
 *   return false;
 * }
 * sensor_msgs::CompressedImage compressed = result.value();
 * ```
 */
class ImageTransportCodecs : public cras::HasLogger
{
public:
  /**
   * \brief Create the codec interface and load all available codecs.
   * \param[in] log Log helper.
   */
  explicit ImageTransportCodecs(const cras::LogHelperPtr& log = std::make_shared<cras::NodeLogHelper>());

  virtual ~ImageTransportCodecs();

  /**
   * \brief Manually add a codec instance. This is usually not needed as the codecs are autodetected using pluginlib.
   * \param[in] codec The codec to add.
   */
  void addCodec(const ImageTransportCodecPlugin::ConstPtr& codec);

  /**
   * \brief Encode the given raw image into a compressed image shapeshifter.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  virtual ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
    const std::string& topicOrCodec, const dynamic_reconfigure::Config& config);

  /**
   * \brief Decode the given compressed image shapeshifter into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  virtual ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
    const std::string& topicOrCodec, const dynamic_reconfigure::Config& config);

  /**
   * \brief Return the part of the encoded message that represents the actual image data (i.e. the part that can be
   *        passed to external decoders or saved to a file). If the codec messages have no such meaning, empty result
   *        is returned.
   * \param[in] compressed The compressed image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] matchFormat If nonempty, the image data is only returned if their `format` field would match the given
   *                        one. The matching should be case-insensitive.
   * \return If it makes sense, the contained image bytes. If not, empty result. If an error occurred, it is reported
   *         as the unexpected result.
   */
  virtual ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& topicOrCodec, const std::string& matchFormat) const;

  /**
   * \brief Return the part of the encoded message that represents the actual image data (i.e. the part that can be
   *        passed to external decoders or saved to a file). If the codec messages have no such meaning, empty result
   *        is returned.
   * \param[in] compressed The compressed image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \return If it makes sense, the contained image bytes. If not, empty result. If an error occurred, it is reported
   *         as the unexpected result.
   */
  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& topicOrCodec) const;

  /**
   * \brief Encode the given raw image into a  compressed image shapeshifter using the default compression parameters.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw, const std::string& topicOrCodec);

  /**
   * \brief Decode the given compressed image shapeshifter into a raw image using the default decompression parameters.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const std::string& topicOrCodec);

  /**
   * \brief Encode the given raw image into a  compressed image shapeshifter.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the compression (if it has any parameters). Pass a XmlRpc dict.
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                           const std::string& topicOrCodec, const XmlRpc::XmlRpcValue& config);

  /**
   * \brief Decode the given compressed image shapeshifter into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the decompression (if it has any parameters). Pass a XmlRpc dict.
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const std::string& topicOrCodec, const XmlRpc::XmlRpcValue& config);

  /**
   * \brief Encode the given raw image into a  compressed image shapeshifter.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] nh Node handle to get parameters from.
   * \param[in] param Name of the parameter from which config of the compression can be read (if it has any parameters).
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
    const std::string& topicOrCodec, const ros::NodeHandle& nh, const std::string& param);

  /**
   * \brief Decode the given compressed image shapeshifter into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] nh Node handle to get parameters from.
   * \param[in] param Name of the parameter from which config of the decompression can be read (if it has any
   *                  parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
    const std::string& topicOrCodec, const ros::NodeHandle& nh, const std::string& param);

  /**
   * \brief Encode the given raw image into a  compressed image shapeshifter.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding image_transport publisher.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  template<typename Config>
  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
    const std::string& topicOrCodec, const Config& config)
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->encode(raw, topicOrCodec, configMsg);
  }

  /**
   * \brief Decode the given compressed image shapeshifter into a raw image.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding image_transport subscriber.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename Config>
  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const std::string& topicOrCodec, const Config& config)
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->decode(compressed, topicOrCodec, configMsg);
  }

  /**
   * \brief Encode the given raw image into a compressed image.
   * \tparam M Type of the compressed message.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The compressed image message (if encoding succeeds), or an error message.
   */
  template<typename M>
  inline cras::expected<M, std::string> encodeTyped(const sensor_msgs::Image& raw, const std::string& topicOrCodec,
                                                    const dynamic_reconfigure::Config& config)
  {
    const auto shifter = this->encode(raw, topicOrCodec, config);
    if (!shifter)
      return cras::make_unexpected(shifter.error());

    try
    {
      return *shifter->instantiate<M>();
    }
    catch (const ros::Exception& e)
    {
      return cras::make_unexpected(cras::format("Invalid shapeshifter returned from encoder: %s.", e.what()));
    }
  }

  /**
   * \brief Decode the given compressed image into a raw image.
   * \tparam M Type of the compressed message.
   * \param[in] compressed The compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename M>
  inline ImageTransportCodec::DecodeResult decodeTyped(const M& compressed, const std::string& topicOrCodec,
                                                       const dynamic_reconfigure::Config& config)
  {
    topic_tools::ShapeShifter shifter;
    cras::msgToShapeShifter(compressed, shifter);
    return this->decode(shifter, topicOrCodec, config);
  }

  /**
   * \brief Encode the given raw image into a compressed image using the default compression parameters.
   * \tparam M Type of the compressed message.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \return The compressed image message (if encoding succeeds), or an error message.
   */
  template<typename M>
  inline cras::expected<M, std::string> encodeTyped(const sensor_msgs::Image& raw, const std::string& topicOrCodec)
  {
    return this->encodeTyped<M>(raw, topicOrCodec, dynamic_reconfigure::Config());
  }

  /**
   * \brief Decode the given compressed image into a raw image using the default decompression parameters.
   * \tparam M Type of the compressed message.
   * \param[in] compressed The compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename M>
  inline ImageTransportCodec::DecodeResult decodeTyped(const M& compressed, const std::string& topicOrCodec)
  {
    return this->decodeTyped(compressed, topicOrCodec, dynamic_reconfigure::Config());
  }

  /**
   * \brief Encode the given raw image into a compressed image.
   * \tparam M Type of the compressed message.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the compression (if it has any parameters). Pass a XmlRpc dict.
   * \return The compressed image message (if encoding succeeds), or an error message.
   */
  template<typename M>
  inline cras::expected<M, std::string> encodeTyped(const sensor_msgs::Image& raw, const std::string& topicOrCodec,
                                                    const XmlRpc::XmlRpcValue& config)
  {
    dynamic_reconfigure::Config configMsg;
    std::list<std::string> errors;
    if (!cras::convert(config, configMsg, true, &errors))
      return cras::make_unexpected("Invalid encoder config: " + cras::join(errors, " "));

    return this->encodeTyped<M>(raw, topicOrCodec, configMsg);
  }

  /**
   * \brief Decode the given compressed image into a raw image.
   * \tparam M Type of the compressed message.
   * \param[in] compressed The compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the decompression (if it has any parameters). Pass a XmlRpc dict.
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename M>
  inline ImageTransportCodec::DecodeResult decodeTyped(const M& compressed, const std::string& topicOrCodec,
                                                       const XmlRpc::XmlRpcValue& config)
  {
    dynamic_reconfigure::Config configMsg;
    std::list<std::string> errors;
    if (!cras::convert(config, configMsg, true, &errors))
      return cras::make_unexpected("Invalid decoder config: " + cras::join(errors, " "));

    return this->decodeTyped(compressed, topicOrCodec, configMsg);
  }

  /**
   * \brief Encode the given raw image into a compressed image.
   * \tparam M Type of the compressed message.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] nh Node handle to get parameters from.
   * \param[in] param Name of the parameter from which config of the compression can be read (if it has any parameters).
   * \return The compressed image message (if encoding succeeds), or an error message.
   */
  template<typename M>
  inline cras::expected<M, std::string> encodeTyped(const sensor_msgs::Image& raw, const std::string& topicOrCodec,
                                                    const ros::NodeHandle& nh, const std::string& param)
  {
    return this->encodeTyped<M>(raw, topicOrCodec, nh.param(param, XmlRpc::XmlRpcValue()));
  }

  /**
   * \brief Decode the given compressed image into a raw image.
   * \tparam M Type of the compressed message.
   * \param[in] compressed The compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] nh Node handle to get parameters from.
   * \param[in] param Name of the parameter from which config of the decompression can be read (if it has any
   *                  parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename M>
  inline ImageTransportCodec::DecodeResult decodeTyped(const M& compressed, const std::string& topicOrCodec,
                                                       const ros::NodeHandle& nh, const std::string& param)
  {
    return this->encodeTyped(compressed, topicOrCodec, nh.param(param, XmlRpc::XmlRpcValue()));
  }

  /**
   * \brief Encode the given raw image into a compressed image.
   * \tparam M Type of the compressed message.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding image_transport publisher.
   * \param[in] raw The input raw image.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
   *                         compressed message will be published (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The compressed image message (if encoding succeeds), or an error message.
   */
  template<typename M, typename Config>
  inline cras::expected<M, std::string> encodeTyped(const sensor_msgs::Image& raw, const std::string& topicOrCodec,
                                                    const Config& config)
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->encodeTyped<M>(raw, topicOrCodec, configMsg);
  }

  /**
   * \brief Decode the given compressed image into a raw image.
   * \tparam M Type of the compressed message.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding image_transport publisher.
   * \param[in] compressed The compressed image to be decoded.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
   *                         compressed message was received (so that it is possible to parse the codec from the
   *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
   *                         you probably do not want.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename M, typename Config>
  inline ImageTransportCodec::DecodeResult decodeTyped(const M& compressed, const std::string& topicOrCodec,
                                                       const Config& config)
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->decodeTyped(compressed, topicOrCodec, configMsg);
  }

protected:
  /**
   * \brief Load all codecs available via pluginlib. This function can be called multiple times without negative
   *        performance impact - the loading is only done for the first time.
   */
  void loadCodecs();

  /**
   * \brief Parse the name of the codec from a topic.
   * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of an image topic.
   * \return Name of the codec, or empty string if not found.
   */
  std::string parseTransport(const std::string& topicOrCodec) const;

  std::unique_ptr<pluginlib::ClassLoader<ImageTransportCodecPlugin>> loader;  //!< \brief Pluginlib loader of codecs.
  std::unordered_map<std::string, ImageTransportCodecPlugin::ConstPtr> codecs;  //!< \brief Loaded codecs.
};

}

/////////////
/// C API ///
/////////////

/**
 * \brief Encode the given raw image using an automatically determined codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic where the
 *                         compressed message will be published (so that it is possible to parse the codec from the
 *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
 *                         you probably do not want.
 * \param[in] rawHeight Raw image height, that is, number of rows.
 * \param[in] rawWidth Raw image width, that is, number of columns.
 * \param[in] rawEncoding Raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[in] rawIsBigEndian Is raw image bigendian?
 * \param[in] rawStep Raw image full row length in bytes.
 * \param[in] rawDataLength Length of raw image data in bytes, should be `step * rows`.
 * \param[in] rawData The raw image bytes.
 * \param[in,out] compressedTypeAllocator Allocator for the string version of the type of the compressed message.
 * \param[in,out] md5SumAllocator Allocator for the MD5 sum of the compressed message.
 * \param[in,out] compressedDataAllocator Allocator for the byte data of the compressed image.
 * \param[in] serializedConfigLength Length of the `dynamic_reconfigure::Config` serialization.
 * \param[in] serializedConfig Bytes of the serialized `dynamic_reconfigure::Config`.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, `compressedFormatAllocator` and `compressedDataAllocator`
 *         allocate their buffers and write the output to them. If not, `errorStringAllocator` allocates its buffer
 *         and stores the error string in it.
 * \sa Corresponding C++ API function: `image_transport_codecs::ImageTransportCodecs::encode()`.
 */
extern "C" bool imageTransportCodecsEncode(
  const char* topicOrCodec,
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  cras::allocator_t compressedTypeAllocator,
  cras::allocator_t md5SumAllocator,
  cras::allocator_t compressedDataAllocator,
  size_t serializedConfigLength,
  const uint8_t serializedConfig[],
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);

/**
 * \brief Decode the given compressed image using `compressed` codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
 *                         compressed message was received (so that it is possible to parse the codec from the
 *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
 *                         you probably do not want.
 * \param[in] compressedType The string version of the type of the compressed message.
 * \param[in] compressedMd5sum The MD5 sum of the compressed message.
 * \param[in] compressedDataLength Length of the compressed image data in bytes.
 * \param[in] compressedData Bytes of the compressed image.
 * \param[out] rawHeight Raw image height, that is, number of rows.
 * \param[out] rawWidth Raw image width, that is, number of columns.
 * \param[in,out] rawEncodingAllocator Allocator for raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[out] rawIsBigEndian Is raw image bigendian?
 * \param[out] rawStep Raw image full row length in bytes.
 * \param[in,out] rawDataAllocator Allocator for raw image bytes.
 * \param[in] serializedConfigLength Length of the `dynamic_reconfigure::Config` serialization.
 * \param[in] serializedConfig Bytes of the serialized `dynamic_reconfigure::Config`.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, output parameters are set, `rawEncodingAllocator` and
 *         `rawDataAllocator` allocate their buffers and write the output to them. If not, `errorStringAllocator`
 *         allocates its buffer and stores the error string in it.
 * \sa Corresponding C++ API function: `image_transport_codecs::ImageTransportCodecs::decode()`.
 */
extern "C" bool imageTransportCodecsDecode(
  const char* topicOrCodec,
  const char* compressedType,
  const char* compressedMd5sum,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  sensor_msgs::Image::_height_type& rawHeight,
  sensor_msgs::Image::_width_type& rawWidth,
  cras::allocator_t rawEncodingAllocator,
  sensor_msgs::Image::_is_bigendian_type& rawIsBigEndian,
  sensor_msgs::Image::_step_type& rawStep,
  cras::allocator_t rawDataAllocator,
  size_t serializedConfigLength,
  const uint8_t serializedConfig[],
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);

/**
 * \brief Return the part of the encoded message that represents the actual image data (i.e. the part that can be
 *        passed to external decoders or saved to a file). If the codec messages have no such meaning, empty result
 *        is returned.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] topicOrCodec Either the output of a codec's `getTransportName()`, or name of a topic from which the
 *                         compressed message was received (so that it is possible to parse the codec from the
 *                         topic). Do not pass the raw image topic - that would result in using `RawCodecPlugin` which
 *                         you probably do not want.
 * \param[in] compressedType The string version of the type of the compressed message.
 * \param[in] compressedMd5sum The MD5 sum of the compressed message.
 * \param[in] compressedDataLength Length of the compressed image data in bytes.
 * \param[in] compressedData Bytes of the compressed image.
 * \param[in] matchFormat If nonempty, the image data is only returned if their `format` field would match the given
 *                        one. The matching should be case-insensitive.
 * \param[out] hasData Whether some content data were found in the image.
 * \param[in,out] formatAllocator Allocator for content format string.
 * \param[in,out] dataAllocator Allocator for the content bytes.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the function has succeeded. If yes, output parameters are set, `formatAllocator` and
 *         `dataAllocator` allocate their buffers and write the output to them. If not, `errorStringAllocator`
 *         allocates its buffer and stores the error string in it.
 * \sa Corresponding C++ API function: `image_transport_codecs::ImageTransportCodecs::getCompressedImageContent()`.
 */
extern "C" bool getCompressedImageContents(
  const char* topicOrCodec,
  const char* compressedType,
  const char* compressedMd5sum,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  const char* matchFormat,
  bool& hasData,
  cras::allocator_t formatAllocator,
  cras::allocator_t dataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);
