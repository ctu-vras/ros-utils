#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief %Base for all image transport codecs.
 * \author Martin Pecka
 */

#include <string>

#include <dynamic_reconfigure/Config.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <topic_tools/shape_shifter.h>
#include <xmlrpcpp/XmlRpc.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_topic_tools/shape_shifter.h>

namespace image_transport_codecs
{

/**
 * \brief %Base for all image transport codecs. All codecs have to extend class `ImageTransportCodec` and implement the
 *        pure virtual methods `std::string getTransportName() const`,
 *        `encode(const sensor_msgs::Image&, const dynamic_reconfigure::Config&) const` and
 *        `decode(const topic_tools::ShapeShifter&, const dynamic_reconfigure::Config&) const`.
 *        There are other convenience methods provided, but all of them are based on these implementations.
 *        
 * Each codec should be implemented as a standalone library for direct use, and also as a plugin inheriting from
 * `ImageTransportCodecPlugin` for generic use via `ImageTransportCodecs`.
 *
 * \parblock
 * \note As there is no common superclass for ROS messages and each codec can output compressed messages of different
 * type, the generic interface for the compressed messages is `ShapeShifter`. The good thing is it offers a really
 * generic way of working with messages. The bad thing is that it does that through (de)serialization of the messages,
 * which might become a bottleneck in high-performance applications. Therefore, if you know the codec in advance, you
 * should directly use the concrete codec and not this generic interface.
 * \endparblock
 *
 * \parblock
 * \note The `EncodeResult` and `DecodeResult` types used for outputs of the API functions make use of `cras::expected`,
 * which is an implementation of the proposed `std::expected` template. Generally, `expected` can either hold the result
 * (in the expected case), or an error (in the unexpected case). So after getting the encoding/decoding result, you
 * always first have to check it for validity (`if (!result) return false`). On a valid result, you can call
 * `cras::expected::value()` to get the expected object. On an invalid result, you can call `cras::expected::error()` to
 * get the error message.
 * \endparblock
 *
 * \parblock
 * \note Throughout this interface, you will see usages of `cras::ShapeShifter` instead of `topic_tools::ShapeShifter`.
 * These two are the same API-wise, but `cras::ShapeShifter` contains a
 * [super-important bugfix](https://github.com/ros/ros_comm/pull/1722) without which your programs would quickly
 * segfault. Generally, it is not safe to make copies of `topic_tools::ShapeShifter` (not even returning it from
 * functions!). `cras::ShapeShifter` makes copying possible. Note that it is safe to pass around references to
 * `topic_tools::ShapeShifter` (but you can never copy the objects).
 * \endparblock
 */
class ImageTransportCodec : public cras::HasLogger
{
public:
  //! \brief Shared pointer to `ImageTransportCodec`.
  typedef boost::shared_ptr<ImageTransportCodec> Ptr;

  //! \brief Shared pointer to `const ImageTransportCodec`.
  typedef boost::shared_ptr<const ImageTransportCodec> ConstPtr;

  // There has to be cras::ShapeShifter instead of topic_tools::ShapeShifter to avoid Melodic memory corruption issues.
  //! \brief Result of image encoding. Either a shapeshifter holding the compressed message, or error message.
  typedef cras::expected<cras::ShapeShifter, std::string> EncodeResult;

  //! \brief Result of image decoding. Either a `sensor_msgs::Image` holding the raw message, or error message.
  typedef cras::expected<sensor_msgs::Image, std::string> DecodeResult;

  /**
   * \brief Create an instance of the codec.
   * \param[in] logHelper The logger to use for error messages not directly related to the currently processed image.
   */
  explicit ImageTransportCodec(const cras::LogHelperPtr& logHelper);

  virtual ~ImageTransportCodec();

  /**
   * \brief Get the name of the codec/transport (used e.g. as topic suffix).
   * \note This name has to be unique among all defined codecs.
   * \return The transport name.
   */
  virtual std::string getTransportName() const = 0;

  /**
   * \brief Encode the given raw image into the given shapeshifter object.
   * \param[in] raw The input raw image.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  virtual EncodeResult encode(const sensor_msgs::Image& raw, const dynamic_reconfigure::Config& config) const = 0;

  /**
   * \brief Decode the given compressed image into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  virtual DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                              const dynamic_reconfigure::Config& config) const = 0;

  /**
   * \brief Encode the given raw image into the given shapeshifter object using the default compression parameters.
   * \param[in] raw The input raw image.
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  EncodeResult encode(const sensor_msgs::Image& raw) const;

  /**
   * \brief Decode the given compressed image into a raw image using the default decompression parameters.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  DecodeResult decode(const topic_tools::ShapeShifter& compressed) const;

  /**
   * \brief Encode the given raw image into the given shapeshifter object.
   * \param[in] raw The input raw image.
   * \param[in] config Config of the compression (if it has any parameters). Pass a XmlRpc dict.
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  EncodeResult encode(const sensor_msgs::Image& raw, const XmlRpc::XmlRpcValue& config) const;

  /**
   * \brief Decode the given compressed image into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters). Pass a XmlRpc dict.
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  DecodeResult decode(const topic_tools::ShapeShifter& compressed, const XmlRpc::XmlRpcValue& config) const;

  /**
   * \brief Encode the given raw image into the given shapeshifter object.
   * \param[in] raw The input raw image.
   * \param[in] nh Node handle to get parameters from.
   * \param[in] param Name of the parameter from which config of the compression can be read (if it has any parameters).
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  EncodeResult encode(const sensor_msgs::Image& raw, const ros::NodeHandle& nh, const std::string& param) const;

  /**
   * \brief Decode the given compressed image into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] nh Node handle to get parameters from.
   * \param[in] param Name of the parameter from which config of the decompression can be read (if it has any
   *                  parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  DecodeResult decode(const topic_tools::ShapeShifter& compressed, const ros::NodeHandle& nh,
                      const std::string& param) const;

  /**
   * \brief Encode the given raw image into the given shapeshifter object.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding image_transport publisher.
   * \param[in] raw The input raw image.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed image message (if encoding succeeds), or an error message.
   */
  template<typename Config>
  EncodeResult encode(const sensor_msgs::Image& raw, const Config& config) const
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->encode(raw, configMsg);
  }

  /**
   * \brief Decode the given compressed image into a raw image.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding image_transport subscriber.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  template<typename Config>
  DecodeResult decode(const topic_tools::ShapeShifter& compressed, const Config& config) const
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->decode(compressed, configMsg);
  }
};

}
