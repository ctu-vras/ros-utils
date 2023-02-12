#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin interface to image transport codecs which allows automatic selection of a suitable codec by
 *        `ImageTransportCodecs` class.
 * \author Martin Pecka
 */

#include <string>

#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/Image.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/log_utils.h>
#include <image_transport_codecs/image_transport_codec.h>

namespace image_transport_codecs
{

/**
 * \brief Basic interface of an image transport codec plugin.
 * 
 * \note Do not forget to register the codec plugin library via pluginlib:
 *       - terminology: "codec library" is the C++ library implementing the `ImageTransportCodec` interface. "codec
 *         plugin library" is a different library (cannot be the same!) which implements the `ImageTransportCodecPlugin`
 *         interface.
 *       - put this at the beginning of the plugin .cpp file: `#include <pluginlib/class_list_macros.h>`
 *       - put this to the end of the plugin .cpp file:
 *         `PLUGINLIB_EXPORT_CLASS(<PLUGIN>, image_transport_codecs::ImageTransportCodecPlugin)`
 *         - substitute `<PLUGIN>` with the fully qualified name of your plugin class
 *       - do NOT add the codec plugin library to `catkin_package(LIBRARIES...)` in your `CMakeLists.txt`.
 *       - DO add the codec (not plugin) library to `catkin_package(LIBRARIES...)` in your `CMakeLists.txt`.
 *       - implement the plugin to be just a thin wrapper around the codec library (`ImageTransportCodecPluginBase` can
 *         make that really easy)
 *       - add `<exec_depend>image_transport_codecs</exec_depend>` to your package.xml
 *       - add `<export><image_transport_codecs plugin="${prefix}/plugins.xml" /></export>` to your package.xml
 *       - create file `plugins.xml` in the root of your package with the following content
 * ```
 * <library path="lib/libPLUGIN_LIB_NAME">
 *     <class name="image_transport_codecs/CODEC" type="PLUGIN"
 *            base_class_type="image_transport_codecs::ImageTransportCodecPlugin">
 *         <description>MY PLUGIN DESCRIPTION</description>
 *     </class>
 * </library>
 * ```
 *         - substitute `CODEC` with the topic suffix your codec uses
 *         - substitute `PLUGIN` with the fully qualified name of your codec plugin class
 *         - substitute `PLUGIN_LIB_NAME` with the name of the dynamic library your codec plugin uses
 *         - substitute `MY PLUGIN DESCRIPTION` with some meaningful description of the codec
 *       - do not forget to install the codec library, codec plugin library, includes of the codec library and
 *         `plugins.xml` files in your `CMakeLists.txt`.
 */
class ImageTransportCodecPlugin
{
public:
  //! \brief Shared pointer to `ImageTransportCodecPlugin`.
  typedef boost::shared_ptr<ImageTransportCodecPlugin> Ptr;

  //! \brief Shared pointer to `const ImageTransportCodecPlugin`.
  typedef boost::shared_ptr<const ImageTransportCodecPlugin> ConstPtr;

  virtual ~ImageTransportCodecPlugin() = default;

  /**
   * \brief Use the given log helper for logging messages.
   * \param[in] logHelper The log helper to use.
   */
  virtual void setLogHelper(const cras::LogHelperPtr& logHelper) = 0;

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
  virtual ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                                   const dynamic_reconfigure::Config& config) const = 0;

  /**
   * \brief Decode the given compressed image into a raw image.
   * \param[in] compressed The shapeshifter of the compressed image to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw image (if decoding succeeds), or an error message.
   */
  virtual ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                                   const dynamic_reconfigure::Config& config) const = 0;

  /**
   * \brief Return the part of the encoded message that represents the actual image data (i.e. the part that can be
   *        passed to external decoders or saved to a file). If the codec messages have no such meaning, empty result
   *        is returned.
   * \param[in] compressed The compressed image.
   * \param[in] matchFormat If nonempty, the image data is only returned if their `format` field would match the given
   *                        one. The matching should be case-insensitive.
   * \return If it makes sense, the contained image bytes. If not, empty result. If an error occurred, it is reported
   *         as the unexpected result.
   */
  virtual ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const = 0;
};

/**
 * \brief Convenience class for implementing image transport codec plugins, which just relays the whole API to a `Codec`
 *        library.
 * \tparam Codec Class of the codec.
 * 
 * If used properly, the whole code of the codec plugin library can be as short as:
 * ```
 * #include <pluginlib/class_list_macros.h>

 * #include <image_transport_codecs/codecs/compressed_codec.h>
 * #include <image_transport_codecs/image_transport_codec_plugin.h>
 * 
 * namespace image_transport_codecs
 * {
 * 
 * class CompressedImageTransportCodecPlugin : public ImageTransportCodecPluginBase<CompressedCodec> {};
 * 
 * }
 * 
 * PLUGINLIB_EXPORT_CLASS(image_transport_codecs::CompressedImageTransportCodecPlugin,
 *                        image_transport_codecs::ImageTransportCodecPlugin)
 * ```
 */
template<typename Codec>
class ImageTransportCodecPluginBase : public ImageTransportCodecPlugin
{
public:
  void setLogHelper(const cras::LogHelperPtr& logHelper) override
  {
    this->codec.setCrasLogger(logHelper);
  }

  std::string getTransportName() const override
  {
    return this->codec.getTransportName();
  }

  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                           const dynamic_reconfigure::Config& config) const override
  {
    return this->codec.encode(raw, config);
  }

  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const dynamic_reconfigure::Config& config) const override
  {
    return this->codec.decode(compressed, config);
  }

  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const override
  {
    return this->codec.getCompressedImageContent(compressed, matchFormat);
  }

private:
  Codec codec;  //!< \brief The codec used by this plugin.
};

}
