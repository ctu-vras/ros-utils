// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for raw transport codec (just a passthrough).
 * \author Martin Pecka
 */

#include <string>

#include <dynamic_reconfigure/Config.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <image_transport_codecs/image_transport_codec_plugin.h>

namespace image_transport_codecs
{

class RawCodecPlugin : public ImageTransportCodecPlugin
{
public:
  void setLogHelper(const cras::LogHelperPtr& logHelper) override
  {
  }

  std::string getTransportName() const override
  {
    return "raw";
  }

  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                           const dynamic_reconfigure::Config&) const override
  {
    cras::ShapeShifter compressed;
    cras::msgToShapeShifter(raw, compressed);
    return compressed;
  }

  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const dynamic_reconfigure::Config&) const override
  {
    try
    {
      return *compressed.instantiate<sensor_msgs::Image>();
    }
    catch (const ros::Exception& e)
    {
      return cras::make_unexpected(cras::format("Invalid shapeshifter passed to raw codec decoder: %s.", e.what()));
    }
  }

  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const override
  {
    if (!matchFormat.empty() && cras::toLower(matchFormat) != "raw")
      return cras::nullopt;

    try
    {
      return CompressedImageContent{"raw", compressed.instantiate<sensor_msgs::Image>()->data};
    }
    catch (const ros::Exception& e)
    {
      return cras::make_unexpected(cras::format("Invalid shapeshifter passed to raw codec decoder: %s.", e.what()));
    }
  }
};

}

PLUGINLIB_EXPORT_CLASS(image_transport_codecs::RawCodecPlugin, image_transport_codecs::ImageTransportCodecPlugin)
