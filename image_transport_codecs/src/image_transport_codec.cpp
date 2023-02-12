/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <list>
#include <memory>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <image_transport_codecs/image_transport_codec.h>

namespace image_transport_codecs
{

ImageTransportCodec::ImageTransportCodec(const cras::LogHelperPtr& logHelper) : cras::HasLogger(logHelper)
{
}

ImageTransportCodec::~ImageTransportCodec() = default;

ImageTransportCodec::EncodeResult ImageTransportCodec::encode(
  const sensor_msgs::Image& raw, const XmlRpc::XmlRpcValue& config) const
{
  dynamic_reconfigure::Config configMsg;
  std::list<std::string> errors;
  if (!cras::convert(config, configMsg, true, &errors))
    return cras::make_unexpected("Invalid encoder config: " + cras::join(errors, " "));
  return this->encode(raw, configMsg);
}

ImageTransportCodec::EncodeResult ImageTransportCodec::encode(const sensor_msgs::Image& raw,
  const ros::NodeHandle& nh, const std::string& param) const
{
  return this->encode(raw, nh.param(param, XmlRpc::XmlRpcValue()));
}

ImageTransportCodec::EncodeResult ImageTransportCodec::encode(const sensor_msgs::Image& raw) const
{
  return this->encode(raw, dynamic_reconfigure::Config());
}

ImageTransportCodec::DecodeResult ImageTransportCodec::decode(const topic_tools::ShapeShifter& compressed) const
{
  return this->decode(compressed, dynamic_reconfigure::Config());
}

ImageTransportCodec::DecodeResult ImageTransportCodec::decode(
  const topic_tools::ShapeShifter& compressed, const XmlRpc::XmlRpcValue& config) const
{
  dynamic_reconfigure::Config configMsg;
  std::list<std::string> errors;
  if (!cras::convert(config, configMsg, true, &errors))
    return cras::make_unexpected("Invalid decoder config: " + cras::join(errors, " "));
  return this->decode(compressed, configMsg);
}

ImageTransportCodec::DecodeResult ImageTransportCodec::decode(const topic_tools::ShapeShifter& compressed,
  const ros::NodeHandle& nh, const std::string& param) const
{
  return this->decode(compressed, nh.param(param, XmlRpc::XmlRpcValue()));
}

ImageTransportCodec::GetCompressedContentResult ImageTransportCodec::getCompressedImageContent(
  const topic_tools::ShapeShifter& compressed) const
{
  return this->getCompressedImageContent(compressed, "");
}

}
