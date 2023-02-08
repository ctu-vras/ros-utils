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

#include <dynamic_reconfigure/Config.h>
#include <pluginlib/class_loader.hpp>
#include <ros/node_handle.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <image_transport_codecs/image_transport_codec_plugin.h>
#include <image_transport_codecs/image_transport_codecs.h>

namespace image_transport_codecs
{

ImageTransportCodecs::ImageTransportCodecs(const cras::LogHelperPtr& log) : cras::HasLogger(log)
{
  this->loadCodecs();
}

ImageTransportCodecs::~ImageTransportCodecs()
{
}

void ImageTransportCodecs::loadCodecs()
{
  if (this->loader != nullptr)
    return;

  this->loader = std::make_unique<pluginlib::ClassLoader<ImageTransportCodecPlugin>>(
    "image_transport_codecs", "image_transport_codecs::ImageTransportCodecPlugin");

  for (const std::string& lookupName : this->loader->getDeclaredClasses())
  {
    try
    {
      auto codec = this->loader->createInstance(lookupName);
      codec->setLogHelper(this->log);
      this->addCodec(codec);
    }
    catch (const pluginlib::LibraryLoadException& e)
    {
      CRAS_ERROR("Error loading codec plugin library for codec %s: %s.", lookupName.c_str(), e.what());
    }
    catch (const pluginlib::CreateClassException& e)
    {
      CRAS_ERROR("Codec plugin library for %s found, but instantiating the class failed: %s.",
                 lookupName.c_str(), e.what());
    }
  }
}

void ImageTransportCodecs::addCodec(const ImageTransportCodecPlugin::ConstPtr& codec)
{
  const auto transportName = codec->getTransportName();
  this->codecs[transportName] = codec;
}

std::string ImageTransportCodecs::parseTransport(const std::string& topicOrCodec)
{
  const auto& splits = cras::split(topicOrCodec, "/");

  // Directly specified transport.
  if (splits.size() == 1)
    return topicOrCodec;

  // Transport is the last part after a slash
  const auto& transport = splits[splits.size() - 1];
  if (this->codecs.find(transport) != this->codecs.end())
    return transport;

  // If the last part after slash is not known, fail
  return "";
}

ImageTransportCodec::EncodeResult ImageTransportCodecs::encode(const sensor_msgs::Image& raw,
  const std::string& topicOrCodec, const dynamic_reconfigure::Config& config)
{
  const auto& transport = this->parseTransport(topicOrCodec);
  if (this->codecs.find(transport) == this->codecs.end())
    return cras::make_unexpected("Could not find any codec for " + topicOrCodec + ".");

  const auto& codec = this->codecs[transport];
  return codec->encode(raw, config);
}

ImageTransportCodec::EncodeResult ImageTransportCodecs::encode(const sensor_msgs::Image& raw,
  const std::string& topicOrCodec, const XmlRpc::XmlRpcValue& config)
{
  dynamic_reconfigure::Config c;
  std::list<std::string> errors;
  if (!cras::convert(config, c, true, &errors))
    return cras::make_unexpected("Invalid encoder config: " + cras::join(errors, " "));
  return this->encode(raw, topicOrCodec, c);
}

ImageTransportCodec::EncodeResult ImageTransportCodecs::encode(const sensor_msgs::Image& raw,
                                                               const std::string& topicOrCodec)
{
  return this->encode(raw, topicOrCodec, dynamic_reconfigure::Config());
}

ImageTransportCodec::DecodeResult ImageTransportCodecs::decode(const topic_tools::ShapeShifter& compressed,
  const std::string& topicOrCodec)
{
  return this->decode(compressed, topicOrCodec, dynamic_reconfigure::Config());
}

ImageTransportCodec::EncodeResult ImageTransportCodecs::encode(const sensor_msgs::Image& raw,
  const std::string& topicOrCodec, const ros::NodeHandle& nh, const std::string& param)
{
  return this->encode(raw, topicOrCodec, nh.param(param, XmlRpc::XmlRpcValue()));
}

ImageTransportCodec::DecodeResult ImageTransportCodecs::decode(const topic_tools::ShapeShifter& compressed,
  const std::string& topicOrCodec, const dynamic_reconfigure::Config& config)
{
  const auto& transport = this->parseTransport(topicOrCodec);
  if (this->codecs.find(transport) == this->codecs.end())
    return cras::make_unexpected("Could not find any codec for " + topicOrCodec + ".");

  const auto& codec = this->codecs[transport];
  return codec->decode(compressed, config);
}

ImageTransportCodec::DecodeResult ImageTransportCodecs::decode(const topic_tools::ShapeShifter& compressed,
  const std::string& topicOrCodec, const XmlRpc::XmlRpcValue& config)
{
  dynamic_reconfigure::Config c;
  std::list<std::string> errors;
  if (!cras::convert(config, c, true, &errors))
    return cras::make_unexpected("Invalid decoder config: " + cras::join(errors, " "));
  return this->decode(compressed, topicOrCodec, c);
}

ImageTransportCodec::DecodeResult ImageTransportCodecs::decode(const topic_tools::ShapeShifter& compressed,
  const std::string& topicOrCodec, const ros::NodeHandle& nh, const std::string& param)
{
  return this->decode(compressed, topicOrCodec, nh.param(param, XmlRpc::XmlRpcValue()));
}

thread_local auto globalLogger = std::make_shared<cras::MemoryLogHelper>();
thread_local ImageTransportCodecs image_transport_codecs_instance(globalLogger);

}

bool imageTransportCodecsEncode(
  const char* topicOrCodec,
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  cras::allocator_t compressedTypeAllocator,
  cras::allocator_t compressedMd5SumAllocator,
  cras::allocator_t compressedDataAllocator,
  size_t serializedConfigLength,
  const uint8_t serializedConfig[],
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
)
{
  dynamic_reconfigure::Config config;
  if (serializedConfigLength > 0)
  {
    ros::serialization::IStream data(const_cast<uint8_t*>(serializedConfig), serializedConfigLength);
    try
    {
      ros::serialization::deserialize(data, config);
    }
    catch (const ros::Exception& e)
    {
      cras::outputString(errorStringAllocator, cras::format("Could not deserialize encoder config: %s.", e.what()));
      return false;
    }
  }

  sensor_msgs::Image raw;
  raw.height = rawHeight;
  raw.width = rawWidth;
  raw.encoding = rawEncoding;
  raw.is_bigendian = rawIsBigEndian;
  raw.step = rawStep;
  raw.data.resize(rawDataLength);
  memcpy(raw.data.data(), rawData, rawDataLength);

  image_transport_codecs::globalLogger->clear();

  const auto compressed = image_transport_codecs::image_transport_codecs_instance.encode(raw, topicOrCodec, config);

  for (const auto& msg : image_transport_codecs::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  image_transport_codecs::globalLogger->clear();

  if (!compressed)
  {
    cras::outputString(errorStringAllocator, compressed.error());
    return false;
  }

  cras::outputString(compressedTypeAllocator, compressed->getDataType());
  cras::outputString(compressedMd5SumAllocator, compressed->getMD5Sum());
  cras::outputByteBuffer(compressedDataAllocator, cras::getBuffer(compressed.value()), compressed->size());

  return true;
}

bool imageTransportCodecsDecode(
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
)
{
  dynamic_reconfigure::Config config;
  if (serializedConfigLength > 0)
  {
    ros::serialization::IStream data(const_cast<uint8_t*>(serializedConfig), serializedConfigLength);
    try
    {
      ros::serialization::deserialize(data, config);
    }
    catch (const ros::Exception& e)
    {
      cras::outputString(errorStringAllocator, cras::format("Could not deserialize decoder config: %s.", e.what()));
      return false;
    }
  }

  topic_tools::ShapeShifter compressed;
  compressed.morph(compressedMd5sum, compressedType, "", "");
  cras::resizeBuffer(compressed, compressedDataLength);
  memcpy(cras::getBuffer(compressed), compressedData, compressedDataLength);

  image_transport_codecs::globalLogger->clear();

  const auto raw = image_transport_codecs::image_transport_codecs_instance.decode(compressed, topicOrCodec, config);

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
