/**
 * \file
 * \brief Tools for more convenient working with ShapeShifter objects.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// HACK begin
#include <sstream>
#define private public
#include <cras_topic_tools/shape_shifter.h>
#undef private
// HACK end

#include <boost/shared_ptr.hpp>

#include <ros/common.h>
#include <ros/serialization.h>
#include <std_msgs/Header.h>

#include <cras_cpp_common/optional.hpp>

namespace cras
{

#if ROS_VERSION_MINIMUM(1, 15, 0)

uint8_t* getBuffer(topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf.data();
}

const uint8_t* getBuffer(const topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf.data();
}

void resizeBuffer(topic_tools::ShapeShifter& msg, size_t newLength)
{
  if (newLength == msg.size())
    return;
  // resize() makes sure that if the buffer was reallocated, the start of the old buffer will get copied to the new one
  msg.msgBuf.resize(newLength);
}

void copyShapeShifter(const topic_tools::ShapeShifter& in, topic_tools::ShapeShifter& out)
{
  out = in;
}

#else

uint8_t* getBuffer(topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf;
}

const uint8_t* getBuffer(const topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf;
}

void resizeBuffer(topic_tools::ShapeShifter& msg, size_t newLength)
{
  if (newLength == msg.size())
    return;
  if (newLength < msg.size())
  {
    msg.msgBufUsed = newLength;
  }
  else
  {
    auto oldBuf = msg.msgBuf;
    const auto oldLength = msg.msgBufUsed;
    msg.msgBuf = new uint8_t[newLength];
    msg.msgBufAlloc = newLength;
    msg.msgBufUsed = newLength;
    // Not exactly needed, but to keep compatibility with the Noetic version
    std::memcpy(msg.msgBuf, oldBuf, oldLength);
    delete[] oldBuf;
  }
}

void copyShapeShifter(const topic_tools::ShapeShifter& in, topic_tools::ShapeShifter& out)
{
  out = in;
  out.msgBuf = new uint8_t[in.msgBufUsed];
  out.msgBufAlloc = in.msgBufUsed;
  std::memcpy(out.msgBuf, in.msgBuf, in.msgBufUsed);
}

#endif

size_t getBufferLength(const topic_tools::ShapeShifter& msg)
{
  return msg.size();
}

cras::optional<std_msgs::Header> getHeader(const topic_tools::ShapeShifter& msg)
{
  try
  {
    auto p = boost::make_shared<std_msgs::Header>();

    ros::serialization::IStream s(const_cast<uint8_t*>(cras::getBuffer(msg)), cras::getBufferLength(msg));
    ros::serialization::deserialize(s, *p);

    return *p;
  }
  catch (...)
  {
    return cras::nullopt;
  }
}

bool setHeader(topic_tools::ShapeShifter& msg, std_msgs::Header& header)
{
  const auto oldHeader = cras::getHeader(msg);
  if (!oldHeader.has_value())
    return false;

  const auto oldHeaderLength = ros::serialization::serializationLength(oldHeader.value());
  const auto newHeaderLength = ros::serialization::serializationLength(header);
  const auto oldLength = msg.size();
  const auto newLength = oldLength + newHeaderLength - oldHeaderLength;

  if (oldHeaderLength == newHeaderLength)
  {
    // New message is same length as the old one - we just overwrite the header
    try
    {
      ros::serialization::OStream ostream(cras::getBuffer(msg), cras::getBufferLength(msg));
      ros::serialization::serialize(ostream, header);
      return true;
    }
    catch (...)
    {
      return false;
    }
  }
  else if (newHeaderLength < oldHeaderLength)
  {
    // New message is shorter - we just move the data part to the left
    cras::resizeBuffer(msg, newLength);  // The buffer remains the same.
    auto buffer = cras::getBuffer(msg);
    try
    {
      ros::serialization::OStream ostream(buffer, newLength);
      ros::serialization::serialize(ostream, header);
      std::memmove(buffer + newHeaderLength, buffer + oldHeaderLength, oldLength - oldHeaderLength);
      return true;
    }
    catch (...)
    {
      return false;
    }
  }
  else
  {
    // New message is longer - we need to reallocate the buffer, move its data to the right and then write new header
    cras::resizeBuffer(msg, newLength);  // buffer contents are copied to the reallocated one
    auto buffer = cras::getBuffer(msg);
    std::memmove(buffer + newHeaderLength, buffer + oldHeaderLength, oldLength - oldHeaderLength);
    try
    {
      ros::serialization::OStream ostream(buffer, newLength);
      ros::serialization::serialize(ostream, header);
      return true;
    }
    catch (...)
    {
      return false;
    }
  }
}

#if ROS_VERSION_MINIMUM(1, 15, 0)
#else

ShapeShifter::ShapeShifter() = default;
ShapeShifter::~ShapeShifter() = default;

ShapeShifter::ShapeShifter(const topic_tools::ShapeShifter& other) :
  ShapeShifter(reinterpret_cast<const ShapeShifter&>(other))
{
}

ShapeShifter::ShapeShifter(topic_tools::ShapeShifter&& other) noexcept :
  ShapeShifter(reinterpret_cast<ShapeShifter&&>(other))
{
}

ShapeShifter& ShapeShifter::operator=(const topic_tools::ShapeShifter& other)
{
  return operator=(reinterpret_cast<const ShapeShifter&>(other));
}

ShapeShifter& ShapeShifter::operator=(topic_tools::ShapeShifter&& other) noexcept
{
  return operator=(reinterpret_cast<ShapeShifter&&>(other));
}

ShapeShifter::ShapeShifter(const ShapeShifter& other)
{
  this->md5 = other.md5;
  this->datatype = other.datatype;
  this->msg_def = other.msg_def;
  this->latching = other.latching;
  this->typed = other.typed;
  this->msgBufUsed = this->msgBufAlloc = other.msgBufUsed;
  if (other.msgBuf != nullptr && other.msgBufUsed > 0)
  {
    this->msgBuf = new uint8_t[other.msgBufUsed];
    memcpy(this->msgBuf, other.msgBuf, other.msgBufUsed);
  }
}

ShapeShifter::ShapeShifter(ShapeShifter&& other) noexcept
{
  this->md5 = other.md5;
  this->datatype = other.datatype;
  this->msg_def = other.msg_def;
  this->latching = other.latching;
  this->typed = other.typed;
  this->msgBufUsed = this->msgBufAlloc = other.msgBufUsed;
  if (other.msgBuf != nullptr && other.msgBufUsed > 0)
  {
    this->msgBuf = std::exchange(other.msgBuf, nullptr);
    other.msgBufAlloc = 0;
    other.msgBufUsed = 0;
  }
}

ShapeShifter& ShapeShifter::operator=(const ShapeShifter& other)
{
  if (this == &other)
    return *this;

  this->md5 = other.md5;
  this->datatype = other.datatype;
  this->msg_def = other.msg_def;
  this->latching = other.latching;
  this->typed = other.typed;
  this->msgBufUsed = other.msgBufUsed;
  if (other.msgBuf != nullptr && other.msgBufUsed > 0)
  {
    if (this->msgBufAlloc < other.msgBufUsed)
    {
      delete[] this->msgBuf;
      this->msgBuf = new uint8_t[other.msgBufUsed];
      this->msgBufAlloc = other.msgBufUsed;
    }
    memcpy(this->msgBuf, other.msgBuf, other.msgBufUsed);
  }
  else if (this->msgBuf != nullptr)
  {
    delete[] this->msgBuf;
    this->msgBuf = nullptr;
    this->msgBufAlloc = 0;
  }
  return *this;
}

ShapeShifter& ShapeShifter::operator=(ShapeShifter&& other) noexcept
{
  if (this == &other)
    return *this;

  this->md5 = other.md5;
  this->datatype = other.datatype;
  this->msg_def = other.msg_def;
  this->latching = other.latching;
  this->typed = other.typed;
  std::swap(this->msgBufUsed, other.msgBufUsed);
  std::swap(this->msgBufAlloc, other.msgBufAlloc);
  std::swap(this->msgBuf, other.msgBuf);
  return *this;
}

#endif

}
