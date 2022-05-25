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
const uint8_t* getBuffer(const topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf.data();
}

size_t getBufferLength(const topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf.size();
}
#else
const uint8_t* getBuffer(const topic_tools::ShapeShifter& msg)
{
  return msg.msgBuf;
}

size_t getBufferLength(const topic_tools::ShapeShifter& msg)
{
  return msg.msgBufUsed;
}
#endif

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

}