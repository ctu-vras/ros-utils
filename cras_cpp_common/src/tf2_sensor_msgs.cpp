/**
 * \file
 * \brief Transformation tools for sensor_msgs messages.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Geometry>  // needs to be implementation-private as we want -march=native optimizations

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cras_cpp_common/cloud.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_sensor_msgs.h>

namespace cras
{

//! \brief Channels that are considered a part of the XYZ point description.
static const std::unordered_map<std::string, CloudChannelType> XYZ_CHANNELS({
  {"", CloudChannelType::POINT},
});

//! \brief Default assignment of channel prefixes to their type.
static std::unordered_map<std::string, CloudChannelType> DEFAULT_CHANNELS({
  {"", CloudChannelType::POINT},
  {"vp_", CloudChannelType::POINT},
  {"normal_", CloudChannelType::DIRECTION},
});

void registerCloudChannelType(const std::string& channelPrefix, const CloudChannelType type)
{
  DEFAULT_CHANNELS.insert({channelPrefix, type});
}

void unregisterCloudChannelType(const std::string& channelPrefix)
{
  DEFAULT_CHANNELS.erase(channelPrefix);
}

/**
 * \brief Check whether a given field name matches a channel name.
 * \param[in] fieldName Name of a pointcloud field.
 * \param[in] channelName Name of a channel.
 * \param[in] channelType Type of the channel.
 * \return Whether the field belongs to the given channel.
 */
bool fieldNameMatchesChannel(const std::string& fieldName, const std::string& channelName,
  const CloudChannelType channelType)
{
  if (channelType == CloudChannelType::SCALAR)
  {
    return fieldName == channelName;
  }
  else if (channelName.empty())
  {
    return fieldName == "x" || fieldName == "y" || fieldName == "z";
  }
  else
  {
    return fieldName.length() == channelName.length() + 1 && cras::startsWith(fieldName, channelName) && (
      cras::endsWith(fieldName, "x") || cras::endsWith(fieldName, "y") || cras::endsWith(fieldName, "z"));
  }
}

/**
 * \brief Transform the given pointcloud channel using the given transform.
 * \param[in] cloudIn Input cloud.
 * \param[out] cloudOut Output cloud (can be the same as input cloud).
 * \param[in] transform The transform to apply.
 * \param[in] channelPrefix Prefix of the channel.
 * \param[in] type Type of the channel.
 * \note This function cannot be exposed via the header as we can't expose any Eigen types.
 */
void transformChannel(const sensor_msgs::PointCloud2& cloudIn, sensor_msgs::PointCloud2& cloudOut,
  const Eigen::Isometry3f& transform, const std::string& channelPrefix, const CloudChannelType type)
{
  if (numPoints(cloudIn) == 0)
    return;

  if (type == CloudChannelType::SCALAR)
    return;

  CloudConstIter x_in(cloudIn, channelPrefix + "x");
  CloudConstIter y_in(cloudIn, channelPrefix + "y");
  CloudConstIter z_in(cloudIn, channelPrefix + "z");

  CloudIter x_out(cloudOut, channelPrefix + "x");
  CloudIter y_out(cloudOut, channelPrefix + "y");
  CloudIter z_out(cloudOut, channelPrefix + "z");

  Eigen::Vector3f point;

  // the switch has to be outside the for loop for performance reasons
  switch (type)
  {
    case CloudChannelType::POINT:
      for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
      {
        point = transform * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply the whole transform
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      break;
    case CloudChannelType::DIRECTION:
      for (; x_out != x_out.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
      {
        point = transform.linear() * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply only rotation
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      break;
    default:
      break;
  }
}

void transformChannel(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Transform& tf,
  const std::string& channelPrefix, const CloudChannelType type)
{
  const auto transform = tf2::transformToEigen(tf).cast<float>();
  transformChannel(cloud, cloud, transform, channelPrefix, type);
}

sensor_msgs::PointCloud2& transformWithChannels(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
  const geometry_msgs::TransformStamped& tf)
{
  return transformWithChannels(in, out, tf, DEFAULT_CHANNELS);
}

sensor_msgs::PointCloud2& transformWithChannels(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
  const geometry_msgs::TransformStamped& tf, const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  for (const auto& field : in.fields)
  {
    for (const auto& channelAndType : channels)
    {
      const auto& channel = channelAndType.first;
      const auto& channelType = channelAndType.second;
      if (channelType != CloudChannelType::SCALAR && fieldNameMatchesChannel(field.name, channel, channelType))
      {
        channelsPresent.insert(channel);
      }
    }
  }

  out = in;
  out.header = tf.header;

  const auto transform = tf2::transformToEigen(tf).cast<float>();

  for (const auto& channel : channelsPresent)
    transformChannel(in, out, transform, channel, channels.at(channel));

  return out;
}

sensor_msgs::PointCloud2& transformWithChannels(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
  const tf2_ros::Buffer& tfBuffer, const std::string& targetFrame)
{
  return transformWithChannels(in, out, tfBuffer, targetFrame, DEFAULT_CHANNELS);
}

sensor_msgs::PointCloud2& transformWithChannels(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
  const tf2_ros::Buffer& tfBuffer, const std::string& targetFrame,
  const std::unordered_map<std::string, CloudChannelType>& channels)
{
  const auto tf = tfBuffer.lookupTransform(targetFrame, in.header.frame_id, in.header.stamp);
  return transformWithChannels(in, out, tf, channels);
}

sensor_msgs::PointCloud2& transformOnlyChannels(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
  const geometry_msgs::TransformStamped& tf, const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  out.point_step = 0;
  for (const auto& field : in.fields)
  {
    for (const auto& channelAndType : channels)
    {
      const auto& channel = channelAndType.first;
      const auto& channelType = channelAndType.second;
      if (fieldNameMatchesChannel(field.name, channel, channelType))
      {
        channelsPresent.insert(channel);
        out.fields.push_back(field);
        out.fields.back().offset = out.point_step;
        out.point_step += sizeOfPointField(field.datatype);
      }
    }
  }

  out.header = tf.header;
  out.is_dense = in.is_dense;
  out.height = in.height;
  out.width = in.width;
  out.is_bigendian = in.is_bigendian;
  out.row_step = out.width * out.point_step;

  CloudModifier mod(out);
  mod.resize(numPoints(in));

  const auto transform = tf2::transformToEigen(tf).cast<float>();

  for (const auto& channel : channelsPresent)
  {
    const auto channelType = channels.at(channel);
    if (channelType != CloudChannelType::SCALAR)
      transformChannel(in, out, transform, channel, channelType);
    else
      copyChannelData(in, out, channel);
  }

  return out;
}

sensor_msgs::PointCloud2& transformOnlyXYZ(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
  const geometry_msgs::TransformStamped& tf)
{
  return transformOnlyChannels(in, out, tf, XYZ_CHANNELS);
}

}
