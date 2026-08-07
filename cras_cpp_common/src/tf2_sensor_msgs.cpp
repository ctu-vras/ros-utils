// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Transformation tools for sensor_msgs messages.
 * \author Martin Pecka
 */

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Geometry>  // needs to be implementation-private as we want -march=native optimizations

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cras_cpp_common/cloud.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_sensor_msgs.hpp>
#include <cras_cpp_common/time_utils.hpp>

namespace cras {

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

void registerCloudChannelType(const std::string& channel_prefix, const CloudChannelType type) {
  DEFAULT_CHANNELS.insert({channel_prefix, type});
}

void unregisterCloudChannelType(const std::string& channel_prefix) {
  DEFAULT_CHANNELS.erase(channel_prefix);
}

/**
 * \brief Check whether a given field name matches a channel name.
 * \param[in] field_name Name of a pointcloud field.
 * \param[in] channel_name Name of a channel.
 * \param[in] channel_type Type of the channel.
 * \return Whether the field belongs to the given channel.
 */
bool fieldNameMatchesChannel(
    const std::string& field_name, const std::string& channel_name, const CloudChannelType channel_type) {
  if (channel_type == CloudChannelType::SCALAR) {
    return field_name == channel_name;
  } else if (channel_name.empty()) {
    return field_name == "x" || field_name == "y" || field_name == "z";
  } else {
    return field_name.length() == channel_name.length() + 1 && cras::startsWith(field_name, channel_name) && (
      cras::endsWith(field_name, "x") || cras::endsWith(field_name, "y") || cras::endsWith(field_name, "z"));
  }
}

/**
 * \brief Transform the given pointcloud channel using the given transform.
 * \param[in] cloud_in Input cloud.
 * \param[out] cloud_out Output cloud (can be the same as input cloud).
 * \param[in] transform The transform to apply.
 * \param[in] channel_prefix Prefix of the channel.
 * \param[in] type Type of the channel.
 * \note This function cannot be exposed via the header as we can't expose any Eigen types.
 */
void transformChannel(
    const sensor_msgs::msg::PointCloud2& cloud_in, sensor_msgs::msg::PointCloud2& cloud_out,
    const Eigen::Isometry3f& transform, const std::string& channel_prefix, const CloudChannelType type) {
  if (numPoints(cloud_in) == 0) {
    return;
  }

  if (type == CloudChannelType::SCALAR) {
    return;
  }

  CloudConstIter x_in(cloud_in, channel_prefix + "x");
  CloudConstIter y_in(cloud_in, channel_prefix + "y");
  CloudConstIter z_in(cloud_in, channel_prefix + "z");

  CloudIter x_out(cloud_out, channel_prefix + "x");
  CloudIter y_out(cloud_out, channel_prefix + "y");
  CloudIter z_out(cloud_out, channel_prefix + "z");

  Eigen::Vector3f point;

  // the switch has to be outside the for loop for performance reasons
  switch (type) {
    case CloudChannelType::POINT:
      for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
        point = transform * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply the whole transform
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
    break;
    case CloudChannelType::DIRECTION:
      for (; x_out != x_out.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
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

void transformChannel(
    sensor_msgs::msg::PointCloud2& cloud, const geometry_msgs::msg::Transform& transform,
    const std::string& channel_prefix, const CloudChannelType type) {
  const auto tf = tf2::transformToEigen(transform).cast<float>();
  transformChannel(cloud, cloud, tf, channel_prefix, type);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    const geometry_msgs::msg::TransformStamped& tf) {
  return transformWithChannels(in, out, tf, DEFAULT_CHANNELS);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    const geometry_msgs::msg::TransformStamped& tf, const std::unordered_map<std::string, CloudChannelType>& channels) {
  std::unordered_set<std::string> channels_present;
  for (const auto& field : in.fields) {
    for (const auto& [channel, channelType] : channels) {
      if (channelType != CloudChannelType::SCALAR && fieldNameMatchesChannel(field.name, channel, channelType)) {
        channels_present.insert(channel);
      }
    }
  }

  out = in;
  out.header = tf.header;

  const auto transform = tf2::transformToEigen(tf).cast<float>();

  for (const auto& channel : channels_present) {
    transformChannel(in, out, transform, channel, channels.at(channel));
  }

  return out;
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    const tf2::BufferCoreInterface& tf_buffer, const std::string& target_frame) {
  return transformWithChannels(in, out, tf_buffer, target_frame, DEFAULT_CHANNELS);
}

sensor_msgs::msg::PointCloud2& transformWithChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    const tf2::BufferCoreInterface& tf_buffer, const std::string& target_frame,
    const std::unordered_map<std::string, CloudChannelType>& channels) {
  const auto stamp = cras::convertTime<tf2::TimePoint>(in.header.stamp);
  const auto tf = tf_buffer.lookupTransform(target_frame, in.header.frame_id, stamp);
  return transformWithChannels(in, out, tf, channels);
}

sensor_msgs::msg::PointCloud2& transformOnlyChannels(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    const geometry_msgs::msg::TransformStamped& tf, const std::unordered_map<std::string, CloudChannelType>& channels) {
  std::unordered_set<std::string> channels_present;
  out.point_step = 0;
  for (const auto& field : in.fields) {
    for (const auto& [channel, channelType] : channels) {
      if (fieldNameMatchesChannel(field.name, channel, channelType)) {
        channels_present.insert(channel);
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

  for (const auto& channel : channels_present) {
    const auto channelType = channels.at(channel);
    if (channelType != CloudChannelType::SCALAR) {
      transformChannel(in, out, transform, channel, channelType);
    } else {
      copyChannelData(in, out, channel);
    }
  }

  return out;
}

sensor_msgs::msg::PointCloud2& transformOnlyXYZ(
    const sensor_msgs::msg::PointCloud2& in, sensor_msgs::msg::PointCloud2& out,
    const geometry_msgs::msg::TransformStamped& tf) {
  return transformOnlyChannels(in, out, tf, XYZ_CHANNELS);
}

}  // namespace cras
