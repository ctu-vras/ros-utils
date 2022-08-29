#pragma once

/**
 * \file
 * \brief Transformation tools for sensor_msgs messages.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <unordered_map>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

namespace cras
{

/**
 * \brief Type of a pointcloud channel.
 */
enum class CloudChannelType
{
  //! \brief A 3D point (if transformed, both translation and rotation is applied).
  POINT,

  //! \brief A 3D vector/direction (if transformed, only rotation is applied).
  DIRECTION,

  //! \brief A scalar value (not affected by transforms).
  SCALAR
};

/**
 * \brief Register the given pointcloud channel prefix with the given type. This registration will be used by
 * transformWithChannels() when called without an explicit channel list.
 * \param[in] channelPrefix Prefix of the channel. E.g. `normal_` for registering type of normal_x,normal_y,normal_z.
 * \param[in] type Type of the channel.
 */
void registerCloudChannelType(const ::std::string& channelPrefix, ::cras::CloudChannelType type);

/**
 * \brief Unregister a cloud channel type registered earlier with registerCloudChannelType().
 * \param[in] channelPrefix Prefix of the channel.
 */
void unregisterCloudChannelType(const ::std::string& channelPrefix);

/**
 * \brief Transform the given channel in the given cloud using the given transform.
 * \param[in,out] cloud The cloud.
 * \param[in] transform The transform to apply.
 * \param[in] channelPrefix Prefix of the channel.
 * \param[in] type Type of the channel.
 */
void transformChannel(::sensor_msgs::PointCloud2& cloud, const ::geometry_msgs::Transform& transform,
  const ::std::string& channelPrefix, ::cras::CloudChannelType type);

/**
 * \brief Copy `in` cloud to `out` and transform channels using the given transform. The list of channels to be
 *        transformed consists of the XYZ channel, vp_, normal_ and all channels registered by
 *        registerCloudChannelType().
 * \param[in] in The input cloud.
 * \param[out] out The output cloud (can be the same as input).
 * \param[in] tf The transform to apply.
 * \return `out`.
 */
::sensor_msgs::PointCloud2& transformWithChannels(const ::sensor_msgs::PointCloud2& in, ::sensor_msgs::PointCloud2& out,
  const ::geometry_msgs::TransformStamped& tf);

/**
 * \brief Copy `in` cloud to `out` and transform channels using the given transform. Only the channels passed in
 *        `channels` will be transformed, according to their type.
 * \param[in] in The input cloud.
 * \param[out] out The output cloud (can be the same as input).
 * \param[in] tf The transform to apply.
 * \param[in] channels A map of `channel prefix`-`channel type` of channels that should be transformed.
 * \return `out`.
 */
::sensor_msgs::PointCloud2& transformWithChannels(const ::sensor_msgs::PointCloud2& in, ::sensor_msgs::PointCloud2& out,
  const ::geometry_msgs::TransformStamped& tf,
  const ::std::unordered_map<::std::string, ::cras::CloudChannelType>& channels);

/**
 * \brief Copy `in` cloud to `out` and transform channels using the given transform. The list of channels to be
 *        transformed consists of the XYZ channel, vp_, normal_ and all channels registered by
 *        registerCloudChannelType().
 * \param[in] in The input cloud.
 * \param[out] out The output cloud (can be the same as input).
 * \param[in] tfBuffer The TF buffer.
 * \param[in] targetFrame The frame to transform to.
 * \return `out`.
 * \throws tf2::TransformException No exceptions thrown from lookupTransform() will be catched.
 */
::sensor_msgs::PointCloud2& transformWithChannels(const ::sensor_msgs::PointCloud2& in, ::sensor_msgs::PointCloud2& out,
  const ::tf2_ros::Buffer& tfBuffer, const ::std::string& targetFrame);

/**
 * \brief Copy `in` cloud to `out` and transform channels using the given transform. Only the channels passed in
 *        `channels` will be transformed, according to their type.
 * \param[in] in The input cloud.
 * \param[out] out The output cloud (can be the same as input).
 * \param[in] tfBuffer The TF buffer.
 * \param[in] targetFrame The frame to transform to.
 * \param[in] channels A map of `channel prefix`-`channel type` of channels that should be transformed.
 * \return `out`.
 * \throws tf2::TransformException No exceptions thrown from lookupTransform() will be catched.
 */
::sensor_msgs::PointCloud2& transformWithChannels(const ::sensor_msgs::PointCloud2& in, ::sensor_msgs::PointCloud2& out,
  const ::tf2_ros::Buffer& tfBuffer, const ::std::string& targetFrame,
  const ::std::unordered_map<::std::string, ::cras::CloudChannelType>& channels);

/**
 * \brief Copy the selected channels from `in` cloud to `out` and transform them using the given transform.
 * \param[in] in The input cloud.
 * \param[out] out The output cloud (can not be the same as input).
 * \param[in] tf The transform to apply.
 * \param[in] channels A map of `channel prefix`-`channel type` of channels that should be transformed. No other
 *                     channels will be present in the output cloud.
 * \return `out`.
 */
::sensor_msgs::PointCloud2& transformOnlyChannels(const ::sensor_msgs::PointCloud2& in, ::sensor_msgs::PointCloud2& out,
  const ::geometry_msgs::TransformStamped& tf,
  const ::std::unordered_map<::std::string, ::cras::CloudChannelType>& channels);

/**
 * \brief Copy only the XYZ channel from `in` cloud to `out` and transform it using the given transform.
 * \param[in] in The input cloud.
 * \param[out] out The output cloud (can not be the same as input).
 * \param[in] tf The transform to apply.
 * \return `out`.
 */
::sensor_msgs::PointCloud2& transformOnlyXYZ(const ::sensor_msgs::PointCloud2& in, ::sensor_msgs::PointCloud2& out,
  const ::geometry_msgs::TransformStamped& tf);

}
