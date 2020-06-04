#pragma once

#include <string>
#include <unordered_map>

#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace cras
{

enum class CloudChannelType { POINT, DIRECTION };

void transformChannel(sensor_msgs::PointCloud2& cloud, const Eigen::Isometry3f& t,
                      const std::string& channelPrefix, CloudChannelType type);

void transformChannel(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Transform& t,
                      const std::string& channelPrefix, CloudChannelType type);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels);

}
