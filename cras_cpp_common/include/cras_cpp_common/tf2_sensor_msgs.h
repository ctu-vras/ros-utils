#pragma once

#include <string>
#include <unordered_map>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace cras
{

enum class CloudChannelType { POINT, DIRECTION };

void transformChannel(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Transform& t,
                      const std::string& channelPrefix, CloudChannelType type);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels);

}
