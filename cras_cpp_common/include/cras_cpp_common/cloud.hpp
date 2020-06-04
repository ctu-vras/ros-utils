#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace cras
{

typedef sensor_msgs::PointCloud2 Cloud;
typedef sensor_msgs::PointCloud2Iterator<float> CloudIter;
typedef sensor_msgs::PointCloud2Iterator<int> CloudIndexIter;
typedef sensor_msgs::PointCloud2ConstIterator<float> CloudConstIter;
typedef sensor_msgs::PointCloud2ConstIterator<int> CloudIndexConstIter;
typedef sensor_msgs::PointCloud2Modifier CloudModifier;

inline size_t numPoints(const Cloud& cloud) {
  return static_cast<size_t>(cloud.height) * static_cast<size_t>(cloud.width);
}

}
