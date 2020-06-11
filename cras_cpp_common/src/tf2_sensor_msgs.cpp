#include <cras_cpp_common/tf2_sensor_msgs.h>

#include <cras_cpp_common/cloud.hpp>
#include <cras_cpp_common/string_utils.hpp>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>  // needs to be implementation-private as we want -march=native optimizations

#include <unordered_set>

namespace cras {

const static std::unordered_map<std::string, CloudChannelType> DEFAULT_CHANNELS({
  {"vp_", CloudChannelType::POINT},
  {"normal_", CloudChannelType::DIRECTION}
});

void transformChannel(sensor_msgs::PointCloud2& cloud, const Eigen::Isometry3f& tt,
    const std::string& channelPrefix, const CloudChannelType type)
{
  CloudIter x_out(cloud, channelPrefix + "x");
  CloudIter y_out(cloud, channelPrefix + "y");
  CloudIter z_out(cloud, channelPrefix + "z");

  Eigen::Vector3f point;
  Eigen::Isometry3f t = tt;  // allow alignment

  // the switch has to be outside the for loop for performance reasons
  switch (type) {
    case CloudChannelType::POINT:
      for (; x_out != x_out.end(); ++x_out, ++y_out, ++z_out) {
        point = t * Eigen::Vector3f(*x_out, *y_out, *z_out);  // apply the whole transform
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      break;
    case CloudChannelType::DIRECTION:
      for (; x_out != x_out.end(); ++x_out, ++y_out, ++z_out) {
        point = t.linear() * Eigen::Vector3f(*x_out, *y_out, *z_out);  // apply only rotation
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      break;
  }
}

void transformChannel(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Transform& tf,
                      const std::string& channelPrefix, CloudChannelType type)
{
  const auto t = tf2::transformToEigen(tf).cast<float>();
  transformChannel(cloud, t, channelPrefix, type);
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf)
{
    return transformWithChannels(in, out, tf, DEFAULT_CHANNELS);
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  for (const auto& field: in.fields) {
    for (const auto& channelAndType : channels)
    {
      const std::string& channel = channelAndType.first;
      if (startsWith(field.name, channel))
        channelsPresent.insert(channel);
    }
  }

  out = in;
  out.header = tf.header;

  const auto t = tf2::transformToEigen(tf).cast<float>();

  transformChannel(out, t, "", CloudChannelType::POINT);
  for (const auto& channel : channelsPresent)
    transformChannel(out, t, channel, channels.at(channel));

  return out;
}

}