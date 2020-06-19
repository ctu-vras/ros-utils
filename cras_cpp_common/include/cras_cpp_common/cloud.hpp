#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cras_cpp_common/cloud-impl.hpp>

namespace cras
{

typedef sensor_msgs::PointCloud2 Cloud;
typedef sensor_msgs::PointCloud2Iterator<float> CloudIter;
typedef sensor_msgs::PointCloud2Iterator<int> CloudIndexIter;
typedef sensor_msgs::PointCloud2ConstIterator<float> CloudConstIter;
typedef sensor_msgs::PointCloud2ConstIterator<int> CloudIndexConstIter;
typedef sensor_msgs::PointCloud2Modifier CloudModifier;

// from cloud-impl.hpp
/**
 * GenericCloudIter and GenericCloudConstIter are iterators of fields of types unknown at compile time.
 *
 * The iterators allow you to dereference them into an unsigned char, which doesn't however need to be the actual data,
 * as they may span multiple bytes.
 *
 * It adds function getData() which returns a pointer to the current position in the uchar data stream. You can use
 * reinterpret_cast to transform the data into some desired type and get or set the value. Any kind of data safety is on
 * you.
 *
 * The non-const iterator also provides method copyData() which can copy the field data from another generic iterator.
 * This can be used to copy fields of types which are not known at compile time.
 */
typedef impl::GenericCloudIterator<unsigned char> GenericCloudIter;
typedef impl::GenericCloudConstIterator<unsigned char> GenericCloudConstIter;

/**
 * Return the number of points the given pointcloud contains.
 * @param cloud The cloud to examine.
 * @return The number of points.
 */
inline size_t numPoints(const Cloud& cloud) {
  return static_cast<size_t>(cloud.height) * static_cast<size_t>(cloud.width);
}

/**
 * Return true if the cloud contains a field with the given name.
 * @param cloud The cloud to search.
 * @param fieldName Name of the field.
 * @return Whether the field is there or not.
 */
bool hasField(const Cloud& cloud, const std::string& fieldName);

/**
 * Return the sensor_msgs::PointField with the given name.
 * @param cloud Cloud to extract the field from.
 * @param fieldName Name of the field.
 * @return Reference to the field.
 * @throws std::runtime_error if the field doesn't exist.
 */
sensor_msgs::PointField& getField(Cloud& cloud, const std::string& fieldName);

/**
 * Return the sensor_msgs::PointField with the given name.
 * @param cloud Cloud to extract the field from.
 * @param fieldName Name of the field.
 * @return Reference to the field.
 * @throws std::runtime_error if the field doesn't exist.
 */
const sensor_msgs::PointField& getField(const Cloud& cloud, const std::string& fieldName);

/**
 * Return the size (in bytes) of a sensor_msgs::PointField datatype.
 * @param datatype The datatype (one of sensor_msgs::PointField::(U?INT(8|16|32)|FLOAT(32|64)) constants).
 * @return Size of the datatype in bytes.
 * @throws std::runtime_error if wrong datatype is passed.
 */
size_t sizeOfPointField(int datatype);

/**
 * Return the size (in bytes) of the data represented by the sensor_msgs::PointField.
 * @param field The pointfield specification.
 * @return Size of the data.
 * @throws std::runtime_error if wrong datatype is passed.
 */
size_t sizeOfPointField(const sensor_msgs::PointField& field);

/**
 * Copy data belonging to the given field from `in` cloud to `out` cloud.
 * @param in The input cloud.
 * @param out The ouptut cloud. It has to be resized to contain at least that many points as the input cloud. It also
 *            has to have the given field present already.
 * @param fieldName Name of the field whose data should be copied.
 * @throws std::runtime_error If the output cloud is smaller (in nr. of points) than the input cloud.
 * @throws std::runtime_error If the given field doesn't exist in either of the clouds.
 */
void copyChannelData(const Cloud& in, Cloud& out, const std::string& fieldName);
}
