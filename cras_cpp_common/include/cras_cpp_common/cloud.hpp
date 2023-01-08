#pragma once

/**
 * \file
 * \brief Utilities for comfortable working with PointCloud2 messages.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "cloud/impl/cloud.hpp"

namespace cras
{

//! \brief Shorthand for sensor_msgs::PointCloud2
typedef ::sensor_msgs::PointCloud2 Cloud;

//! \brief Cloud float field iterator.
typedef ::sensor_msgs::PointCloud2Iterator<float> CloudIter;

//! \brief Cloud int field iterator.
typedef ::sensor_msgs::PointCloud2Iterator<int> CloudIndexIter;

//! \brief Const cloud float field iterator.
typedef ::sensor_msgs::PointCloud2ConstIterator<float> CloudConstIter;

//! \brief Const cloud int field iterator.
typedef ::sensor_msgs::PointCloud2ConstIterator<int> CloudIndexConstIter;

//! \brief Shorthand for sensor_msgs::PointCloud2Modifier.
typedef ::sensor_msgs::PointCloud2Modifier CloudModifier;

/**
 * \brief GenericCloudIter and GenericCloudConstIter are iterators of fields of types unknown at compile time.
 *
 * \details
 * The iterators allow you to dereference them into an unsigned char, which doesn't however need to be the actual data,
 * as they may span multiple bytes.
 *
 * It adds function rawData() which returns a pointer to the current position in the uchar data stream. You can use
 * reinterpret_cast to transform the data into some desired type and get or set the value. Any kind of data safety is on
 * you.
 *
 * Another provided function is dataAs<Type>() which returns the current iterator position as a pointer to data of the
 * requested type. This function does a basic check that the requested data type has the same size as the type of the
 * iterated field.
 *
 * The non-const iterator also provides method copyData() which can copy the field data from another generic iterator.
 * This can be used to copy fields of types which are not known at compile time.
 */
typedef ::cras::impl::GenericCloudIterator<> GenericCloudIter;

//! \copydoc ::cras::GenericCloudIter
typedef ::cras::impl::GenericCloudConstIterator<> GenericCloudConstIter;

/**
 * \brief Return the number of points the given pointcloud contains.
 * \param[in] cloud The cloud to examine.
 * \return The number of points.
 */
inline size_t numPoints(const ::cras::Cloud& cloud)
{
  return static_cast<size_t>(cloud.height) * static_cast<size_t>(cloud.width);
}

/**
 * \brief Return true if the cloud contains a field with the given name.
 * \param[in] cloud The cloud to search.
 * \param[in] fieldName Name of the field.
 * \return Whether the field is there or not.
 */
bool hasField(const ::cras::Cloud& cloud, const ::std::string& fieldName);

/**
 * \brief Return the sensor_msgs::PointField with the given name.
 * \param[in] cloud Cloud to extract the field from.
 * \param[in] fieldName Name of the field.
 * \return Reference to the field.
 * \throws std::runtime_error if the field doesn't exist.
 */
::sensor_msgs::PointField& getField(::cras::Cloud& cloud, const ::std::string& fieldName);

/**
 * \brief Return the sensor_msgs::PointField with the given name.
 * \param[in] cloud Cloud to extract the field from.
 * \param[in] fieldName Name of the field.
 * \return Reference to the field.
 * \throws std::runtime_error if the field doesn't exist.
 */
const ::sensor_msgs::PointField& getField(const ::cras::Cloud& cloud, const ::std::string& fieldName);

/**
 * \brief Return the size (in bytes) of a sensor_msgs::PointField datatype.
 * \param[in] datatype The datatype (one of sensor_msgs::PointField::(U?INT(8|16|32)|FLOAT(32|64)) constants).
 * \return Size of the datatype in bytes.
 * \throws std::runtime_error if wrong datatype is passed.
 */
size_t sizeOfPointField(int datatype);

/**
 * \brief Return the size (in bytes) of the data represented by the sensor_msgs::PointField.
 * \param[in] field The pointfield specification.
 * \return Size of the data.
 * \throws std::runtime_error if wrong datatype is passed.
 */
size_t sizeOfPointField(const ::sensor_msgs::PointField& field);

/**
 * \brief Copy data belonging to the given field from `in` cloud to `out` cloud.
 * \param[in] in The input cloud.
 * \param[out] out The ouptut cloud. It has to be resized to contain at least that many points as the input cloud.
 *                 It also has to have the given field present already.
 * \param[in] fieldName Name of the field whose data should be copied.
 * \throws std::runtime_error If the output cloud is smaller (in nr. of points) than the input cloud.
 * \throws std::runtime_error If the given field doesn't exist in either of the clouds.
 */
void copyChannelData(const ::cras::Cloud& in, ::cras::Cloud& out, const ::std::string& fieldName);

/**
 * \brief Create a pointcloud that contains a subset of points of `IN` defined by
 * the filter `FILTER`. The result is saved into `OUT`. `FILTER` should be a boolean expression
 * which can use the following: `i`: index of the point, `x_it, y_it, z_it` iterators to XYZ coordinates.
 * Points for which FILTER is true are part of the final pointcloud.
 */
#define CREATE_FILTERED_CLOUD(IN, OUT, KEEP_ORGANIZED, FILTER) {\
  const auto inputIsOrganized = (IN).height > 1; \
  const auto outIsOrganized = (KEEP_ORGANIZED) && inputIsOrganized; \
  \
  (OUT).header = (IN).header; \
  (OUT).fields = (IN).fields; \
  (OUT).point_step = (IN).point_step; \
  (OUT).height = outIsOrganized ? (IN).height : 1; \
  (OUT).width = outIsOrganized ? (IN).width : 0; \
  \
  (OUT).data.resize(0); \
  (OUT).data.reserve((IN).data.size()); \
  \
  ::cras::CloudConstIter x_it((IN), "x"); \
  ::cras::CloudConstIter y_it((IN), "y"); \
  ::cras::CloudConstIter z_it((IN), "z"); \
  \
  const auto numPoints = ::cras::numPoints(IN); \
  \
  if (!outIsOrganized) { \
    for (size_t i = 0; i < numPoints; ++i, ++x_it, ++y_it, ++z_it) { \
      if (FILTER) { \
        size_t from = (i / (IN).width) * (IN).row_step + (i % (IN).width) * (IN).point_step; \
        size_t to = from + (IN).point_step; \
        (OUT).data.insert((OUT).data.end(), (IN).data.begin() + from, \
                        (IN).data.begin() + to); \
        (OUT).width++; \
      } \
    } \
    (OUT).is_dense = true; \
  } else { \
    (OUT).data.insert((OUT).data.end(), (IN).data.begin(), (IN).data.end()); \
    \
    ::cras::CloudIter x_out_it((OUT), "x"); \
    ::cras::CloudIter y_out_it((OUT), "y"); \
    ::cras::CloudIter z_out_it((OUT), "z"); \
    const auto invalidValue = std::numeric_limits<float>::quiet_NaN(); \
    \
    for (size_t i = 0; i < numPoints; ++i, ++x_it, ++y_it, ++z_it, ++x_out_it, ++y_out_it, ++z_out_it) { \
      if (!(FILTER)) { \
        *x_out_it = *y_out_it = *z_out_it = invalidValue; \
        (OUT).is_dense = false; \
      } \
    } \
  } \
  \
  (OUT).row_step = (OUT).width * (OUT).point_step;\
}
}
