#pragma once

/**
 * \file
 * \brief Utilities for comfortable working with PointCloud2 messages.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace cras
{

namespace impl
{

/**
 * \brief Base of a generic cloud iterator which can return the data in the raw type.
 * \tparam T Type of the value on which the child class will be templated.
 * \tparam TT Type of the value to be retrieved (same as T except for constness).
 * \tparam U Type of the raw data in PointCloud2 (only uchar and const uchar are supported).
 * \tparam C Type of the pointcloud to intialize from (const or not).
 * \tparam V The derived class (yop, curiously recurring template pattern).
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
class GenericCloudIteratorBase : public sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V>
{
public:
  
  /**
   * \param[in] cloud_msg The PointCloud2 to iterate upon.
   * \param[in] field_name The field to iterate upon.
   */
  GenericCloudIteratorBase(C& cloudMsg, const std::string& fieldName);

  /**
   * \brief Get the byte size of the field which this iterator iterates.
   * \return Size in bytes.
   */
  inline size_t getFieldSize() { return this->fieldSize; }

  /**
   * \brief Return a pointer to the raw data of this field. Only `getFieldSize()` bytes after this pointer are valid.
   * \return The pointer.
   */
  U* getData() const;

protected:
  //! \brief The byte size of the field which this iterator iterates.
  size_t fieldSize = 0;
};

/**
 * \brief Generic const cloud iterator which can return the data in the raw type.
 * \tparam T Type of the value on which the child class will be templated.
 */
template<typename T>
class GenericCloudConstIterator
  : public GenericCloudIteratorBase<unsigned char, const unsigned char, const unsigned char,
      const sensor_msgs::PointCloud2, GenericCloudConstIterator>
{
public:
  /**
   * \param[in] cloud_msg The PointCloud2 to iterate upon.
   * \param[in] field_name The field to iterate upon.
   */
  GenericCloudConstIterator(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
    : GenericCloudIteratorBase<T, const T, const unsigned char, const sensor_msgs::PointCloud2,
        GenericCloudConstIterator>::GenericCloudIteratorBase(cloud_msg, field_name)
  {
  }
};

/**
 * \brief Generic non-const cloud iterator which can return the data in the raw type.
 * \tparam T Type of the value on which the child class will be templated.
 */
template<typename T>
class GenericCloudIterator
  : public GenericCloudIteratorBase<unsigned char, unsigned char, unsigned char,
      sensor_msgs::PointCloud2, GenericCloudIterator>
{
public:
  /**
   * \param[in] cloud_msg The PointCloud2 to iterate upon.
   * \param[in] field_name The field to iterate upon.
   */
  GenericCloudIterator(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
    : GenericCloudIteratorBase<T, T, unsigned char, sensor_msgs::PointCloud2,
        GenericCloudIterator>::GenericCloudIteratorBase(cloud_msg, field_name)
  {
  }

  /**
   * \brief Copy all values of this field from another iterator.
   * \param otherIter The other iterator.
   */
  void copyData(const GenericCloudConstIterator<T>& otherIter) const;

  /**
   * \brief Copy all values of this field from another iterator.
   * \param otherIter The other iterator.
   */
  void copyData(const GenericCloudIterator<T>& otherIter) const;
};

}
}
