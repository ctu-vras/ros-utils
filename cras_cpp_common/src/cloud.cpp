/**
 * \file
 * \brief Utilities for comfortable working with PointCloud2 messages.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <sstream>
#define private protected
#include <sensor_msgs/point_cloud2_iterator.h>
#undef private

#include <cras_cpp_common/cloud.hpp>

namespace cras
{

bool hasField(const ::cras::Cloud &cloud, const ::std::string &fieldName)
{
  for (const auto &field : cloud.fields)
  {
    if (field.name == fieldName)
      return true;
  }
  return false;
}

::sensor_msgs::PointField& getField(::cras::Cloud& cloud, const ::std::string& fieldName)
{
  for (auto &field : cloud.fields) {
    if (field.name == fieldName)
      return field;
  }
  throw ::std::runtime_error(::std::string("Field ") + fieldName + " does not exist.");
}

const ::sensor_msgs::PointField& getField(const ::cras::Cloud& cloud, const ::std::string& fieldName)
{
  for (const auto &field : cloud.fields) {
    if (field.name == fieldName)
      return field;
  }
  throw ::std::runtime_error(::std::string("Field ") + fieldName + " does not exist.");
}

size_t sizeOfPointField(const ::sensor_msgs::PointField& field)
{
  return ::cras::sizeOfPointField(field.datatype);
}

size_t sizeOfPointField(int datatype)
{
  if ((datatype == ::sensor_msgs::PointField::INT8) || (datatype == ::sensor_msgs::PointField::UINT8))
    return 1u;
  else if ((datatype == ::sensor_msgs::PointField::INT16) || (datatype == ::sensor_msgs::PointField::UINT16))
    return 2u;
  else if ((datatype == ::sensor_msgs::PointField::INT32) || (datatype == ::sensor_msgs::PointField::UINT32) ||
           (datatype == ::sensor_msgs::PointField::FLOAT32))
    return 4u;
  else if (datatype == ::sensor_msgs::PointField::FLOAT64)
    return 8u;
  else
    throw ::std::runtime_error(::std::string("PointField of type ") + ::std::to_string(datatype) + " does not exist");
}

void copyChannelData(const ::cras::Cloud& in, ::cras::Cloud& out, const ::std::string& fieldName)
{
  if (numPoints(out) < numPoints(in))
    throw ::std::runtime_error("Output cloud needs to be resized to fit the number of points of the input cloud.");

  ::cras::GenericCloudConstIter dataIn(in, fieldName);
  ::cras::GenericCloudIter dataOut(out, fieldName);
  for (; dataIn != dataIn.end(); ++dataIn, ++dataOut)
    dataOut.copyData(dataIn);
}

namespace impl
{

template<typename T, typename TT, typename U, typename C, template<typename> class V>
GenericCloudIteratorBase<T, TT, U, C, V>::GenericCloudIteratorBase(C& cloudMsg, const ::std::string& fieldName)
  : ::sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V>(cloudMsg, fieldName)
{
  this->fieldSize = ::cras::sizeOfPointField(::cras::getField(cloudMsg, fieldName));
}

template<typename T, typename TT, typename U, typename C, template<typename> class V>
U* GenericCloudIteratorBase<T, TT, U, C, V>::getData() const
{
  return this->data_;
}

template<typename T>
void GenericCloudIterator<T>::copyData(const ::cras::impl::GenericCloudConstIterator<T>& otherIter) const
{
  ::memcpy(this->getData(), otherIter.getData(), this->fieldSize);
}

template<typename T>
void GenericCloudIterator<T>::copyData(const ::cras::impl::GenericCloudIterator<T>& otherIter) const
{
  ::memcpy(this->getData(), otherIter.getData(), this->fieldSize);
}

// explicitly instantiate
template class ::cras::impl::GenericCloudIteratorBase<unsigned char, unsigned char, unsigned char,
  sensor_msgs::PointCloud2, GenericCloudIterator>;
template class ::cras::impl::GenericCloudIteratorBase<unsigned char, const unsigned char, const unsigned char,
  const sensor_msgs::PointCloud2, GenericCloudConstIterator>;
template class ::cras::impl::GenericCloudIterator<unsigned char>;
template class ::cras::impl::GenericCloudConstIterator<unsigned char>;
}

}