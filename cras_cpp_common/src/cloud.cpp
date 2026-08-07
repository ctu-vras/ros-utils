// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Utilities for comfortable working with PointCloud2 messages.
 * \author Martin Pecka
 */

// HACK: we need to access PointCloud2IteratorBase::data_char_ which is private
#include <sstream>
#define private protected
#include <sensor_msgs/point_cloud2_iterator.hpp>
#undef private

#include <algorithm>
#include <cstring>
#include <string>

#include <cras_cpp_common/cloud.hpp>

namespace cras {

bool hasField(const ::cras::Cloud& cloud, const std::string& field_name) {
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
      [&field_name](const sensor_msgs::msg::PointField& f) {return f.name == field_name;});
}

sensor_msgs::msg::PointField& getField(::cras::Cloud& cloud, const std::string& field_name) {
  for (auto& field : cloud.fields) {
    if (field.name == field_name) {
      return field;
    }
  }
  throw std::runtime_error(std::string("Field ") + field_name + " does not exist.");
}

const sensor_msgs::msg::PointField& getField(const ::cras::Cloud& cloud, const std::string& field_name) {
  for (const auto& field : cloud.fields) {
    if (field.name == field_name) {
      return field;
    }
  }
  throw std::runtime_error(std::string("Field ") + field_name + " does not exist.");
}

size_t sizeOfPointField(const ::sensor_msgs::msg::PointField& field) {
  return sizeOfPointField(field.datatype);
}

size_t sizeOfPointField(const int datatype) {
  using sensor_msgs::msg::PointField;
  if (datatype == PointField::INT8 || datatype == PointField::UINT8) {
    return 1u;
  } else if (datatype == PointField::INT16 || datatype == PointField::UINT16) {
    return 2u;
  } else if (datatype == PointField::INT32 || datatype == PointField::UINT32 || datatype == PointField::FLOAT32) {
    return 4u;
  } else if (datatype == PointField::FLOAT64) {
    return 8u;
  } else {
    throw std::runtime_error(std::string("PointField of type ") + std::to_string(datatype) + " does not exist");
  }
}

void copyChannelData(const ::cras::Cloud& in, ::cras::Cloud& out, const std::string& field_name) {
  if (numPoints(out) < numPoints(in)) {
    throw std::runtime_error("Output cloud needs to be resized to fit the number of points of the input cloud.");
  }

  GenericCloudConstIter dataIn(in, field_name);
  GenericCloudIter dataOut(out, field_name);
  for (; dataIn != dataIn.end(); ++dataIn, ++dataOut) {
    dataOut.copyData(dataIn);
  }
}

namespace impl {

template<typename T, typename TT, typename U, typename C, template<typename> class V>
GenericCloudIteratorBase<T, TT, U, C, V>::GenericCloudIteratorBase(C& cloud_msg, const std::string& field_name)
    : sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V>(cloud_msg, field_name) {
  field_size_ = sizeOfPointField(getField(cloud_msg, field_name));
}

template<typename T, typename TT, typename U, typename C, template<typename> class V>
U* GenericCloudIteratorBase<T, TT, U, C, V>::rawData() const {
  return this->data_char_;
}

template<typename T>
void GenericCloudIterator<T>::copyData(const GenericCloudConstIterator<T>& other_iter) const {
  std::memcpy(this->rawData(), other_iter.rawData(), this->field_size_);
}

template<typename T>
void GenericCloudIterator<T>::copyData(const GenericCloudIterator<T>& other_iter) const {
  std::memcpy(this->rawData(), other_iter.rawData(), this->field_size_);
}

// explicitly instantiate
template class GenericCloudIteratorBase<unsigned char, unsigned char, unsigned char,
    sensor_msgs::msg::PointCloud2, GenericCloudIterator>;
template class GenericCloudIteratorBase<unsigned char, const unsigned char, const unsigned char,
    const sensor_msgs::msg::PointCloud2, GenericCloudConstIterator>;

template class GenericCloudIterator<>;
template class GenericCloudConstIterator<>;
}  // namespace impl

}  // namespace cras
