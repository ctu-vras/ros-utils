#pragma once

/**
 * \file
 * \brief Specializations of getParam() for Eigen vectors and matrices.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cras
{

namespace
{
  template <typename Scalar>
  void checkSize(const ::std::vector<Scalar>& v, const size_t i, const ::std::string& type)
  {
    if (v.size() != i)
      throw ::std::runtime_error("Cannot construct " + type + " from " + ::std::to_string(v.size()) + " elements " +
        "(expected " + ::std::to_string(i) + ").");
  }
}

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector3d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v){ checkSize(v, 3, "vector"); return ::Eigen::Vector3d(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector3f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v){ checkSize(v, 3, "vector"); return ::Eigen::Vector3f(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector4d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v){ checkSize(v, 4, "vector"); return ::Eigen::Vector4d(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector4f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v){ checkSize(v, 4, "vector"); return ::Eigen::Vector4f(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix3d, ::std::vector<double>, "",
  ([](const ::std::vector<double>& v){ checkSize(v, 9, "3x3 matrix");
    return ::Eigen::Matrix<double, 3, 3, ::Eigen::RowMajor>(v.data()); }))

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix3f, ::std::vector<float>, "",
  ([](const ::std::vector<float>& v){ checkSize(v, 9, "3x3 matrix");
    return ::Eigen::Matrix<float, 3, 3, ::Eigen::RowMajor>(v.data()); }))

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix4d, ::std::vector<double>, "",
  ([](const ::std::vector<double>& v){ checkSize(v, 16, "4x4 matrix");
    return ::Eigen::Matrix<double, 4, 4, ::Eigen::RowMajor>(v.data()); }))

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix4f, ::std::vector<float>, "",
  ([](const ::std::vector<float>& v){ checkSize(v, 16, "4x4 matrix");
    return ::Eigen::Matrix<float, 4, 4, ::Eigen::RowMajor>(v.data()); }))

DEFINE_CONVERTING_GET_PARAM(::Eigen::Quaterniond, ::std::vector<double>, "",
  [](const ::std::vector<double>& v)
  {
    switch (v.size())
    {
      case 3:
        return ::Eigen::Quaterniond(
          ::Eigen::AngleAxisd(v[0], ::Eigen::Vector3d::UnitX()) *
          ::Eigen::AngleAxisd(v[1], ::Eigen::Vector3d::UnitY()) *
          ::Eigen::AngleAxisd(v[2], ::Eigen::Vector3d::UnitZ()));
      case 4:
        return ::Eigen::Quaterniond(v[3], v[0], v[1], v[2]);  // w first!
      default:
        throw ::std::runtime_error("Cannot construct quaternion from " + ::std::to_string(v.size()) + " elements.");
    }
  })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Quaternionf, ::std::vector<float>, "",
  [](const ::std::vector<float>& v)
  {
    switch (v.size())
    {
      case 3:
        return ::Eigen::Quaternionf(
          ::Eigen::AngleAxisf(v[0], ::Eigen::Vector3f::UnitX()) *
          ::Eigen::AngleAxisf(v[1], ::Eigen::Vector3f::UnitY()) *
          ::Eigen::AngleAxisf(v[2], ::Eigen::Vector3f::UnitZ()));
      case 4:
        return ::Eigen::Quaternionf(v[3], v[0], v[1], v[2]);  // w first!
      default:
        throw ::std::runtime_error("Cannot construct quaternion from " + ::std::to_string(v.size()) + " elements.");
    }
  })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Isometry3d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v)
  {
    auto result = ::Eigen::Isometry3d::Identity();
    switch (v.size())
    {
      case 6:
        result.translationExt() = ::Eigen::Vector3d(v[0], v[1], v[2]);
        result.linearExt() = ::Eigen::Quaterniond(
          ::Eigen::AngleAxisd(v[3], ::Eigen::Vector3d::UnitX()) *
          ::Eigen::AngleAxisd(v[4], ::Eigen::Vector3d::UnitY()) *
          ::Eigen::AngleAxisd(v[5], ::Eigen::Vector3d::UnitZ())).toRotationMatrix();
        return result;
      case 7:
        result.translationExt() = ::Eigen::Vector3d(v[0], v[1], v[2]);
        result.linearExt() = ::Eigen::Quaterniond(v[6], v[3], v[4], v[5]).toRotationMatrix();  // w first!
        return result;
      case 16:
        result = (::Eigen::Matrix<double, 4, 4, ::Eigen::RowMajor>(v.data()));
        return result;
      default:
        throw ::std::runtime_error("Cannot construct Eigen isometry from " + ::std::to_string(v.size()) + " elements.");
    }
  })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Isometry3f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v)
  {
    auto result = ::Eigen::Isometry3f::Identity();
    switch (v.size())
    {
      case 6:
        result.translationExt() = ::Eigen::Vector3f(v[0], v[1], v[2]);
        result.linearExt() = ::Eigen::Quaternionf(
          ::Eigen::AngleAxisf(v[3], ::Eigen::Vector3f::UnitX()) *
          ::Eigen::AngleAxisf(v[4], ::Eigen::Vector3f::UnitY()) *
          ::Eigen::AngleAxisf(v[5], ::Eigen::Vector3f::UnitZ())).toRotationMatrix();
        return result;
      case 7:
        result.translationExt() = ::Eigen::Vector3f(v[0], v[1], v[2]);
        result.linearExt() = ::Eigen::Quaternionf(v[6], v[3], v[4], v[5]).toRotationMatrix();  // w first!
        return result;
      case 16:
        result = (::Eigen::Matrix<float, 4, 4, ::Eigen::RowMajor>(v.data()));
        return result;
      default:
        throw ::std::runtime_error("Cannot construct Eigen isometry from " + ::std::to_string(v.size()) + " elements.");
    }
  })


}
