#pragma once

/**
 * \file
 * \brief Specializations of cras::to_string() for Eigen types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sstream>
#include <string>

namespace cras
{

template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
inline ::std::string to_string(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& value)
{
  ::std::stringstream ss;
  ss << value.format({6, ::Eigen::DontAlignCols, ", ", "; ", "[", "]", "[", "]"});
  return ss.str();
}

template<typename Derived, int Dim>
inline ::std::string to_string(const ::Eigen::RotationBase<Derived, Dim>& value)
{
  ::std::stringstream ss;
  ss << value.matrix().format({6, ::Eigen::DontAlignCols, ", ", "; ", "[", "]", "[", "]"});
  return ss.str();
}

template<typename Scalar, int Dim, int Mode, int Options>
inline ::std::string to_string(const ::Eigen::Transform<Scalar, Dim, Mode, Options>& value)
{
  ::std::stringstream ss;
  ss << value.matrix().format({6, ::Eigen::DontAlignCols, ", ", "; ", "[", "]", "[", "]"});
  return ss.str();
}

}
