#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of cras::to_string() for TF2 types.
 * \author Martin Pecka
 */

#include <format>
#include <string>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace cras
{

inline ::std::string to_string(const ::tf2::Vector3& value)
{
  return ::std::format("[{:.6f}, {:.6f}, {:.6f}]", value.getX(), value.getY(), value.getZ());
}

inline ::std::string to_string(const ::tf2::Quaternion& value)
{
  ::tf2::Matrix3x3 m;
  m.setRotation(value);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return ::std::format("[x={:.6f}, y={:.6f}, z={:.6f}, w={:.6f} (r={:.3f}, p={:.3f}, y={:.3f})]",
                       value.getX(), value.getY(), value.getZ(), value.getW(), roll, pitch, yaw);
}

inline ::std::string to_string(const ::tf2::Matrix3x3& value)
{
  return ::std::format("[[{:.6f}, {:.6f}, {:.6f}]; [{:.6f}, {:.6f}, {:.6f}]; [{:.6f}, {:.6f}, {:.6f}]]",
                       value.getRow(0).getX(), value.getRow(0).getY(), value.getRow(0).getZ(),
                       value.getRow(1).getX(), value.getRow(1).getY(), value.getRow(1).getZ(),
                       value.getRow(2).getX(), value.getRow(2).getY(), value.getRow(2).getZ());
}

inline ::std::string to_string(const ::tf2::Transform& value)
{
  return "Transform(t=" + ::cras::to_string(value.getOrigin()) + ", r=" +
         ::cras::to_string(value.getRotation()) + ")";
}

}
