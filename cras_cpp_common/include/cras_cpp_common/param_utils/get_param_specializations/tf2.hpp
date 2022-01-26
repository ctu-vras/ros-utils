#pragma once

/**
 * \file
 * \brief Specializations of getParam() for tf2 types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 * 
 * \details Quaternion can be loaded either from 4 values (direct coeffs) or 3 values (roll, pitch, yaw in rad).
 * \details Transform can be loaded either from 6 values (3 translation + 3 rotation),
 *          7 values (3 translation + 4 rotation) or 16 values (column-wise transformation matrix).
 */

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace cras {

DEFINE_CONVERTING_GET_PARAM(::tf2::Vector3, ::std::vector<double>, "", [](const ::std::vector<double>& v) {
  if (v.size() != 3)
    throw ::std::runtime_error(::cras::format("Cannot load %s parameter from an array of length %lu",
      "tf2::Vector3", v.size()));
  return ::tf2::Vector3(v[0], v[1], v[2]);
})

DEFINE_CONVERTING_GET_PARAM(::tf2::Quaternion, ::std::vector<double>, "", [](const ::std::vector<double>& v) {
  if (v.size() != 3 && v.size() != 4)
    throw ::std::runtime_error(::cras::format("Cannot load %s parameter from an array of length %lu",
      "tf2::Quaternion", v.size()));
  ::tf2::Quaternion m;
  if (v.size() == 4)
  {
    m.setX(v[0]); m.setY(v[1]); m.setZ(v[2]); m.setW(v[3]);
  }
  else
  {
    m.setRPY(v[0], v[1], v[2]);
  }
  return m;
})

DEFINE_CONVERTING_GET_PARAM(::tf2::Transform, ::std::vector<double>, "", [](const ::std::vector<double>& v) {
  if (v.size() != 6 && v.size() != 7 && v.size() != 16)
    throw ::std::runtime_error(::cras::format("Cannot load %s parameter from an array of length %lu",
      "tf2::Transform", v.size()));
  ::tf2::Transform m;
  if (v.size() == 6 || v.size() == 7)
  {
    m.setOrigin({v[0], v[1], v[2]});
    if (v.size() == 6)
    {
      ::tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]);
      m.setRotation(q);
    }
    else
    {
      m.setRotation({v[3], v[4], v[5], v[6]});
    }
  }
  else
  {
    m.setOrigin({v[3], v[7], v[11]});
    m.setBasis({v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10]});
  }
  return m;
})

}
