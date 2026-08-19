#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of getParam() for geometry_msgs.
 * \author Martin Pecka
 *
 * \details Quaternion can be loaded either from 4 values (direct coeffs) or 3 values (roll, pitch, yaw in rad).
 * \details Transform can be loaded either from 6 values (3 translation + 3 rotation),
 *          7 values (3 translation + 4 rotation) or 16 values (column-wise transformation matrix).
 */

#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace cras {

#define DEFINE_CONVERTING_GET_PARAM_ROS_VECTOR3(resultType, scalarType, defaultUnit) \
DEFINE_CONVERTING_GET_PARAM(resultType, std::vector<scalarType>, defaultUnit, [](const ::std::vector<scalarType>& v){ \
  if (v.size() != 3) { \
    throw ::std::runtime_error( \
      ::cras::format("Cannot load {} parameter from an array of length {}", #resultType, v.size())); \
  } \
  resultType m; m.x = v[0]; m.y = v[1]; m.z = v[2]; \
  return m; \
})

DEFINE_CONVERTING_GET_PARAM_ROS_VECTOR3(::geometry_msgs::msg::Vector3, double, "")
DEFINE_CONVERTING_GET_PARAM_ROS_VECTOR3(::geometry_msgs::msg::Point, double, "")
DEFINE_CONVERTING_GET_PARAM_ROS_VECTOR3(::geometry_msgs::msg::Point32, float, "")

DEFINE_CONVERTING_GET_PARAM(::geometry_msgs::msg::Quaternion, ::std::vector<double>, "",
  [](const ::std::vector<double>& v) {
    if (v.size() != 3 && v.size() != 4) {
      throw ::std::runtime_error(::cras::format(
        "Cannot load geometry_msgs::Quaternion parameter from an array of length {}", v.size()));
    }
    ::geometry_msgs::msg::Quaternion m;
    if (v.size() == 4) {
      m.x = v[0]; m.y = v[1]; m.z = v[2]; m.w = v[3];
    } else {
      ::tf2::Quaternion q; q.setRPY(v[0], v[1], v[2]);
      m.x = q.getX(); m.y = q.getY(); m.z = q.getZ(); m.w = q.getW();
    }
    return m;
  }
)

DEFINE_CONVERTING_GET_PARAM(::geometry_msgs::msg::Transform, ::std::vector<double>, "",
  [](const ::std::vector<double>& v) {
    if (v.size() != 6 && v.size() != 7 && v.size() != 16) {
      throw ::std::runtime_error(::cras::format(
        "Cannot load geometry_msgs::Transform parameter from an array of length {}", v.size()));
    }
    ::geometry_msgs::msg::Transform m;
    if (v.size() == 6 || v.size() == 7) {
      m.translation.x = v[0]; m.translation.y = v[1]; m.translation.z = v[2];
      if (v.size() == 6) {
        ::tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]);
        m.rotation.x = q.getX(); m.rotation.y = q.getY(); m.rotation.z = q.getZ(); m.rotation.w = q.getW();
      } else {
        m.rotation.x = v[3]; m.rotation.y = v[4]; m.rotation.z = v[5]; m.rotation.w = v[6];
      }
    } else {
      m.translation.x = v[3]; m.translation.y = v[7]; m.translation.z = v[11];
      ::tf2::Matrix3x3 r(v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10]);
      ::tf2::Quaternion q;
      r.getRotation(q);
      m.rotation.x = q.getX(); m.rotation.y = q.getY(); m.rotation.z = q.getZ(); m.rotation.w = q.getW();
    }
    return m;
  }
)

DEFINE_CONVERTING_GET_PARAM(::geometry_msgs::msg::Pose, ::std::vector<double>, "",
  [](const ::std::vector<double>& v) {
    if (v.size() != 6 && v.size() != 7 && v.size() != 16) {
      throw ::std::runtime_error(::cras::format(
        "Cannot load geometry_msgs::Pose parameter from an array of length {}", v.size()));
    }
    ::geometry_msgs::msg::Pose m;
    if (v.size() == 6 || v.size() == 7) {
      m.position.x = v[0]; m.position.y = v[1]; m.position.z = v[2];
      if (v.size() == 6) {
        ::tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]);
        m.orientation.x = q.getX(); m.orientation.y = q.getY(); m.orientation.z = q.getZ(); m.orientation.w = q.getW();
      } else {
        m.orientation.x = v[3]; m.orientation.y = v[4]; m.orientation.z = v[5]; m.orientation.w = v[6];
      }
    } else {
      m.position.x = v[3]; m.position.y = v[7]; m.position.z = v[11];
      ::tf2::Matrix3x3 r(v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10]);
      ::tf2::Quaternion q;
      r.getRotation(q);
      m.orientation.x = q.getX(); m.orientation.y = q.getY(); m.orientation.z = q.getZ(); m.orientation.w = q.getW();
    }
    return m;
  })

}  // namespace cras
