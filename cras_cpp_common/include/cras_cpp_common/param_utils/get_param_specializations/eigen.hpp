#pragma once

/**
 * \file
 * \brief Specializations of getParam() for Eigen vectors and matrices.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <Eigen/Core>

namespace cras {

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector3d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v){ return ::Eigen::Vector3d(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector3f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v){ return ::Eigen::Vector3f(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector4d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v){ return ::Eigen::Vector4d(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Vector4f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v){ return ::Eigen::Vector4f(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix3d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v){ return ::Eigen::Matrix3d(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix3f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v){ return ::Eigen::Matrix3f(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix4d, ::std::vector<double>, "",
  [](const ::std::vector<double>& v){ return ::Eigen::Matrix4d(v.data()); })

DEFINE_CONVERTING_GET_PARAM(::Eigen::Matrix4f, ::std::vector<float>, "",
  [](const ::std::vector<float>& v){ return ::Eigen::Matrix4f(v.data()); })


}
