#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of getParam() for ROS basic types.
 * \author Martin Pecka
 */

#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

namespace cras {

DEFINE_CONVERTING_GET_PARAM(::rclcpp::Duration, double, "s", ::rclcpp::Duration::from_seconds)
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::rclcpp::Time, double, "s")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::rclcpp::Rate, double, "Hz")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::rclcpp::WallRate, double, "Hz")

}  // namespace cras
