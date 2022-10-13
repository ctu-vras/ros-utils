#pragma once

/**
 * \file
 * \brief Specializations of getParam() for ROS basic types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

namespace cras {

DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::Time, double, "s")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::Duration, double, "s")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::Rate, double, "Hz")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::WallTime, double, "s")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::WallDuration, double, "s")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::WallRate, double, "Hz")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::ros::SteadyTime, double, "s")

}
