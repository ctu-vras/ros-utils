/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

namespace cras
{

/**
 * \brief Return remaining time to timeout from the query time.
 * \param[in] query The query time, e.g. of a TF.
 * \param[in] timeout Maximum time to wait from the query time onwards.
 * \return The remaining time.
 */
::ros::Duration remainingTime(const ::ros::Time& query, double timeout);

/**
 * \brief Return remaining time to timeout from the query time.
 * \param[in] query The query time, e.g. of a TF.
 * \param[in] timeout Maximum time to wait from the query time onwards.
 * \return The remaining time.
 */
::ros::Duration remainingTime(const ::ros::Time& query, const ::ros::Duration& timeout);

/**
 * \brief Return the frequency represented by the given rate.
 * \param[in] rate The rate to convert.
 * \param[in] maxCycleTimeMeansZero If true, return 0 frequency in case the rate's cycle time is the maximum duration.
 * \return The frequency.
 */
double frequency(const ::ros::Rate& rate, bool maxCycleTimeMeansZero = false);

/**
 * \brief Return the frequency represented by the given rate.
 * \param[in] rate The rate to convert.
 * \param[in] maxCycleTimeMeansZero If true, return 0 frequency in case the rate's cycle time is the maximum duration.
 * \return The frequency.
 */
double frequency(const ::ros::WallRate& rate, bool maxCycleTimeMeansZero = false);

/**
 * \brief Return a rate representing the given frequency. If the frequency is zero or too small, return min/max
 * representable rate.
 * \param[in] frequency The frequency to convert.
 * \return The corresponding Rate object.
 */
::ros::Rate safeRate(double frequency);

/**
 * \brief Return a rate representing the given frequency. If the frequency is zero or too small, return min/max
 * representable rate.
 * \param[in] frequency The frequency to convert.
 * \return The corresponding Rate object.
 */
::ros::WallRate safeWallRate(double frequency);

}

namespace ros
{

/**
 * \brief Test two rates for equality.
 * \param[in] r1 First rate. 
 * \param[in] r2 Second rate.
 * \return Whether the rates are exactly equal.
 */
bool operator==(const ::ros::Rate& r1, const ::ros::Rate& r2);

/**
 * \brief Test two rates for equality.
 * \param[in] r1 First rate. 
 * \param[in] r2 Second rate.
 * \return Whether the rates are exactly equal.
 */
bool operator==(const ::ros::WallRate& r1, const ::ros::WallRate& r2);

}