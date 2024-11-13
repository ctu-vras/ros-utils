/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <ctime>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

namespace cras
{

template<typename T, typename ::std::enable_if_t<
    ::std::is_same<T, ::ros::Time>::value ||
    ::std::is_same<T, ::ros::WallTime>::value ||
    ::std::is_same<T, ::ros::SteadyTime>::value
  >* = nullptr>
struct DurationType
{
};

template<> struct DurationType<ros::Time>
{
  typedef ::ros::Duration value;
};

template<> struct DurationType<ros::WallTime>
{
  typedef ::ros::WallDuration value;
};

template<> struct DurationType<ros::SteadyTime>
{
  typedef ::ros::WallDuration value;
};

template<
  typename T1,
  typename T2,
  typename ::std::enable_if_t<
    ::std::is_same<T1, ::ros::Time>::value ||
    ::std::is_same<T1, ::ros::WallTime>::value ||
    ::std::is_same<T1, ::ros::SteadyTime>::value>* = nullptr,
  typename ::std::enable_if_t<
    ::std::is_same<T2, ::ros::Time>::value ||
    ::std::is_same<T2, ::ros::WallTime>::value ||
    ::std::is_same<T2, ::ros::SteadyTime>::value>* = nullptr
>
inline T1 convertTime(const T2& time)
{
  return T1{time.sec, time.nsec};
}

template<
  typename D1,
  typename D2,
  typename ::std::enable_if_t<
    ::std::is_same<D1, ::ros::Duration>::value ||
    ::std::is_same<D1, ::ros::WallDuration>::value>* = nullptr,
  typename ::std::enable_if_t<
    ::std::is_same<D2, ::ros::Duration>::value ||
    ::std::is_same<D2, ::ros::WallDuration>::value>* = nullptr
>
inline D1 convertDuration(const D2& duration)
{
  return D1{duration.sec, duration.nsec};
}

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

/**
 * \brief Return current ROS time if it has already been initialized, or current wall time.
 * \return Current time.
 */
::ros::Time nowFallbackToWall();

/**
 * \brief Add the given duration to the given time, but saturate the result instead of throwing exception on overflow.
 * \param[in] time The time to be added to.
 * \param[in] duration The duration to add.
 * \return The time plus the duration saturated between 0 and TIME_MAX.
 */
::ros::Time saturateAdd(const ::ros::Time& time, const ::ros::Duration& duration);

/**
 * \brief Add the given duration to the given time, but saturate the result instead of throwing exception on overflow.
 * \param[in] time The time to be added to.
 * \param[in] duration The duration to add.
 * \return The time plus the duration saturated between 0 and TIME_MAX.
 */
::ros::WallTime saturateAdd(const ::ros::WallTime& time, const ::ros::WallDuration& duration);

/**
 * \brief Add the given duration to the given time, but saturate the result instead of throwing exception on overflow.
 * \param[in] time The time to be added to.
 * \param[in] duration The duration to add.
 * \return The time plus the duration saturated between 0 and TIME_MAX.
 */
::ros::SteadyTime saturateAdd(const ::ros::SteadyTime& time, const ::ros::WallDuration& duration);

/**
 * \brief Convert the given ROS time to C tm struct representing UTC time.
 * \param[in] time The ROS time to convert.
 * \return The struct tm with corresponding time (please remember that year is offset from 1900 and month is 0-based).
 * \note The fractional seconds will be lost during the conversion.
 */
::tm toStructTm(const ::ros::Time& time);

/**
 * \brief Convert the given ROS time to C tm struct representing UTC time.
 * \param[in] time The ROS time to convert.
 * \return The struct tm with corresponding time (please remember that year is offset from 1900 and month is 0-based).
 * \note The fractional seconds will be lost during the conversion.
 */
inline ::tm toStructTm(const ::ros::WallTime& time)
{
  return ::cras::toStructTm(::cras::convertTime<::ros::Time>(time));
}

/**
 * \brief Convert the given ROS time to C tm struct representing UTC time.
 * \param[in] time The ROS time to convert.
 * \return The struct tm with corresponding time (please remember that year is offset from 1900 and month is 0-based).
 * \note The fractional seconds will be lost during the conversion.
 */
inline ::tm toStructTm(const ::ros::SteadyTime& time)
{
  return ::cras::toStructTm(::cras::convertTime<::ros::Time>(time));
}

::cras::expected<::ros::Time, ::std::string> fromStructTm(const ::tm& time);

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The ROS time.
 * \return The year.
 */
int getYear(const ::ros::Time& time);

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The ROS time.
 * \return The year.
 */
inline int getYear(const ::ros::WallTime& time)
{
  return ::cras::getYear(::cras::convertTime<::ros::Time>(time));
}

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The ROS time.
 * \return The year.
 */
inline int getYear(const ::ros::SteadyTime& time)
{
  return ::cras::getYear(::cras::convertTime<::ros::Time>(time));
}

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

/**
 * \brief Multiply two durations. The units of the result are [s^2].
 * \param[in] val1 Duration to multiply.
 * \param[in] val2 Duration to multiply.
 * \return The multiple.
 * \note This function will throw an exception if the result is out of bounds of the standard ROS duration range.
 */
::ros::Duration operator*(const ::ros::Duration& val1, const ::ros::Duration& val2);

/**
 * \brief Divide duration val1 by val2. The result is unitless.
 * \param[in] numerator Duration to multiply.
 * \param[in] denominator Duration to multiply.
 * \return The fraction.
 * \note This function will throw an exception if the result is out of bounds of the standard ROS duration range.
 */
::ros::Duration operator/(const ::ros::Duration& numerator, const ::ros::Duration& denominator);

/**
 * \brief Multiply two durations. The units of the result are [s^2].
 * \param[in] val1 Duration to multiply.
 * \param[in] val2 Duration to multiply.
 * \return The multiple.
 * \note This function will throw an exception if the result is out of bounds of the standard ROS duration range.
 */
::ros::WallDuration operator*(const ::ros::WallDuration& val1, const ::ros::WallDuration& val2);

/**
 * \brief Divide duration val1 by val2. The result is unitless.
 * \param[in] numerator Duration to multiply.
 * \param[in] denominator Duration to multiply.
 * \return The fraction.
 * \note This function will throw an exception if the result is out of bounds of the standard ROS duration range.
 */
::ros::WallDuration operator/(const ::ros::WallDuration& numerator, const ::ros::WallDuration& denominator);

}
