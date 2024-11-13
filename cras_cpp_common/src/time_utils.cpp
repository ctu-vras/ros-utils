/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <limits>
#include <string>

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <cras_cpp_common/string_utils/xmlrpc.hpp>
#include <cras_cpp_common/time_utils.hpp>

// Fallback for 128bit ints on armhf or non-gcc compilers
#ifndef __SIZEOF_INT128__
#include <boost/multiprecision/cpp_int.hpp>
#endif

namespace cras
{

#ifdef __SIZEOF_INT128__
typedef __int128_t cras_int128_t;
#else
// Fallback for 128bit ints on armhf or non-gcc compilers
typedef boost::multiprecision::int128_t cras_int128_t;
#endif

ros::Duration remainingTime(const ros::Time& query, const double timeout)
{
  return remainingTime(query, ros::Duration(timeout));
}

ros::Duration remainingTime(const ros::Time& query, const ros::Duration& timeout)
{
  const auto passed = ros::Time::now() - query;
  return (timeout > passed) ? timeout - passed : ros::Duration(0);
}

double frequency(const ros::Rate& rate, const bool maxCycleTimeMeansZero)
{
  if (maxCycleTimeMeansZero &&
    (rate.expectedCycleTime() == ros::DURATION_MAX || rate.expectedCycleTime() == ros::DURATION_MIN))
    return 0.0;
  return 1.0 / rate.expectedCycleTime().toSec();
}

double frequency(const ros::WallRate& rate, const bool maxCycleTimeMeansZero)
{
  const auto expectedNSec = rate.expectedCycleTime().toNSec();
  if (maxCycleTimeMeansZero &&
    (expectedNSec == ros::DURATION_MAX.toNSec() || expectedNSec == ros::DURATION_MIN.toNSec()))
    return 0.0;
  return 1.0 / rate.expectedCycleTime().toSec();
}

ros::Rate safeRate(double frequency)
{
  try
  {
    return {frequency};
  }
  catch (const std::runtime_error&)
  {
    return ros::Rate(frequency >= 0 ? ros::DURATION_MAX : ros::DURATION_MIN);
  }
}

ros::WallRate safeWallRate(double frequency)
{
  try
  {
    return {frequency};
  }
  catch (const std::runtime_error&)
  {
    return ros::WallRate(frequency >= 0 ? ros::DURATION_MAX : ros::DURATION_MIN);
  }
}

ros::Time nowFallbackToWall()
{
  try
  {
    return ros::Time::now();
  }
  catch (const ros::TimeNotInitializedException& ex)
  {
    const auto wall = ros::WallTime::now();
    return {wall.sec, wall.nsec};
  }
}

ros::Time saturateAdd(const ros::Time& time, const ros::Duration& duration)
{
  const auto nsec64 = static_cast<int64_t>(time.toNSec()) + duration.toNSec();
  const auto sec64 = nsec64 / 1000000000LL;
  if (sec64 < 0)
    return {0, 0};
  if (sec64 > std::numeric_limits<uint32_t>::max())
    return ros::TIME_MAX;
  return time + duration;
}

ros::WallTime saturateAdd(const ros::WallTime& time, const ros::WallDuration& duration)
{
  const auto nsec64 = static_cast<int64_t>(time.toNSec()) + duration.toNSec();
  const auto sec64 = nsec64 / 1000000000LL;
  if (sec64 < 0)
    return {0, 0};
  if (sec64 > std::numeric_limits<uint32_t>::max())
    return {ros::TIME_MAX.sec, ros::TIME_MAX.nsec};
  return time + duration;
}

ros::SteadyTime saturateAdd(const ros::SteadyTime& time, const ros::WallDuration& duration)
{
  const auto nsec64 = static_cast<int64_t>(time.toNSec()) + duration.toNSec();
  const auto sec64 = nsec64 / 1000000000LL;
  if (sec64 < 0)
    return {0, 0};
  if (sec64 > std::numeric_limits<uint32_t>::max())
    return {ros::TIME_MAX.sec, ros::TIME_MAX.nsec};
  return time + duration;
}

tm toStructTm(const ros::Time& time)
{
  const auto timet = static_cast<time_t>(time.sec);

  tm structTm{};
  const auto result = gmtime_r(&timet, &structTm);

  // This shouldn't ever happen. gmtime can return nullptr only if year overflows, and max year of ros::Time is far
  // from being able to overflow an int (even if 16-bit).
  if (result == nullptr)
    return structTm;

  return structTm;
}

cras::expected<ros::Time, std::string> fromStructTm(const tm& time)
{
  tm t = time;
#if _DEFAULT_SOURCE
  errno = 0;
  const auto timeSecs = timegm(&t);
#else
  const auto tz = getenv("TZ");
  setenv("TZ", "", 1);
  tzset();
  const auto timeSecs = mktime(&t);
  if (tz)
    setenv("TZ", tz, 1);
  else
    unsetenv("TZ");
  tzset();
#endif
  if (timeSecs == static_cast<time_t>(-1) || errno == EOVERFLOW)
    return cras::make_unexpected(cras::format(
      "Cannot convert the given tm struct to ROS time (timegm failed, errno=%d).", errno));
  if (timeSecs < 0)
    return cras::make_unexpected("Cannot convert the given tm struct to ROS time (negative seconds since 1970).");

  try
  {
    return ros::Time(timeSecs, 0);
  }
  catch (const std::runtime_error& e)
  {
    return cras::make_unexpected(cras::format("Cannot convert the given tm struct to ROS time (%s).", e.what()));
  }
}

int getYear(const ros::Time& time)
{
  return toStructTm(time).tm_year + 1900;
}

}

namespace ros
{

bool operator==(const Rate& r1, const Rate& r2)
{
  return r1.expectedCycleTime() == r2.expectedCycleTime();
}

bool operator==(const WallRate& r1, const WallRate& r2)
{
  return r1.expectedCycleTime() == r2.expectedCycleTime();
}

ros::Duration operator*(const ros::Duration& val1, const ros::Duration& val2)
{
  const auto s1 = static_cast<int64_t>(val1.sec);
  const auto s2 = static_cast<int64_t>(val2.sec);
  const auto ns1 = static_cast<int64_t>(val1.nsec);
  const auto ns2 = static_cast<int64_t>(val2.nsec);
  return ros::Duration().fromNSec(s1 * s2 * 1000000000LL + s1 * ns2 + s2 * ns1 + (ns1 * ns2) / 1000000000LL);
}

ros::Duration operator/(const ros::Duration& numerator, const ros::Duration& denominator)
{
  if (denominator.sec == 0 && denominator.nsec == 0)
    throw std::runtime_error("Division by zero");
  const auto numeratorLarge = static_cast<cras::cras_int128_t>(numerator.toNSec()) * 1000000000LL;
  return ros::Duration().fromNSec(static_cast<int64_t>(numeratorLarge / denominator.toNSec()));
}

ros::WallDuration operator*(const ros::WallDuration& val1, const ros::WallDuration& val2)
{
  const auto s1 = static_cast<int64_t>(val1.sec);
  const auto s2 = static_cast<int64_t>(val2.sec);
  const auto ns1 = static_cast<int64_t>(val1.nsec);
  const auto ns2 = static_cast<int64_t>(val2.nsec);
  return ros::WallDuration().fromNSec(s1 * s2 * 1000000000LL + s1 * ns2 + s2 * ns1 + (ns1 * ns2) / 1000000000LL);
}

ros::WallDuration operator/(const ros::WallDuration& numerator, const ros::WallDuration& denominator)
{
  if (denominator.sec == 0 && denominator.nsec == 0)
    throw std::runtime_error("Division by zero");
  const auto numeratorLarge = static_cast<cras::cras_int128_t>(numerator.toNSec()) * 1000000000LL;
  return ros::WallDuration().fromNSec(static_cast<int64_t>(numeratorLarge / denominator.toNSec()));
}

}
