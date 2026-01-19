/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <chrono>
#include <ctime>
#include <limits>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/format.hpp>
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

rclcpp::Duration remainingTime(const rclcpp::Time& query, const double timeout,
  const rclcpp::Clock::ConstSharedPtr& clock)
{
  return remainingTime(query, rclcpp::Duration::from_seconds(timeout), clock);
}

rclcpp::Duration remainingTime(const rclcpp::Time& query, const rclcpp::Duration& timeout,
  const rclcpp::Clock::ConstSharedPtr& clock)
{
  const auto passed = clock->now() - query;
  return (timeout > passed) ? timeout - passed : rclcpp::Duration(0, 0);
}

double frequency(const rclcpp::Rate& rate, const bool maxPeriodMeansZero)
{
  if (maxPeriodMeansZero &&
    (rate.period().count() == rclcpp::Duration::max().nanoseconds() || rate.period().count() == 0))
    return 0.0;
  return 1.0 / std::chrono::duration_cast<std::chrono::duration<double>>(rate.period()).count();
}

rclcpp::Rate safeRate(const double frequency, const rclcpp::Clock::SharedPtr& clock)
{
  if (frequency <= 0)
    return rclcpp::Rate(rclcpp::Duration::max(), clock);

  const auto dur = 1.0 / frequency;
  if (dur >= rclcpp::Duration::max().seconds())
    return rclcpp::Rate(rclcpp::Duration::max(), clock);

  const auto duration = rclcpp::Duration::from_seconds(dur);
  if (duration.nanoseconds() > 0)
    return rclcpp::Rate(duration, clock);
  else
    return rclcpp::Rate(rclcpp::Duration(0, 1), clock);
}

rclcpp::WallRate safeWallRate(const double frequency)
{
  if (frequency <= 0)
    return rclcpp::WallRate(rclcpp::Duration::max());

  const auto dur = 1.0 / frequency;
  if (dur >= rclcpp::Duration::max().seconds())
    return rclcpp::WallRate(rclcpp::Duration::max());

  const auto duration = rclcpp::Duration::from_seconds(dur);
  if (duration.nanoseconds() > 0)
    return rclcpp::WallRate(duration);
  else
    return rclcpp::WallRate(rclcpp::Duration(0, 1));
}

/*
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
*/

rclcpp::Time saturateAdd(const rclcpp::Time& time, const rclcpp::Duration& duration)
{
  const auto nsec = time.nanoseconds() + duration.nanoseconds();
  const auto clockType = time.get_clock_type();

  // *INDENT-OFF*
  if (nsec < 0)
    return {0, 0, clockType};
  // *INDENT-ON*

  if (nsec > rclcpp::Time::max(clockType).nanoseconds())
    return rclcpp::Time::max(clockType);

  return rclcpp::Time(nsec, clockType);
}

template<>
builtin_interfaces::msg::Time convertTime(const rclcpp::Time& t)
{
  return rclcpp::convert_rcl_time_to_sec_nanos(t.nanoseconds());
}

template<>
double convertTime(const rclcpp::Time& t)
{
  return t.seconds();
}

template<>
rcl_time_point_value_t convertTime(const rclcpp::Time& t)
{
  return t.nanoseconds();
}

template<>
tm convertTime(const rclcpp::Time& t)
{
  const auto timet = static_cast<time_t>(cras::sec(t));

  tm structTm{};
  const auto result = gmtime_r(&timet, &structTm);

  // This shouldn't ever happen. gmtime can return nullptr only if year overflows, and max year of ros::Time is far
  // from being able to overflow an int (even if 16-bit).
  if (result == nullptr)
    return structTm;

  return structTm;
}

template<>
std::chrono::system_clock::time_point convertTime(const rclcpp::Time& t)
{
  return std::chrono::system_clock::time_point(std::chrono::nanoseconds(t.nanoseconds()));
}

template<>
rclcpp::Time convertTime(const builtin_interfaces::msg::Time& t)
{
  return {t, RCL_SYSTEM_TIME};
}

template<>
rclcpp::Time convertTime(const double& t)
{
  return {static_cast<int32_t>(std::floor(t)), static_cast<uint32_t>((t - std::floor(t)) * 1e9)};
}

template<>
rclcpp::Time convertTime(const rcl_time_point_value_t& t)
{
  return rclcpp::Time(t);
}

template<>
double convertTime(const rcl_time_point_value_t& t)
{
  return t * 1e-9;
}

template<>
rclcpp::Time convertTime(const rmw_time_t& t)
{
  return rclcpp::Time(rmw_time_total_nsec(t));
}

cras::expected<rclcpp::Time, std::string> fromStructTm(const tm& time)
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
      "Cannot convert the given tm struct to ROS time (timegm failed, errno={}).", errno));
  if (timeSecs < 0)
    return cras::make_unexpected("Cannot convert the given tm struct to ROS time (negative seconds since 1970).");

  try
  {
    return rclcpp::Time(timeSecs, 0);
  }
  catch (const std::runtime_error& e)
  {
    return cras::make_unexpected(cras::format("Cannot convert the given tm struct to ROS time ({}).", e.what()));
  }
}

template<>
builtin_interfaces::msg::Duration convertDuration(const rclcpp::Duration& t)
{
  return t;
}

template<>
double convertDuration(const rclcpp::Duration& t)
{
  return t.seconds();
}

template<>
rcl_duration_value_t convertDuration(const rclcpp::Duration& t)
{
  return t.nanoseconds();
}

template<>
rmw_time_t convertDuration(const rclcpp::Duration& t)
{
  return t.to_rmw_time();
}

template<>
rclcpp::Duration convertDuration(const builtin_interfaces::msg::Duration& t)
{
  return t;
}

template<>
rclcpp::Duration convertDuration(const double& t)
{
  return rclcpp::Duration::from_seconds(t);
}

template<>
rclcpp::Duration convertDuration(const rcl_duration_value_t& t)
{
  return rclcpp::Duration::from_nanoseconds(t);
}

template<>
rclcpp::Duration convertDuration(const rcl_duration_t& t)
{
  return rclcpp::Duration::from_nanoseconds(t.nanoseconds);
}

template<>
double convertDuration(const rcl_duration_value_t& t)
{
  return t * 1e-9;
}

template<>
rclcpp::Duration convertDuration(const rmw_time_t& t)
{
  return rclcpp::Duration::from_rmw_time(t);
}

template<>
rclcpp::Time convertTime(const tm& t)
{
  const auto maybeTime = fromStructTm(t);
  if (!maybeTime.has_value())
    throw std::runtime_error(maybeTime.error());
  return *maybeTime;
}

int getYear(const rclcpp::Time& time)
{
  return convertTime<tm>(time).tm_year + 1900;
}

}

namespace rclcpp
{

bool operator==(const Rate& r1, const Rate& r2)
{
  return r1.get_type() == r2.get_type() && r1.period() == r2.period();
}

rclcpp::Duration operator*(const rclcpp::Duration& val1, const rclcpp::Duration& val2)
{
  const auto sn1 = cras::secNsec(val1);
  const auto sn2 = cras::secNsec(val2);
  const auto s1 = static_cast<int64_t>(sn1.first);
  const auto ns1 = static_cast<int64_t>(sn1.second);
  const auto s2 = static_cast<int64_t>(sn2.first);
  const auto ns2 = static_cast<int64_t>(sn2.second);

  const auto nanosecondsLarge = static_cast<cras::cras_int128_t>(s1 * s2) * 1000000000LL +
    s1 * ns2 + s2 * ns1 +
    ns1 * ns2 / 1000000000LL;

  if (nanosecondsLarge > rclcpp::Duration::max().nanoseconds())
    throw std::invalid_argument("Overflow in duration multiplication.");

  return rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(nanosecondsLarge));
}

rclcpp::Duration operator/(const rclcpp::Duration& numerator, const rclcpp::Duration& denominator)
{
  if (denominator == rclcpp::Duration(0, 0))
    throw std::runtime_error("Division by zero");

  const auto numeratorLarge = static_cast<cras::cras_int128_t>(numerator.nanoseconds()) * 1000000000LL;
  return rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(numeratorLarge / denominator.nanoseconds()));
}

}
