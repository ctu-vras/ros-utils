/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

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

};

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
	const auto numeratorLarge = static_cast<__int128_t>(numerator.toNSec()) * 1000000000LL;
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
	const auto numeratorLarge = static_cast<__int128_t>(numerator.toNSec()) * 1000000000LL;
	return ros::WallDuration().fromNSec(static_cast<int64_t>(numeratorLarge / denominator.toNSec()));
}

}