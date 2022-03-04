/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cras_cpp_common/time_utils.hpp>

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

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

}