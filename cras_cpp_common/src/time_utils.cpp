#include <cras_cpp_common/time_utils.hpp>

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>

ros::Duration cras::remainingTime(const ros::Time &query, const double timeout)
{
  const auto passed = (ros::Time::now() - query).toSec();
  return ros::Duration(std::max(0.0, timeout - passed));
}

ros::Duration cras::remainingTime(const ros::Time &query, const ros::Duration &timeout)
{
  const auto passed = ros::Time::now() - query;
  const auto remaining = timeout - passed;
  return (remaining.sec >= 0) ? remaining : ros::Duration(0);
}

double cras::frequency(const ros::Rate& rate, const bool maxCycleTimeMeansZero)
{
  if (maxCycleTimeMeansZero &&
    (rate.expectedCycleTime() == ros::DURATION_MAX || rate.expectedCycleTime() == ros::DURATION_MIN))
      return 0.0;
  return 1.0 / rate.expectedCycleTime().toSec();
}

double cras::frequency(const ros::WallRate& rate, const bool maxCycleTimeMeansZero)
{
  const auto expectedNSec = rate.expectedCycleTime().toNSec();
  if (maxCycleTimeMeansZero &&
    (expectedNSec == ros::DURATION_MAX.toNSec() || expectedNSec == ros::DURATION_MIN.toNSec()))
      return 0.0;
  return 1.0 / rate.expectedCycleTime().toSec();
}

ros::Rate cras::safeRate(double frequency)
{
  try
  {
    return ros::Rate(frequency);
  }
  catch (const std::runtime_error&)
  {
    return ros::Rate(frequency >= 0 ? ros::DURATION_MAX : ros::DURATION_MIN);
  }
}

ros::WallRate cras::safeWallRate(double frequency)
{
  try
  {
    return ros::WallRate(frequency);
  }
  catch (const std::runtime_error&)
  {
    return ros::WallRate(frequency >= 0 ? ros::DURATION_MAX : ros::DURATION_MIN);
  }
}

bool ros::operator==(const ros::Rate& r1, const ros::Rate& r2)
{
  return r1.expectedCycleTime().operator==(r2.expectedCycleTime());
}

bool ros::operator==(const ros::WallRate& r1, const ros::WallRate& r2)
{
  return r1.expectedCycleTime().operator==(r2.expectedCycleTime());
}
