#include "cras_cpp_common/time_utils.hpp"

namespace cras {

ros::Duration remainingTime(const ros::Time &query, const double timeout)
{
  const auto passed = (ros::Time::now() - query).toSec();
  return ros::Duration(std::max(0.0, timeout - passed));
}

ros::Duration remainingTime(const ros::Time &query,
                            const ros::Duration &timeout)
{
  const auto passed = ros::Time::now() - query;
  const auto remaining = timeout - passed;
  return (remaining.sec >= 0) ? remaining : ros::Duration(0);
}

std::string to_string(const ros::Time& time)
{
  std::stringstream ss;
  ss << time;
  return ss.str();
}

std::string to_string(const ros::WallTime& time)
{
  std::stringstream ss;
  ss << time;
  return ss.str();
}

std::string to_string(const ros::SteadyTime& time)
{
  std::stringstream ss;
  ss << time;
  return ss.str();
}

std::string to_string(const ros::Duration& time)
{
  std::stringstream ss;
  ss << time;
  return ss.str();
}

std::string to_string(const ros::WallDuration& time)
{
  std::stringstream ss;
  ss << time;
  return ss.str();
}

};