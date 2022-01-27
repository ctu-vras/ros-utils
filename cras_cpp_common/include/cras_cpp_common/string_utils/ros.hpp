#pragma once

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <ros/message_traits.h>

namespace cras
{

template<typename T, typename std::enable_if_t<
    std::is_same<T, ros::Time>::value ||
    std::is_same<T, ros::WallTime>::value ||
    std::is_same<T, ros::SteadyTime>::value ||
    std::is_same<T, ros::Duration>::value ||
    std::is_same<T, ros::WallDuration>::value
  >* = nullptr>
inline std::string to_string(const T& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<typename T, typename std::enable_if_t<
    std::is_same<T, ros::Rate>::value ||
    std::is_same<T, ros::WallRate>::value
  >* = nullptr>
inline std::string to_string(const T& value)
{
  std::stringstream ss;
  ss << (1. / value.expectedCycleTime().toSec());
  return ss.str();
}

template<typename M, std::enable_if_t<ros::message_traits::IsMessage<M>::value>* = nullptr>
inline std::string to_string(const M& msg)
{
  std::stringstream ss;
  ss << msg;
  std::string s = ss.str();
  if (!s.empty() && s[s.length() - 1] == '\n')
    s.erase(s.length()-1);
  ::cras::replace(s, "\n", ", ");
  return s;
}

}
