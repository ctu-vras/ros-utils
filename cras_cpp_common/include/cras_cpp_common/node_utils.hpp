#ifndef CRAS_CPP_COMMON_NODE_UTILS_HPP
#define CRAS_CPP_COMMON_NODE_UTILS_HPP

#include <string>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include "cras_cpp_common/string_utils.hpp"

namespace cras {

/**
 * \brief Get the value of the given ROS parameter, falling back to the
 *        specified default value, and print out a ROS info/warning message with
 *        the loaded values.
 * \tparam T Param type.
 * \param node The node handle to read the param value from.
 * \param name Name of the parameter.
 * \param defaultValue The default value to use.
 * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
 * \return The loaded param value.
 */
template<typename T>
inline T getParam(ros::NodeHandle &node, const std::string &name,
           const T &defaultValue = T(), const std::string &unit = "")
{
  T value;
  if (node.getParam(name, value))
  {
    ROS_INFO_STREAM(node.getNamespace() << ": Found parameter: " << name <<
      ", value: " << cras::to_string(value) <<
      cras::prependIfNonEmpty(unit, " "));
    return value;
  }
  else
  {
    ROS_WARN_STREAM(node.getNamespace() << ": Cannot find value for parameter: "
      << name << ", assigning default: " << cras::to_string(defaultValue) <<
      cras::prependIfNonEmpty(unit, " "));
  }
  return defaultValue;
}

// std::string - char interop specializations

inline std::string getParam(ros::NodeHandle &node, const std::string &name, const char *defaultValue, const std::string &unit = "")
{
  return getParam<std::string>(node, name, std::string(defaultValue), unit);
}

// getParam specializations for unsigned values

namespace impl
{
template <typename Result, typename Param>
inline Result getParamUnsigned(ros::NodeHandle &node, const std::string &name, const Result &defaultValue, const std::string &unit = "")
{
  const Param signedValue = getParam(node, name, static_cast<Param>(defaultValue), unit);
  if (signedValue < 0)
  {
    ROS_ERROR_STREAM(node.getNamespace() << ": Value " << signedValue <<
      " of unsigned parameter " << name << " is negative.");
    throw std::invalid_argument(name);
  }
  return static_cast<Result>(signedValue);
}
};

template <>
inline size_t getParam(ros::NodeHandle &node, const std::string &name, const size_t &defaultValue, const std::string &unit)
{
  return impl::getParamUnsigned<size_t, int>(node, name, defaultValue, unit);
}

template <>
inline unsigned int getParam(ros::NodeHandle &node, const std::string &name, const unsigned int &defaultValue, const std::string &unit)
{
  return impl::getParamUnsigned<unsigned int, int>(node, name, defaultValue, unit);
}

namespace impl
{
// generic casting getParam()
template <typename Result, typename Param>
inline Result getParamCast(ros::NodeHandle &node, const std::string &name, const Param &defaultValue, const std::string &unit = "")
{
  const Param paramValue = getParam(node, name, defaultValue, unit);
  return Result(paramValue);
}
};

// ROS types specializations

template <>
inline ros::Duration getParam(ros::NodeHandle &node, const std::string &name, const ros::Duration &defaultValue, const std::string &unit)
{
  return impl::getParamCast<ros::Duration, double>(node, name, defaultValue.toSec(), unit);
}

};

#endif //CRAS_CPP_COMMON_NODE_UTILS_HPP