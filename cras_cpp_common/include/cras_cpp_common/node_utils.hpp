#pragma once

#include <string>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/param_utils.hpp>

/**
 * This file adds utility functions to be used within nodes. So far it only makes ParamHelper templated getParam()
 * functions accessible via static global calls cras::getParam(nh, ...).
 */

namespace cras {

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class NodeLogHelper : public LogHelper {
protected:
  void printDebug(const std::string &text) const override { ROS_DEBUG("%s", text.c_str()); }
  void printInfo(const std::string &text) const override { ROS_INFO("%s", text.c_str()); }
  void printWarn(const std::string &text) const override { ROS_WARN("%s", text.c_str()); }
  void printError(const std::string &text) const override { ROS_ERROR("%s", text.c_str()); }
  void printFatal(const std::string &text) const override { ROS_FATAL("%s", text.c_str()); }
};

/** Static variable that serves as the singleton for getting node parameters. */
const ParamHelper paramHelper(std::make_shared<NodeLogHelper>());

// These functions are forward-declared with default parameters in param_utils.hpp

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value, and print out a
 *        ROS info/warning message with the loaded values.
 * \tparam T Param type (the C++ type; various specializations make it possible to convert different parameter server
 *         values to the corresponding C++ type if the conversion is non-trivial).
 * \param node The node handle to read the param value from.
 * \param name Name of the parameter.
 * \param defaultValue The default value to use.
 * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
 * \return The loaded param value.
 */
template<typename T>
inline T getParam(ros::NodeHandle &node, const std::string &name, const T &defaultValue, const std::string &unit)
{
  const auto param = cras::NodeRawGetParamAdapter(node);
  return paramHelper.getParam(param, name, defaultValue, unit);
}

// std::string - char interop specializations
inline std::string getParam(ros::NodeHandle &node, const std::string &name, const char *defaultValue, const std::string &unit)
{
  const auto param = cras::NodeRawGetParamAdapter(node);
  return paramHelper.getParam(param, name, defaultValue, unit);
}

}
