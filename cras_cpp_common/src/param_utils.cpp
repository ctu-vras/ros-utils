// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS parameter helpers.
 * \author Martin Pecka
 */

#include <cras_cpp_common/param_utils.hpp>

namespace cras {

ParamHelper::ParamHelper(const RequiredInterfaces& node_interfaces, const std::string& log_prefix)
  : log_prefix_(log_prefix), node_interfaces_(node_interfaces) {}

bool ParamHelper::hasParam(const std::string& name, const bool allow_undeclared) const {
  return cras::hasParam(node_interfaces_.get_node_parameters_interface(), name, allow_undeclared);
}

cras::GetParamResult<std::string> ParamHelper::getParamVerbose(
  const std::string& name, const std::optional<const char*>& default_value, const std::string& unit,
  const cras::GetParamOptions<std::string>& options) {
  std::optional<std::string> new_default;
  if (default_value.has_value()) {
    new_default = *default_value;
  }
  return getParamVerbose(name, new_default, unit, options);
}

cras::GetParamResult<std::string> ParamHelper::getParamVerbose(
  const std::string& name, const char* default_value, const std::string& unit,
  const cras::GetParamOptions<std::string>& options) {
  return getParamVerbose(name, std::string(default_value), unit, options);
}

std::string ParamHelper::getParam(
  const std::string& name, const std::optional<const char*>& default_value, const std::string& unit,
  const cras::GetParamOptions<std::string>& options) {
  std::optional<std::string> new_default;
  if (default_value.has_value()) {
    new_default = *default_value;
  }
  return getParam(name, new_default, unit, options);
}

std::string ParamHelper::getParam(
  const std::string& name, const char* default_value, const std::string& unit,
  const cras::GetParamOptions<std::string>& options) {
  return getParam(name, std::string(default_value), unit, options);
}

GetParamException::GetParamException(const cras::GetParamResultInfo& info)
  : std::runtime_error(info.message), info(info) {}

bool hasParam(
  const rclcpp::node_interfaces::NodeParametersInterface::ConstSharedPtr& params, const std::string& name,
  const bool allow_undeclared) {
  if (params->has_parameter(name)) {
    return true;
  }
  if (!allow_undeclared) {
    return false;
  }
  const auto overrides = params->get_parameter_overrides();
  return overrides.contains(name);
}

cras::GetParamResult<std::string> getParamVerbose(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params, const std::string& name,
  const std::optional<const char*>& default_value, const std::string& unit,
  const cras::GetParamOptions<std::string>& options,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger, const std::string& log_prefix) {
  ::std::optional<::std::string> default_str;
  if (default_value.has_value()) {
    default_str = default_value.value();
  }
  return ::cras::getParamVerbose(params, name, default_str, unit, options, logger, log_prefix);
}

cras::GetParamResult<std::string> getParamVerbose(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params, const std::string& name,
  const char* default_value, const std::string& unit, const cras::GetParamOptions<std::string>& options,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger, const std::string& log_prefix) {
  ::std::optional<::std::string> default_str(default_value);
  return ::cras::getParamVerbose(params, name, default_str, unit, options, logger, log_prefix);
}

std::string getParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params, const std::string& name,
  const std::optional<const char*>& default_value, const std::string& unit,
  const cras::GetParamOptions<std::string>& options,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger, const std::string& log_prefix) {
  return ::cras::getParamVerbose(params, name, default_value, unit, options, logger, log_prefix).value;
}

std::string getParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params, const std::string& name,
  const char* default_value, const std::string& unit, const cras::GetParamOptions<std::string>& options,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger, const std::string& log_prefix) {
  return ::cras::getParamVerbose(params, name, default_value, unit, options, logger, log_prefix).value;
}

}  // namespace cras
