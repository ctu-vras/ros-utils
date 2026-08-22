#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Base for filters that eases getting parameters and more node interfaces.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <filters/filter_base.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

namespace cras {

class FilterNodeInterfaces {
public:
  FilterNodeInterfaces(
      const ::std::string& name,
      const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
      const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging);
  virtual ~FilterNodeInterfaces();

  virtual ::rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;
  virtual ::rclcpp::node_interfaces::NodeClockInterface::SharedPtr get_node_clock_interface() const;
  virtual ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr get_node_logging_interface() const;
  virtual ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() const;
  virtual ::rclcpp::node_interfaces::NodeServicesInterface::SharedPtr get_node_services_interface() const;
  virtual ::rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() const;

private:
  struct Impl;
  ::std::unique_ptr<Impl> impl_;
};

template<typename F>
class FilterBase : public ::filters::FilterBase<F> {
protected:
  bool configure() override {
    ::cras::ParamHelper::RequiredInterfaces params_interfaces{this->logging_interface_, this->params_interface_};
    params_ = ::std::make_unique<::cras::ParamHelper>(params_interfaces, this->getName());
    node_interfaces_ = ::std::make_unique<::cras::FilterNodeInterfaces>(
      this->getName(), this->params_interface_, this->logging_interface_);
    return true;
  }

  ::rclcpp::Logger get_logger() const {
    return this->logging_interface_->get_logger();
  }

  ::rclcpp::Clock::SharedPtr get_clock() const {
    return node_interfaces_->get_node_clock_interface()->get_clock();
  }

  ::std::string getFullParamName(const std::string& name) const {
    return this->param_prefix_ + name;
  }

  /**
   * \brief Tell whether a parameter has been specified.
   *
   * \param[in] name Name of the parameter.
   * \param[in] allow_undeclared Whether to search also undeclared parameters.
   * \return Whether the parameter is specified.
   */
  bool hasParam(const std::string& name, const bool allow_undeclared = true) const {
    const auto param_name = getFullParamName(name);
    return params_->hasParam(param_name, allow_undeclared);
  }

  const ::rclcpp::ParameterValue& declareParameter(
      const ::std::string& name, const ::rclcpp::ParameterValue& default_value,
      const ::rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
      ::rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false) {
    const auto param_name = getFullParamName(name);
    return this->params_interface_->declare_parameter(param_name, default_value, parameter_descriptor, ignore_override);
  }

  template<typename T>
  const ::rclcpp::ParameterValue& declareParameter(
      const ::std::string& name, const T& default_value,
      const ::rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
      ::rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false) {
    const auto param_name = getFullParamName(name);
    return this->params_interface_->declare_parameter(
      param_name, ::rclcpp::ParameterValue(default_value), parameter_descriptor, ignore_override);
  }

  const ::rclcpp::ParameterValue& declareParameter(
      const ::std::string& name, ::rclcpp::ParameterType type,
      const ::rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
      ::rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false) {
    const auto param_name = getFullParamName(name);
    return this->params_interface_->declare_parameter(param_name, type, parameter_descriptor, ignore_override);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   *
   * \details Overloads defining conversion to various types can be defined in several forms. You can either overload
   *          cras::convert() that converts the ParameterValue to an intermediate value, or you can make the
   *          intermediate value autoconvertible to the result type, or you can create a specialization of
   *          DefaultToResultFn and DefaultParamValueType, or you can overload getParamVerbose() itself.
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
   *                    using options.to_result function (which defaults to static_cast).
   * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted.
   *                        The conversion is done using options.to_param function (which defaults to cras::convert).
   *                        Most overloads of cras::convert are in param_convert.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] default_value The default value to use. If std::nullopt, then the parameter is required.
   *                          If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
      ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
  ::cras::GetParamResult<ResultType> getParamVerbose(
      const ::std::string& name,
      const ::std::optional<ResultType>& default_value = ResultType(),
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    const auto param_name = getFullParamName(name);
    return params_->getParamVerbose(param_name, default_value, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
   *                    using options.to_result function (which defaults to static_cast).
   * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
   *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
   *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
      ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
  ::cras::GetParamResult<ResultType> getParamVerbose(
      const ::std::string& name,
      const ResultType& default_value = ResultType(),
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    const auto param_name = getFullParamName(name);
    return params_->getParamVerbose(param_name, ::std::optional<ResultType>(default_value), unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   *
   * \details Overloads defining conversion to various types can be defined in several forms. You can either overload
   *          cras::convert() that converts the ParameterValue to an intermediate value, or you can make the intermediate
   *          value autoconvertible to the result type, or you can create a specialization of DefaultToResultFn and
   *          DefaultParamValueType, or you can overload getParamVerbose() itself.
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
   *                    using options.to_result function (which defaults to static_cast).
   * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
   *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
   *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] default_value The default value to use. If std::nullopt, then the parameter is required.
   *                          If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
      ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
  ResultType getParam(
      const ::std::string& name,
      const ::std::optional<ResultType>& default_value = ResultType(),
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    const auto param_name = getFullParamName(name);
    return params_->getParamVerbose(param_name, default_value, unit, options).value;
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
   *                    using options.to_result function (which defaults to static_cast).
   * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
   *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
   *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
      ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
  ResultType getParam(
      const ::std::string& name,
      const ResultType& default_value = ResultType(),
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    const auto param_name = getFullParamName(name);
    return params_->getParamVerbose(param_name, default_value, unit, options).value;
  }

  ::cras::GetParamResult<::std::string> getParamVerbose(
      const ::std::string& name,
      const ::std::optional<const char*>& default_value = "",
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<::std::string>& options = {}) {
    ::std::optional<::std::string> new_default;
    if (default_value.has_value()) {
      new_default = *default_value;
    }
    return getParamVerbose(name, new_default, unit, options);
  }

  ::cras::GetParamResult<::std::string> getParamVerbose(
      const ::std::string& name,
      const char* default_value = "",
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<::std::string>& options = {}) {
    return getParamVerbose(name, ::std::string(default_value), unit, options);
  }

  ::std::string getParam(
      const ::std::string& name,
      const ::std::optional<const char*>& default_value = "",
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<::std::string>& options = {}) {
    ::std::optional<::std::string> new_default;
    if (default_value.has_value()) {
      new_default = *default_value;
    }
    return getParam(name, new_default, unit, options);
  }

  ::std::string getParam(
      const ::std::string& name,
      const char* default_value = "",
      const ::std::string& unit = "",
      const ::cras::GetParamOptions<::std::string>& options = {}) {
    return getParam(name, ::std::string(default_value), unit, options);
  }

  ::std::unique_ptr<::cras::ParamHelper> params_;
  ::std::unique_ptr<::cras::FilterNodeInterfaces> node_interfaces_;
};

}  // namespace cras
