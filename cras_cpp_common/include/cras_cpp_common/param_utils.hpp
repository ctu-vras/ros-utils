#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS parameter helpers.
 * \author Martin Pecka
 */

#include <list>
#include <memory>
#include <optional>
#include <string>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/get_param_options.hpp>
#include <cras_cpp_common/param_utils/get_param_result.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "type_utils.hpp"

namespace cras
{

template<typename T>
::std::optional<T> maybeParam(const ::rclcpp::ParameterValue& value)
{
  if (value.get_type() == ::rclcpp::PARAMETER_NOT_SET)
    return ::std::nullopt;

  try
  {
    return value.get<T>();
  }
  catch (const ::rclcpp::ParameterTypeException& e)
  {
    return ::std::nullopt;
  }
}

/**
 * \brief This type is a TrueType if the combination of ResultType and ParamValueType is valid.
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
 *                    using options.to_result function (which defaults to static_cast).
 * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
 *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
 *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
 */
template<typename ResultType, typename ParamValueType>
using check_get_param_types = typename std::enable_if_t<
  // getParam() cannot handle std::optional types
  !::cras::is_optional<ResultType>::value &&
  // C strings are handled via overloads as GetParamOptions is undefined for them
  !::cras::is_c_string<ResultType>::value &&
  !::cras::is_c_string<ParamValueType>::value
>;

/**
 * \brief Exception thrown when conversion of a parameter fails during getParam() if option throwIfConvertFails is true
 *        or when a missing parameter is required.
 */
class GetParamException : public ::std::runtime_error {
public:
  /**
   * \brief Construct the exception.
   * \param info Details about getParam() execution until the failure.
   */
  explicit GetParamException(const ::cras::GetParamResultInfo& info);

  //! \brief Details about getParam() execution.
  ::cras::GetParamResultInfo info;
};

/**
 * \brief Tell whether a parameter has been specified.
 *
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] allow_undeclared Whether to search also undeclared parameters.
 * \return Whether the parameter is specified.
 */
bool hasParam(
  const ::rclcpp::node_interfaces::NodeParametersInterface::ConstSharedPtr& params, const ::std::string& name,
  bool allow_undeclared = true);

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
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use. If std::nullopt, then the parameter is required.
 *                          If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
inline ::cras::GetParamResult<ResultType> getParamVerbose(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const ::std::optional<ResultType>& default_value = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "") {
  ::cras::GetParamResultInfo info;
  ParamValueType value;
  bool should_throw {false};
  bool use_default {false};
  const bool is_required = !default_value.has_value();
  ::std::list<::std::string> errors;

  info.convert_failed = false;
  info.required_missing = false;

  if (options.auto_declare && !::cras::hasParam(params, name, false)) {
    bool declare_type_only {true};
    if (default_value.has_value()) {
      if constexpr (::std::is_constructible_v<::rclcpp::ParameterValue, const ResultType&>) {
        params->declare_parameter(name, ::rclcpp::ParameterValue(*default_value));
        declare_type_only = false;
      }
    }
    if (declare_type_only) {
      params->declare_parameter(name, ::cras::ParameterValueTraits<ParamValueType>::param_type);
    }
  }

  if (::cras::hasParam(params, name, options.allow_undeclared)) {
    ::rclcpp::Parameter parameter;
    ::rclcpp::ParameterValue param;
    if (params->get_parameter(name, parameter)) {
      param = parameter.get_parameter_value();
    } else {
      // hasParam told us that the parameter must be in overrides when it was not declared
      param = params->get_parameter_overrides().at(name);
    }
    // try converting to ParamValueType
    if (!options.to_param(param, value, !options.throw_if_convert_fails, &errors)) {
      // if conversion failed, report appropriate error
      if (::cras::ParameterValueTraits<ParamValueType>::param_type != param.get_type()) {
        info.message = ::cras::format(
          "{}Parameter {} found, but it has wrong ROS type. Expected type {}, got type {} with value {}.",
          ::cras::appendIfNonEmpty(log_prefix, ": "), name, ::cras::ParameterValueTraits<ParamValueType>::string_type,
          ::cras::to_string(param.get_type()), ::cras::to_string(param));
      } else {
        ::std::list<::std::string> unique_errors;
        for (const auto& error : errors) {
          if (std::find(unique_errors.begin(), unique_errors.end(), error) == unique_errors.end()) {
            unique_errors.push_back(error);
          }
        }
        info.message = ::cras::format(
          "{}Parameter {} found with correct ROS type {} and value {}, "
          "but its conversion to type {} has failed due to the following errors: {}.",
          ::cras::appendIfNonEmpty(log_prefix, ": "), name, ::cras::ParameterValueTraits<ParamValueType>::string_type,
          ::cras::to_string(param), ::cras::getTypeName<ParamValueType>(), ::cras::to_string(unique_errors));
      }
      info.message_level = ::rclcpp::Logger::Level::Error;

      if (is_required || options.throw_if_convert_fails) {
        should_throw = true;
      } else {
        use_default = true;
      }

      info.convert_failed = true;
      if (is_required) {
        info.required_missing = true;
      }
    }
  } else {  // param does not exist on param server
    // nested param processing calls return on success, so if we got here, it failed
    info.message = ::cras::format(
      "{}Cannot find value for parameter: {}.", ::cras::appendIfNonEmpty(log_prefix, ": "), name);
    if (!is_required) {
      info.message_level =
        options.print_default_as_warn ? ::rclcpp::Logger::Level::Warn : ::rclcpp::Logger::Level::Info;
      use_default = true;
    } else {
      info.message_level = ::rclcpp::Logger::Level::Error;
      info.required_missing = true;
      should_throw = true;
    }
  }

  ::std::string default_used_message {};
  if (default_value.has_value()) {
    default_used_message = ::cras::format(
      " Assigning default: {}{}.", options.result_to_str(default_value.value()), ::cras::prependIfNonEmpty(unit, " "));
  }

  if (use_default) {
    info.message += default_used_message;
  }

  info.default_used = use_default;

  if (should_throw) {
    if (logger != nullptr && options.print_messages) {
      switch (info.message_level) {
        case rclcpp::Logger::Level::Info:
          RCLCPP_INFO(logger->get_logger(), "%s", info.message.c_str());
        break;
        case rclcpp::Logger::Level::Warn:
          RCLCPP_WARN(logger->get_logger(), "%s", info.message.c_str());
        break;
        case rclcpp::Logger::Level::Error:
          RCLCPP_ERROR(logger->get_logger(), "%s", info.message.c_str());
        break;
        default:
          assert(false), "getParam used unexpected log level";
        break;
      }
    }
    throw ::cras::GetParamException(info);
  }

  // using a pointer allows using ResultType without a no-arg constructor (copy/move constructor is enough)
  ::std::unique_ptr<ResultType> resultValue;
  if (use_default) {
    resultValue = ::std::make_unique<ResultType>(default_value.value());
  } else {
    try {
      resultValue = ::std::make_unique<ResultType>(options.to_result(value));  // try converting to ResultType

      info.message = ::cras::format("{}Found parameter: {}, value: {}{}.",
        ::cras::appendIfNonEmpty(log_prefix, ": "), name, options.result_to_str(*resultValue),
        ::cras::prependIfNonEmpty(unit, " "));
      if (errors.empty()) {
        info.message_level = rclcpp::Logger::Level::Info;
      } else {
        ::std::list<::std::string> unique_errors;
        for (const auto& error : errors) {
          if (std::find(unique_errors.begin(), unique_errors.end(), error) == unique_errors.end()) {
            unique_errors.push_back(error);
          }
        }
        info.message += " Some parts of the value were skipped because of the following conversion errors: " +
          ::cras::to_string(unique_errors);
        info.message_level = ::rclcpp::Logger::Level::Warn;
      }
    } catch (const ::std::runtime_error& e) {  // conversion from ParamValueType to ResultType failed
      info.message = ::cras::format(
        "{}Cannot convert value '{}' of parameter {} to requested type {} (error: {}).",
        ::cras::appendIfNonEmpty(log_prefix, ": "), options.param_to_str(value), name,
        ::cras::getTypeName<ResultType>(), e.what());
      info.message_level = ::rclcpp::Logger::Level::Error;

      if (is_required || options.throw_if_convert_fails) {
        should_throw = true;
      } else {  // use default if provided
        info.message += default_used_message;
        resultValue = ::std::make_unique<ResultType>(default_value.value());
        info.default_used = true;
      }

      info.convert_failed = true;
      if (is_required) {
        info.required_missing = true;
      }
    }
  }

  if (logger != nullptr && options.print_messages) {
    switch (info.message_level) {
      case rclcpp::Logger::Level::Info:
        RCLCPP_INFO(logger->get_logger(), "%s", info.message.c_str());
      break;
      case rclcpp::Logger::Level::Warn:
        RCLCPP_WARN(logger->get_logger(), "%s", info.message.c_str());
      break;
      case rclcpp::Logger::Level::Error:
        RCLCPP_ERROR(logger->get_logger(), "%s", info.message.c_str());
      break;
      default:
        assert(false), "getParam used unexpected log level";
      break;
    }
  }

  if (should_throw) {
    throw ::cras::GetParamException(info);
  }

  return {*resultValue, info};
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
 *                    using options.to_result function (which defaults to static_cast).
 * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
 *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
 *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
inline ::cras::GetParamResult<ResultType> getParamVerbose(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const ResultType& default_value = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "") {
  return ::cras::getParamVerbose(
    params, name, ::std::optional<ResultType>(default_value), unit, options, logger, log_prefix);
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
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use. If std::nullopt, then the parameter is required.
 *                          If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return The loaded parameter value.
 */
template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
inline ResultType getParam(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const ::std::optional<ResultType>& default_value = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "") {
  return ::cras::getParamVerbose(params, name, default_value, unit, options, logger, log_prefix).value;
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
 *                    using options.to_result function (which defaults to static_cast).
 * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
 *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
 *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return The loaded parameter value.
 */
template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamValueType>* = nullptr>
inline ResultType getParam(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const ResultType& default_value = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "") {
  return ::cras::getParamVerbose(params, name, default_value, unit, options, logger, log_prefix).value;
}

// std::string - char interop specializations

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string.
 * \details Overloads defining conversion to various types can be defined in several forms. You can either overload
 *          cras::convert() that converts the ParameterValue to an intermediate value, or you can make the intermediate
 *          value autoconvertible to the result type, or you can create a specialization of DefaultToResultFn and
 *          DefaultParamValueType, or you can overload getParamVerbose() itself.
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamValueType
 *                    using options.to_result function (which defaults to static_cast).
 * \tparam ParamValueType Intermediate type to which the ParameterValue read from parameter interface is converted. The
 *                        conversion is done using options.to_param function (which defaults to cras::convert). Most
 *                        overloads of cras::convert are in param_convert.hpp, but you can add your own.
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use. If std::nullopt, then the parameter is required.
 *                          If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
::cras::GetParamResult<::std::string> getParamVerbose(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const ::std::optional<const char*>& default_value = "",
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "");

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string.
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
::cras::GetParamResult<::std::string> getParamVerbose(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const char* default_value = "",
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "");

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 *
 * \details This is a variant allowing use of C-string instead of std::string.
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use. If std::nullopt, then the parameter is required.
 *                          If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return The loaded parameter value.
 */
::std::string getParam(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const ::std::optional<const char*>& default_value = "",
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "");

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string.
 * \param[in] params The parameter interface from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] default_value The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throw_if_convert_fails = true, .allow_undeclared = false}`.
 * \param[in] logger The log interface used for printing messages. If nullptr, no messages are printed.
 * \param[in] log_prefix The prefix to use in logs.
 * \return The loaded parameter value.
 */
::std::string getParam(
  const ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const ::std::string& name,
  const char* default_value = "",
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {},
  const ::rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logger = nullptr,
  const ::std::string& log_prefix = "");

/**
 * \brief Generate definitions of "specializations" of getParam(Verbose) that use different
 *        ResultType and ParamValueType. They will be automatically used when the user requests a parameter
 *        of type ResultType. If ::cras::to_string() cannot convert ResultType to string, you have to declare
 *        overload ::cras::to_string(const ResultType&) prior to calling this macro.
 * \param resultType Type of the result values.
 * \param ParamValueType Type of the intermediate values to which XmlRpcValues are converted.
 * \param defaultUnit The unit to use if the users doesn't pass any.
 * \param convertToResultFn The ToResultFn to use for parameter conversion.
 * \note This macro has to be called in the `::cras` namespace.
 * \note If ::cras::convert() cannot convert XmlRpcValue to ParamValueType, you have to define specialization
 *       ::cras::DefaultToParamFn<ParamValueType> which implements the toParam() conversion function.
 */
#define DEFINE_CONVERTING_GET_PARAM(resultType, ParamValueType, defaultUnit, convertToResultFn) \
template<> \
struct ParamToStringFn<resultType> \
{ \
  static ::std::string to_string(const resultType& v){ return ::cras::to_string(v); } \
}; \
\
template<> \
struct DefaultToResultFn<resultType, ParamValueType> \
{ \
  static resultType to_result(const ParamValueType& v){ return convertToResultFn(v); } \
}; \
\
template<> \
struct DefaultParamValueType<resultType> \
{ \
  typedef ParamValueType type; \
};

/**
 * \brief Generate definitions of "specializations" of getParam(Verbose) that use different
 *        ResultType and ParamValueType. They will be automatically used when the user requests a parameter
 *        of type ResultType. ParamValueType is converted to resultType via resultType one-arg constructor.
 * \param resultType Type of the result values.
 * \param ParamValueType Type of the intermediate values to which XmlRpcValues are converted.
 * \param defaultUnit The unit to use if the users doesn't pass any.
 * \note This macro has to be called in the `::cras` namespace.
 */
#define DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(resultType, ParamValueType, defaultUnit) \
DEFINE_CONVERTING_GET_PARAM(resultType, ParamValueType, defaultUnit, \
  ([](const ParamValueType& v) { return resultType(v); }))

/**
 * \brief Generate definitions of "specializations" of getParam(Verbose) that use different
 *        ResultType and ParamValueType. They will be automatically used when the user requests a parameter
 *        of type ResultType. ParamValueType is converted to resultType via static_cast.
 * \param resultType Type of the result values.
 * \param ParamValueType Type of the intermediate values to which XmlRpcValues are converted.
 * \param defaultUnit The unit to use if the users doesn't pass any.
 * \note This macro has to be called in the `::cras` namespace.
 */
#define DEFINE_CONVERTING_GET_PARAM_WITH_CAST(resultType, ParamValueType, defaultUnit) \
DEFINE_CONVERTING_GET_PARAM(resultType, ParamValueType, defaultUnit, \
  ([](const ParamValueType& v) { return static_cast<resultType>(v); }))
}

#if __has_include(<rclcpp/duration.hpp>)
#include "param_utils/get_param_specializations/rclcpp.hpp"
#endif

#if __has_include(<tf2/LinearMath/Vector3.h>)
#include "param_utils/get_param_specializations/tf2.hpp"

#if __has_include(<geometry_msgs/msg/vector3.hpp>)
#include "param_utils/get_param_specializations/geometry_msgs.hpp"
#endif

#endif

#if __has_include(<Eigen/Core>)
#include "param_utils/get_param_specializations/eigen.hpp"
#endif

namespace cras {

class ParamHelper {
public:
  using RequiredInterfaces = ::rclcpp::node_interfaces::NodeInterfaces<
    ::rclcpp::node_interfaces::NodeLoggingInterface,
    ::rclcpp::node_interfaces::NodeParametersInterface
  >;

  explicit ParamHelper(const RequiredInterfaces& node_interfaces, const ::std::string& log_prefix = "");

  /**
   * \brief Tell whether a parameter has been specified.
   *
   * \param[in] name Name of the parameter.
   * \param[in] allow_undeclared Whether to search also undeclared parameters.
   * \return Whether the parameter is specified.
   */
  bool hasParam(const ::std::string& name, bool allow_undeclared = true) const;

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
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::std::string& name,
    const ::std::optional<ResultType>& default_value = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    return ::cras::getParamVerbose(
      node_interfaces_.get_node_parameters_interface(), name, default_value, unit, options,
      node_interfaces_.get_node_logging_interface(), log_prefix_);
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
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::std::string& name,
    const ResultType& default_value = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    return ::cras::getParamVerbose(
      node_interfaces_.get_node_parameters_interface(), name, ::std::optional<ResultType>(default_value), unit, options,
      node_interfaces_.get_node_logging_interface(), log_prefix_);
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
  inline ResultType getParam(
    const ::std::string& name,
    const ::std::optional<ResultType>& default_value = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    return ::cras::getParamVerbose(
      node_interfaces_.get_node_parameters_interface(), name, default_value, unit, options,
      node_interfaces_.get_node_logging_interface(), log_prefix_).value;
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
  inline ResultType getParam(
    const ::std::string& name,
    const ResultType& default_value = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamValueType>& options = {}) {
    return ::cras::getParamVerbose(
      node_interfaces_.get_node_parameters_interface(), name, default_value, unit, options,
      node_interfaces_.get_node_logging_interface(), log_prefix_).value;
  }

  // const char* overloads of getParam

  ::cras::GetParamResult<::std::string> getParamVerbose(
    const ::std::string& name,
    const ::std::optional<const char*>& default_value = "",
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {});

  ::cras::GetParamResult<::std::string> getParamVerbose(
    const ::std::string& name,
    const char* default_value = "",
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {});

  ::std::string getParam(
    const ::std::string& name,
    const ::std::optional<const char*>& default_value = "",
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {});

  ::std::string getParam(
    const ::std::string& name,
    const char* default_value = "",
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {});

  ::std::string log_prefix_;  //!< Name used as a prefix in log messages.
  RequiredInterfaces node_interfaces_;
};

}
