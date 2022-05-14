#pragma once

/**
 * \file
 * \brief This file provides helper methods easing access to parameters passed to nodes, nodelets and filters.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <list>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <cras_cpp_common/param_utils/get_param_adapter.hpp>
#include <cras_cpp_common/param_utils/get_param_options.hpp>
#include <cras_cpp_common/param_utils/get_param_result.hpp>

namespace cras
{

/**
 * \brief Exception thrown when conversion of a parameter fails during getParam() if option throwIfConvertFails is true
 *        or when a missing parameter is required.
 */
class GetParamException : public ::std::runtime_error
{
public:
  /**
   * \brief Construct the exception.
   * \param info Details about getParam() execution until the failure.
   */
  explicit GetParamException(const ::cras::GetParamResultInfo& info) : ::std::runtime_error(info.message), info(info) {}
  
  //! \brief Details about getParam() execution.
  ::cras::GetParamResultInfo info;
};

/**
 * \brief This type is a TrueType if the combination of ResultType and ParamServerType is valid.
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 */
template <typename ResultType, typename ParamServerType>
using check_get_param_types = typename std::enable_if_t<
  // getParam() cannot handle cras::optional types
  !::cras::is_optional<ResultType>::value &&
  // C strings are handled via overloads as GetParamOptions is undefined for them
  !::cras::is_c_string<ResultType>::value &&
  !::cras::is_c_string<ParamServerType>::value
>;

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 *
 * \details Overloads defining conversion to various types can be defined in several forms. You can either overload
 *          cras::convert() that converts the XmlRpcValue to an intermediate value, or you can make the intermediate
 *          value autoconvertible to the result type, or you can create a specialization of DefaultToResultFn and
 *          DefaultParamServerType, or you can overload getParamVerbose() itself.
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline ::cras::GetParamResult<ResultType> getParamVerbose(
  const ::cras::GetParamAdapter& param, const ::std::string& name,
  const ::cras::optional<ResultType>& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {},
  const ::cras::LogHelper* const logger = nullptr)
{
  ::cras::GetParamResultInfo info;
  ParamServerType value;
  bool shouldThrow {false};
  bool useDefault {false};
  const bool isRequired = !defaultValue.has_value();
  ::std::list<::std::string> errors;
  
  info.convertFailed = false;
  info.requiredMissing = false;
  const auto origNs = options.origNamespace.empty() ? param.getNamespace() : options.origNamespace;
  const auto origParamName = options.origParamName.empty() ? name : options.origParamName;

  ::XmlRpc::XmlRpcValue xmlValue;
  if (param.getParam(name, xmlValue))  // try getting the parameter as XmlRpcValue
  {
    if (!options.toParam(xmlValue, value, !options.throwIfConvertFails, &errors))  // try converting to ParamServerType
    {
      // if conversion failed, report appropriate error
      if (::cras::XmlRpcValueTraits<ParamServerType>::xmlRpcType != xmlValue.getType())
      {
        info.message = ::cras::format(
          "%s: Parameter %s found, but it has wrong XmlRpc type. Expected type %s, got type %s with value %s.",
          origNs.c_str(), origParamName.c_str(),
          ::cras::XmlRpcValueTraits<ParamServerType>::stringType,
          ::cras::to_cstring(xmlValue.getType()),
          ::cras::to_string(xmlValue).c_str());
      }
      else
      {
        info.message = ::cras::format(
          "%s: Parameter %s found with correct XmlRpc type %s and value %s, "
          "but its conversion to type %s has failed due to the following errors: %s.",
          origNs.c_str(), origParamName.c_str(),
          ::cras::XmlRpcValueTraits<ParamServerType>::stringType,
          ::cras::to_string(xmlValue).c_str(),
          ::cras::getTypeName<ParamServerType>().c_str(),
          ::cras::to_string(errors).c_str());
      }
      info.messageLevel = ::ros::console::Level::Error;
  
      if (isRequired || options.throwIfConvertFails)
        shouldThrow = true;
      else
        useDefault = true;
  
      info.convertFailed = true;
      if (isRequired)
        info.requiredMissing = true;
    }
  }
  else  // param does not exist on param server
  {
    // nested parameters contain a slash and the name of the parameter can be split by the slashes
    // and searched for recursively
    if (options.allowNestedParams && ::cras::contains(name, '/'))
    {
      const auto parts = ::cras::split(name, "/", 1);
      if (parts.size() == 2 && !parts[0].empty() && !parts[1].empty())
      {
        const auto head = parts[0];
        const auto tail = parts[1];
        try
        {
          auto nsParams = param.getNamespaced(head);
          auto nestedOptions = options;
          nestedOptions.origNamespace = origNs;
          nestedOptions.origParamName = origParamName;
          return ::cras::getParamVerbose(*nsParams, tail, defaultValue, unit, nestedOptions, logger);
        }
        catch (const ::std::runtime_error& e)
        {
          // ignore the error, we just couldn't find the nested param
        }
      }
    }
    // nested param processing calls return on success, so if we got here, it failed
    info.message = ::cras::format("%s: Cannot find value for parameter: %s.",
         origNs.c_str(), origParamName.c_str());
    if (!isRequired)
    {
      info.messageLevel = options.printDefaultAsWarn ? ::ros::console::Level::Warn : ::ros::console::Level::Info;
      useDefault = true;
    }
    else
    {
      info.messageLevel = ::ros::console::Level::Error;
      info.requiredMissing = true;
      shouldThrow = true;
    }
  }

  ::std::string defaultUsedMessage {};
  if (defaultValue.has_value())
    defaultUsedMessage = ::cras::format(" Assigning default: %s%s.",
      options.resultToStr(defaultValue.value()).c_str(), ::cras::prependIfNonEmpty(unit, " ").c_str());

  if (useDefault)
    info.message += defaultUsedMessage;

  info.defaultUsed = useDefault;

  if (shouldThrow)
  {
    if (logger != nullptr && options.printMessages)
      logger->print(info.messageLevel, info.message);
    throw GetParamException(info);
  }
  
  // using a pointer allows using ResultType without a no-arg constructor (copy/move constructor is enough)
  ::std::unique_ptr<ResultType> resultValue;
  if (useDefault)
  {
    resultValue = ::std::make_unique<ResultType>(defaultValue.value());
  }
  else
  {
    try
    {
      resultValue = ::std::make_unique<ResultType>(options.toResult(value));  // try converting to ResultType

      info.message = ::cras::format("%s: Found parameter: %s, value: %s%s.",
        origNs.c_str(), origParamName.c_str(), options.resultToStr(*resultValue).c_str(),
        ::cras::prependIfNonEmpty(unit, " ").c_str());
      if (errors.empty())
      {
        info.messageLevel = ros::console::Level::Info;
      }
      else
      {
        info.message += " Some parts of the value were skipped because of the following conversion errors: " +
          ::cras::to_string(errors);
        info.messageLevel = ::ros::console::Level::Warn;
      }
    }
    catch (const std::runtime_error& e)  // conversion from ParamServerType to ResultType failed
    {
      info.message = ::cras::format(
        "%s: Cannot convert value '%s' of parameter %s to requested type %s (error: %s).",
        origNs.c_str(), options.paramToStr(value).c_str(),
        origParamName.c_str(),
        ::cras::getTypeName<ResultType>().c_str(),
        e.what());
      info.messageLevel = ::ros::console::Level::Error;

      if (isRequired || options.throwIfConvertFails)
      {
        shouldThrow = true;
      }
      else  // use default if provided
      {
        info.message += defaultUsedMessage;
        resultValue = ::std::make_unique<ResultType>(defaultValue.value());
        info.defaultUsed = true;
      }

      info.convertFailed = true;
      if (isRequired)
        info.requiredMissing = true;
    }
  }

  if (logger != nullptr && options.printMessages)
    logger->print(info.messageLevel, info.message);

  if (shouldThrow)
    throw GetParamException(info);

  return {*resultValue, info};
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline ::cras::GetParamResult<ResultType> getParamVerbose(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const ResultType& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {},
  const ::cras::LogHelper* const logger = nullptr)
{
  return ::cras::getParamVerbose(param, name, ::cras::optional<ResultType>(defaultValue), unit, options, logger);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return The loaded parameter value.
 */
template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline ResultType getParam(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const ::cras::optional<ResultType>& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {},
  const ::cras::LogHelper* const logger = nullptr)
{
  return ::cras::getParamVerbose(param, name, defaultValue, unit, options, logger).value;
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return The loaded parameter value.
 */
template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline ResultType getParam(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const ResultType& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {},
  const ::cras::LogHelper* const logger = nullptr)
{
  return ::cras::getParamVerbose(param, name, defaultValue, unit, options, logger).value;
}

// std::string - char interop specializations

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
inline ::cras::GetParamResult<::std::string> getParamVerbose(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const ::cras::optional<const char *>& defaultValue, const ::std::string &unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {}, const ::cras::LogHelper* const logger = nullptr)
{
  ::cras::optional<::std::string> defaultStr;
  if (defaultValue.has_value())
    defaultStr = defaultValue.value();
  return ::cras::getParamVerbose(param, name, defaultStr, unit, options, logger);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
inline ::cras::GetParamResult<::std::string> getParamVerbose(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const char* defaultValue, const ::std::string &unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {}, const ::cras::LogHelper* const logger = nullptr)
{
  ::cras::optional<::std::string> defaultStr(defaultValue);
  return ::cras::getParamVerbose(param, name, defaultStr, unit, options, logger);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return The loaded parameter value.
 */
inline ::std::string getParam(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const ::cras::optional<const char *>& defaultValue, const ::std::string &unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {}, const ::cras::LogHelper* const logger = nullptr)
{
  return ::cras::getParamVerbose(param, name, defaultValue, unit, options, logger).value;
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] param The parameter adapter from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \param[in] logger The LogHelper used for printing messages. If nullptr, no messages are printed.
 * \return The loaded parameter value.
 */
inline ::std::string getParam(
  const ::cras::GetParamAdapter& param, const ::std::string &name,
  const char* defaultValue, const ::std::string &unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {}, const ::cras::LogHelper* const logger = nullptr)
{
  return ::cras::getParamVerbose(param, name, defaultValue, unit, options, logger).value;
}

/**
 * \brief Generate definitions of "specializations" of getParam(Verbose) that use different
 *        ResultType and ParamServerType. They will be automatically used when the user requests a parameter
 *        of type ResultType.
 * \param resultType Type of the result values.
 * \param paramServerType Type of the intermediate values to which XmlRpcValues are converted.
 * \param defaultUnit The unit to use if the users doesn't pass any.
 * \param convertToResultFn The ToResultFn to use for parameter conversion.
 */
#define DEFINE_CONVERTING_GET_PARAM(resultType, paramServerType, defaultUnit, convertToResultFn) \
template<>\
struct DefaultToResultFn<resultType, paramServerType>\
{\
	static resultType toResult(const paramServerType& v){ return convertToResultFn(v); };\
};\
\
template<>\
struct DefaultParamServerType<resultType>\
{\
	typedef paramServerType type;\
};

/**
 * \brief Generate definitions of "specializations" of getParam(Verbose) that use different
 *        ResultType and ParamServerType. They will be automatically used when the user requests a parameter
 *        of type ResultType. paramServerType is converted to resultType via resultType one-arg constructor. 
 * \param resultType Type of the result values.
 * \param paramServerType Type of the intermediate values to which XmlRpcValues are converted.
 * \param defaultUnit The unit to use if the users doesn't pass any.
 */
#define DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(resultType, paramServerType, defaultUnit) \
DEFINE_CONVERTING_GET_PARAM(resultType, paramServerType, defaultUnit, \
  ([](const paramServerType& v) { return resultType(v); }))

/**
 * \brief Generate definitions of "specializations" of getParam(Verbose) that use different
 *        ResultType and ParamServerType. They will be automatically used when the user requests a parameter
 *        of type ResultType. paramServerType is converted to resultType via static_cast.
 * \param resultType Type of the result values.
 * \param paramServerType Type of the intermediate values to which XmlRpcValues are converted.
 * \param defaultUnit The unit to use if the users doesn't pass any.
 */
#define DEFINE_CONVERTING_GET_PARAM_WITH_CAST(resultType, paramServerType, defaultUnit) \
DEFINE_CONVERTING_GET_PARAM(resultType, paramServerType, defaultUnit, \
  ([](const paramServerType& v) { return static_cast<resultType>(v); }))
}

#if __has_include(<ros/duration.h>)
#include "param_utils/get_param_specializations/ros.hpp"
#endif

#if __has_include(<geometry_msgs/Vector3.h>)
#include "param_utils/get_param_specializations/geometry_msgs.hpp"
#endif

#if __has_include(<tf2/LinearMath/Vector3.h>)
#include "param_utils/get_param_specializations/tf2.hpp"
#endif

#if __has_include(<Eigen/Core>)
#include "param_utils/get_param_specializations/eigen.hpp"
#endif