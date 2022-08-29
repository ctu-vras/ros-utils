#pragma once

/**
 * \file
 * \brief This class provides a unified experience for nodes, nodelets and filters for getting ROS parameter values.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>
#include <utility>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/get_param_adapter.hpp>
#include <cras_cpp_common/param_utils/get_param_options.hpp>
#include <cras_cpp_common/param_utils/get_param_result.hpp>
#include <cras_cpp_common/optional.hpp>

namespace cras
{

/**
 * \brief This class provides a unified experience for nodes, nodelets and filters for getting ROS parameter values.
 * Each parameter has to be provided a default value, and each parameter read is logged - specified parameters with INFO
 * verbosity level, defaulted parameters with WARN level. There are also lots of template specializations for builtin
 * ROS types or unsigned values which ease the process of reading the parameters correctly.
 */
class ParamHelper
{
public:
  /**
   * Create the param helper using the given log helper for logging messages.
   * @param log The log helper to use for logging.
   */
  explicit ParamHelper(::cras::LogHelperPtr log) : log(::std::move(log)) {}
  virtual ~ParamHelper() = default;

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   *
   * \details Overloads defining conversion to various types can be defined in several forms. You can either overload
   *          cras::convert() that converts the XmlRpcValue to an intermediate value, or you can make the intermediate
   *          value autoconvertible to the result type, or you can overload getParamVerbose() itself.
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
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::cras::GetParamAdapter& param, const ::std::string& name,
    const ::cras::optional<ResultType>& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::getParamVerbose(param, name, defaultValue, unit, options, this->log.get());
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
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::cras::GetParamAdapter& param, const ::std::string& name,
    const ResultType& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::getParamVerbose(param, name, defaultValue, unit, options, this->log.get());
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
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ResultType getParam(
    const ::cras::GetParamAdapter& param, const ::std::string& name,
    const ::cras::optional<ResultType>& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::getParam(param, name, defaultValue, unit, options, this->log.get());
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
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ResultType getParam(
    const ::cras::GetParamAdapter& param, const ::std::string& name,
    const ResultType& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::getParam(param, name, defaultValue, unit, options, this->log.get());
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
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  inline ::cras::GetParamResult<::std::string> getParamVerbose(
    const ::cras::GetParamAdapter& param, const ::std::string& name,
    const ::cras::optional<const char *>& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::getParamVerbose(param, name, defaultValue, unit, options, this->log.get());
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
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  inline ::cras::GetParamResult<::std::string> getParamVerbose(
    const ::cras::GetParamAdapter& param, const ::std::string &name,
    const char* defaultValue, const ::std::string &unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::getParamVerbose(param, name, defaultValue, unit, options, this->log.get());
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
   * \return The loaded parameter value.
   */
  inline ::std::string getParam(
    const ::cras::GetParamAdapter& param, const ::std::string &name,
    const ::cras::optional<const char *>& defaultValue, const ::std::string &unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::getParam(param, name, defaultValue, unit, options, this->log.get());
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
   * \return The loaded parameter value.
   */
  inline ::std::string getParam(
    const ::cras::GetParamAdapter& param, const ::std::string &name,
    const char* defaultValue, const ::std::string &unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::getParam(param, name, defaultValue, unit, options, this->log.get());
  }

  /**
   * \brief Return the log helper used for logging.
   * \return The log helper.
   */
  ::cras::LogHelperPtr getLogger() const
  {
    return this->log;
  }

  /**
   * \brief Set the log helper used for logging.
   * \param[in] logger The new log helper.
   */
  void setLogger(const ::cras::LogHelperPtr& logger)
  {
    this->log = logger;
  }

protected:
  //! \brief The log helper to use for logging parameter read messages.
  ::cras::LogHelperPtr log;
};

typedef ::std::shared_ptr<::cras::ParamHelper> ParamHelperPtr;

}
