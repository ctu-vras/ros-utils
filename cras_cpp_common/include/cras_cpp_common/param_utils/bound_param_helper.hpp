#pragma once

/**
 * \file
 * \brief Bound param helper (allows omitting the param adapter in each getParam call).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils/get_param_adapter.hpp>
#include <cras_cpp_common/param_utils/get_param_options.hpp>
#include <cras_cpp_common/param_utils/get_param_result.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/optional.hpp>

namespace cras
{

class BoundParamHelper;
typedef ::std::shared_ptr<::cras::BoundParamHelper> BoundParamHelperPtr;

/**
 * \brief Bound param helper (allows omitting the param adapter in each getParam call).
 */
class BoundParamHelper : protected ::cras::ParamHelper
{
public:
  /**
   * Create the bound param helper.
   * @param log The log helper to use for logging parameter read messages.
   * @param param The raw parameter adapter to bind to.
   */
  BoundParamHelper(const ::cras::LogHelperPtr& log, const ::cras::GetParamAdapterPtr& param) :
      ::cras::ParamHelper(log), param(param)
  {
  }
  
  ~BoundParamHelper() override = default;

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
   *                         If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamServerType = ResultType>
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::std::string& name, const ::cras::optional<ResultType>& defaultValue = ResultType(),
    const ::std::string& unit = "", const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::ParamHelper::getParamVerbose(*this->param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamServerType = ResultType>
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::std::string& name, const ResultType& defaultValue = ResultType(),
    const ::std::string& unit = "", const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::ParamHelper::getParamVerbose(
      *this->param, name, ::cras::optional<ResultType>(defaultValue), unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
   *                         If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamServerType = ResultType>
  inline ResultType getParam(
    const ::std::string& name, const ::cras::optional<ResultType>& defaultValue = ResultType(),
    const ::std::string& unit = "", const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::ParamHelper::getParam(*this->param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamServerType = ResultType>
  inline ResultType getParam(
    const ::std::string& name, const ResultType& defaultValue = ResultType(),
    const ::std::string& unit = "", const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {}) const
  {
    return ::cras::ParamHelper::getParam(*this->param, name, ::cras::optional<ResultType>(defaultValue), unit, options);
  }

  // std::string - char interop specializations

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
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
    const ::std::string& name, const ::cras::optional<const char *>& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::ParamHelper::getParamVerbose(*this->param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  inline ::cras::GetParamResult<std::string> getParamVerbose(
    const ::std::string& name, const char* const& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::ParamHelper::getParamVerbose(*this->param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
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
    const ::std::string& name, const ::cras::optional<const char *>& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {}) const
  {
    return ::cras::ParamHelper::getParam(*this->param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  inline ::std::string getParam(
    const ::std::string& name, const char* const& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<std::string>& options = {}) const
  {
    return ::cras::ParamHelper::getParam(*this->param, name, defaultValue, unit, options);
  }

  /**
   * \brief Whether a parameter with the given name exists.
   * \param name Name of the parameter.
   * \return Whether the parameter exists.
   */
  inline bool hasParam(const ::std::string& name) const
  {
    return this->param->hasParam(name);
  }

  /**
   * \brief Return a parameter helper of a sub-namespace. 
   * \param ns Namespace.
   * \return The new helper.
   */
  inline ::cras::BoundParamHelperPtr paramsInNamespace(const ::std::string& ns) const
  {
    return ::std::make_shared<::cras::BoundParamHelper>(this->log, this->param->getNamespaced(ns));
  }

protected:
  //! \brief The bound parameter adapter.
  ::cras::GetParamAdapterPtr param;
};

}