#pragma once

/**
 * \file
 * \brief This file adds utility functions to be used within nodes. So far it only makes
 *        ParamHelper templated getParam() functions accessible via static global calls cras::getParam(nh, ...).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <ros/ros.h>

#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>

namespace cras
{

/** Static variable that serves as the singleton for getting node parameters. */
const ParamHelper paramHelper(::std::make_shared<::cras::NodeLogHelper>());

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
template<typename ResultType, typename ParamServerType = ResultType,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline GetParamResult<ResultType> getParamVerbose(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const ::cras::optional<ResultType>& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParamVerbose(param, name, defaultValue, unit, options);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
template<typename ResultType, typename ParamServerType = ResultType,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline GetParamResult<ResultType> getParamVerbose(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const ResultType& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParamVerbose(param, name, defaultValue, unit, options);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return The loaded parameter value.
 */
template<typename ResultType, typename ParamServerType = ResultType,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline ResultType getParam(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const ::cras::optional<ResultType>& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParam(param, name, defaultValue, unit, options);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
 *                    using options.toResult function (which defaults to static_cast).
 * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
 *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
 *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return The loaded parameter value.
 */
template<typename ResultType, typename ParamServerType = ResultType,
  ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
inline ResultType getParam(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const ResultType& defaultValue = ResultType(),
  const ::std::string& unit = "",
  const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParam(param, name, defaultValue, unit, options);
}

// std::string - char interop specializations

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
 *                         If a required param is not found, a GetParamException is thrown.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
inline GetParamResult<::std::string> getParamVerbose(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const ::cras::optional<const char*>& defaultValue, const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParamVerbose(param, name, defaultValue, unit, options);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return A wrapper containing the loaded parameter value and details about the function execution.
 */
inline GetParamResult<::std::string> getParamVerbose(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const char* defaultValue, const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParamVerbose(param, name, defaultValue, unit, options);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] node The node handle from which parameters are read.
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
  const ::ros::NodeHandle& node, const ::std::string& name,
  const ::cras::optional<const char*>& defaultValue, const ::std::string& unit = "",
  const ::cras::GetParamOptions<::std::string>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParam(param, name, defaultValue, unit, options);
}

/**
 * \brief Get the value of the given ROS parameter, falling back to the specified default value,
 *        and print out a ROS log message with the loaded values (if specified).
 * \details This is a variant allowing use of C-string instead of std::string. 
 * \param[in] node The node handle from which parameters are read.
 * \param[in] name Name of the parameter.
 * \param[in] defaultValue The default value to use.
 * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
 *                 more informative.
 * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
 *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
 * \return The loaded parameter value.
 */
inline ::std::string getParam(
  const ::ros::NodeHandle& node, const ::std::string& name,
  const char* defaultValue, const ::std::string& unit = "",
  const ::cras::GetParamOptions<std::string>& options = {})
{
  const auto param = ::cras::NodeHandleGetParamAdapter(node);
  return paramHelper.getParam(param, name, defaultValue, unit, options);
}

/**
 * \brief Returns a param helper structure "bound" to the given node handle, so that it is not needed to specify
 *        the node handle in the subsequent getParam() calls.
 * \param[in] node The node to bind to.
 * \param[in] ns If nonempty, returns just the parameters in the given namespace.
 * \return The bound param helper.
 */
inline ::cras::BoundParamHelperPtr nodeParams(::ros::NodeHandle& node, const ::std::string& ns = "")
{
  const auto log = ::std::make_shared<NodeLogHelper>();
  const auto param = ::std::make_shared<NodeHandleGetParamAdapter>(node);
  auto result = ::std::make_shared<BoundParamHelper>(log, param);
  if (!ns.empty())
    result = result->paramsInNamespace(ns);
  return result;
}

/**
 * \brief Returns a param helper structure "bound" to the given node handle, so that it is not needed to specify
 *        the node handle in the subsequent getParam() calls.
 * \param[in] node The node to bind to.
 * \return The bound param helper.
 */
inline ::cras::BoundParamHelperPtr paramsForNodeHandle(::ros::NodeHandle& node)
{
  return ::cras::nodeParams(node);
}

}
