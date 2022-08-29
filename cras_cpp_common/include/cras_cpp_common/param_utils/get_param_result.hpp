#pragma once

/**
 * \file
 * \brief Wrapper of getParam() call results.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <type_traits>

#include <ros/console_backend.h>

namespace cras
{

/**
 * \brief Detailed information about the executed getParam() call.
 */
struct GetParamResultInfo
{
  //! \brief Whether the default value has been used.
  bool defaultUsed {false};

  //! \brief Whether a value conversion failed.
  bool convertFailed {false};

  //! \brief Whether a required parameter was found missing or could not be read.
  bool requiredMissing {false};

  //! \brief The log message (returned even if option printMessages is false).
  ::std::string message{};

  //! \brief Severity of the log message.
  ::ros::console::Level messageLevel {::ros::console::Level::Count};
};

/**
 * \brief Wrapper for the result of a getParam() call. It is designed to autoconvert to the result type sometimes.
 * \tparam T Type of the result.
 */
template <typename T>
struct GetParamResult
{
  /**
   * \brief Construct the result from a value and info.
   * \param[in] value The returned value.
   * \param[in] info Details about getParam() execution.
   */
  GetParamResult(const T& value, const ::cras::GetParamResultInfo& info) : value(value), info(info) {}

  /**
   * \brief Autoconvert to the result value (works only sometimes).
   * \return The result value.
   */
  operator T() const  // NOLINT(google-explicit-constructor)
  {
    return this->value;
  }

  /**
   * \brief Autoconvert C-string results to std::string.
   * \return The value as std::string.
   */
  template <typename = ::std::enable_if<::std::is_same<T, char*>::value>>
  operator ::std::string() const  // NOLINT(google-explicit-constructor)
  {
    return {this->value};
  }

  /**
   * \brief Autoconvert char* results to std::string.
   * \tparam I Length of the C-string.
   * \return The value as std::string.
   */
  template <int I, typename = ::std::enable_if<::std::is_same<T, char[I]>::value>>
  operator ::std::string() const  // NOLINT(google-explicit-constructor)
  {
    return {this->value};
  }

  //! \brief The returned value.
  T value;

  //! \brief Details about getParam() execution.
  ::cras::GetParamResultInfo info {};
};

}
