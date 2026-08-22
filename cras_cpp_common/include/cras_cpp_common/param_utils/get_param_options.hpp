#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Options for getParam() calls.
 * \author Martin Pecka
 */

#include <functional>
#include <list>
#include <string>

#include <cras_cpp_common/param_utils/param_convert.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <rclcpp/parameter_value.hpp>

namespace cras {

/**
 * \brief Default function for converting rclcpp::ParameterValue types to result types.
 * \tparam ResultType Type of getParam() result.
 * \tparam ParamValueType Type of the intermediate value to which the ParameterValue is converted.
 * \note Create a specialization of this struct in case you need a different implementation of the conversion.
 */
template<typename ResultType, typename ParamValueType>
struct DefaultToResultFn {
  /**
   * \brief Function converting ParamServerType values to ResultType.
   * \param[in] value The value to convert.
   * \return The converted value.
   * \throws std::runtime_error If the conversion failed.
   */
  static ResultType to_result(const ParamValueType& value) {
    return static_cast<ResultType>(value);
  }
};

/**
 * \brief Default function for converting rclcpp::ParameterValue to an intermediate value of type ParamValueType.
 * \tparam ParamValueType Type of the intermediate value to which the ParameterValue param is converted.
 * \note Create a specialization of this struct in case you need a different implementation of the conversion or if you
 *       add an overload of ::cras::convert() that can be included after this file (and declare the specialization
 *       after the declaration of your overload).
 */
template<typename ParamValueType>
struct DefaultToParamFn {
  /**
   * \brief Function converting ParameterValue to an intermediate value of type ParamValueType.
   * \param[in] x The ParameterValue read from parameters interface.
   * \param[out] v The converted value. It is not valid if this function returns false.
   * \param[in] skip_non_convertible If true and the target value is a container, all non-convertible items will be
   *                               skipped. If false, non-convertible values mean failure of the whole conversion.
   *                               If all items of a container are skipped (and there were some),
   *                               the conversion fails anyway.
   * \param[out] errors If non-null, any error messages coming from the conversion can be appended to this list.
   * \return Whether the conversion succeeded.
   * \note This function should not throw.
   */
  static bool to_param(const ::rclcpp::ParameterValue& x, ParamValueType& v, bool skip_non_convertible = false,
                       ::std::list<::std::string>* errors = nullptr) {
    return ::cras::convert(x, v, skip_non_convertible, errors);
  }
};

/**
 * \brief Default function for converting values to string in getParam(Verbose) functions. Uses ::cras::to_string().
 * \tparam T Type of the value to convert to string.
 * \note Create a specialization of this struct in case you need a different implementation of the conversion or if you
 *       add an overload of ::cras::to_string() that can be included after this file (and declare the specialization
 *       after the declaration of your overload).
 */
template<typename T>
struct ParamToStringFn {
  /**
   * \brief Convert the given value to a string representation.
   * \tparam T Type of the value.
   * \param[in] value The value to convert.
   * \return The string representation.
   */
  static ::std::string to_string(const T& value) {
    return ::cras::to_string(value);
  }
};

/**
 * \brief Default ParamValueType for the given ResultType.
 * \tparam ResultType Type of the getParam() result.
 * \note Create a specialization of this struct in case you want to use a different intermediate ParamValueType for
 *       reading values of type ResultType.
 */
template<typename ResultType>
struct DefaultParamValueType {
  //! \brief The ParamValueType to be used as intermediate type when reading values of type ResultType.
  typedef ResultType type;
};

/**
 * \brief Options specifying behavior of getParam() calls.
 * \note For easy usage in getParam() calls, use the braced initializer syntax, e.g. `{.printMessages = false}`.
 * \tparam ResultType Type of getParam() result.
 * \tparam ParamValueType Type of the intermediate value to which the ParameterValue should be converted.
 */
template<typename ResultType, typename ParamValueType = typename ::cras::DefaultParamValueType<ResultType>::type>
struct GetParamOptions {
  /**
   * \brief Function converting ParamValueType values to ResultType.
   * \param[in] value The value to convert.
   * \return The converted value.
   * \throws std::runtime_error If the conversion failed.
   */
  typedef ::std::function<ResultType(const ParamValueType& value)> ToResultFn;

  /**
   * \brief Function converting rclcpp::ParameterValue to an intermediate value of type ParamValueType.
   * \param[in] param_value The ParameterValue read from parameter interface.
   * \param[out] value The converted value. It is not valid if this function returns false.
   * \param[in] skipNonConvertible If true and the target value is a container, all non-convertible items will be
   *                               skipped. If false, non-convertible values mean failure of the whole conversion.
   *                               If all items of a container are skipped (and there were some),
   *                               the conversion fails anyway.
   * \param[out] errors If non-null, any error messages coming from the conversion can be appended to this list.
   * \return Whether the conversion succeeded.
   * \note This function should not throw.
   */
  typedef ::std::function<bool(
      const ::rclcpp::ParameterValue& param,
      ParamValueType& value,
      bool skip_non_convertible,
      ::std::list<::std::string>* errors)> ToParamFn;

  //! \brief Whether to print error messages to log.
  bool print_messages {true};

  //! \brief Whether defaulted parameters are reported as warning or info level messages.
  bool print_default_as_warn {false};

  //! \brief Throw GetParamException if any conversion fails. If false, the default value is used instead of a value
  //! that failed to convert. In such case, the log message is of error level.
  bool throw_if_convert_fails {false};

  //! \brief Whether undeclared parameters (provided in overrides) should be accepted.
  bool allow_undeclared {false};

  //! \brief Whether undeclared parameters should be declared.
  bool auto_declare {false};

  //! \brief A function that converts ParamValueType values to string for use in log messages.
  ::cras::ToStringFn<ParamValueType> param_to_str = &::cras::ParamToStringFn<ParamValueType>::to_string;

  //! \brief A function that converts ResultType values to string for use in log messages.
  ::cras::ToStringFn<ResultType> result_to_str = &::cras::ParamToStringFn<ResultType>::to_string;

  //! \brief A function converting ParamValueType values to ResultType.
  ToResultFn to_result = &::cras::DefaultToResultFn<ResultType, ParamValueType>::to_result;

  //! \brief A function converting ParameterValue to an intermediate value of type ParamValueType.
  ToParamFn to_param = &::cras::DefaultToParamFn<ParamValueType>::to_param;

  /**
   * \brief Assign from options of a different type. Only the non-function members are copied!
   * \tparam R1 Other ResultType.
   * \tparam P1 Other ParamValueType.
   * \param[in] other The options to copy from.
   * \return This.
   */
  template<typename R1, typename P1>
  GetParamOptions& operator=(const ::cras::GetParamOptions<R1, P1>& other) {
    print_messages = other.print_messages;
    print_default_as_warn = other.print_default_as_warn;
    throw_if_convert_fails = other.throw_if_convert_fails;
    allow_undeclared = other.allow_undeclared;
    auto_declare = other.auto_declare;
    return *this;
  }

  /**
   * \brief Assign from options of the same type. Function members are also copied.
   * \param[in] other The options to copy from.
   * \return This.
   */
  GetParamOptions& operator=(const ::cras::GetParamOptions<ResultType, ParamValueType>& other) {
    print_messages = other.print_messages;
    print_default_as_warn = other.print_default_as_warn;
    throw_if_convert_fails = other.throw_if_convert_fails;
    allow_undeclared = other.allow_undeclared;
    auto_declare = other.auto_declare;
    param_to_str = other.param_to_str;
    result_to_str = other.result_to_str;
    to_result = other.to_result;
    to_param = other.to_param;
    return *this;
  }

  /**
   * \brief Convert this options object to a similar object with a different ParamValueType.
   * \tparam NewParamValueType The new ParamValueType.
   * \param[in] new_to_result New to_result function.
   * \param[in] new_param_to_str New param_to_str function.
   * \param[in] new_to_param New to_param function.
   * \return The new options object.
   */
  template<typename NewParamValueType>
  ::cras::GetParamOptions<ResultType, NewParamValueType> asType(
    typename ::cras::GetParamOptions<ResultType, NewParamValueType>::ToResultFn new_to_result =
    & ::cras::DefaultToResultFn<ResultType, NewParamValueType>::to_result,
    ::cras::ToStringFn<NewParamValueType> new_param_to_str =
    [] (const NewParamValueType& s) { return ::cras::to_string(s); },
    typename ::cras::GetParamOptions<ResultType, NewParamValueType>::ToParamFn new_to_param =
    & ::cras::DefaultToParamFn<NewParamValueType>::to_param) const {
    // We can't initialize the object with the default values of to_param and to_result because they are ill-formed
    ::cras::GetParamOptions<ResultType, NewParamValueType> options = {
      {}, {}, {}, {}, {}, {}, {}, new_to_result, new_to_param
    };
    options = *this;
    options.result_to_str = result_to_str;
    options.param_to_str = new_param_to_str;
    return options;
  }
};

}  // namespace cras
