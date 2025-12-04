#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of cras::to_string() for ROS types and messages. Parsing of dates.
 * \author Martin Pecka
 */

#include <sstream>
#include <string>
#include <type_traits>

#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

::std::string to_string(const ::rclcpp::Time& value);

template<typename T, typename ::std::enable_if_t<::cras::TimeType<T>::value>* = nullptr>
inline ::std::string to_string(const T& value)
{
  return to_string(::cras::convertTime<::rclcpp::Time>(value));
}

::std::string to_string(const ::rclcpp::Duration& value);

template<typename T, typename ::std::enable_if_t<::cras::DurationType<T>::value>* = nullptr>
inline ::std::string to_string(const T& value)
{
  return to_string(::cras::convertDuration<::rclcpp::Duration>(value));
}

/**
 * \brief Convert the given time to a human-readable date-time representation according to ISO 8601.
 * \tparam T Time type
 * \param value The time to convert.
 * \return Human-readable date-time representation.
 */
template<typename T, typename ::std::enable_if_t<::cras::TimeType<T>::value>* = nullptr>
::std::string to_pretty_string(const T& value) = delete;

template<>
::std::string to_pretty_string(const ::rclcpp::Time& value);

template<typename T, typename ::std::enable_if_t<
    ::std::is_same<T, ::rclcpp::Rate>::value ||
    ::std::is_same<T, ::rclcpp::WallRate>::value
  >* = nullptr>
inline ::std::string to_string(const T& value)
{
  ::std::stringstream ss;
  ss << ::cras::frequency(value, true);
  return ss.str();
}

template<typename M, ::std::enable_if_t<::rosidl_generator_traits::is_message<M>::value>* = nullptr>
inline std::string to_string(const M& msg)
{
  ::std::string s = to_yaml(msg);
  if (!s.empty() && s[s.length() - 1] == '\n')
    s.erase(s.length() - 1);
  ::cras::replace(s, "\n", ", ");
  return s;
}

/**
 * \brief Parse timezone offset from the given string.
 *
 * This function accepts this format:
 * <pre>
 * {|Z|[{+|-}HH[:]MM]}
 * </pre>
 *
 * \note This function does not support textual timezones like CET.
 *
 * \param[in] s The string to parse.
 * \return The duration corresponding to the timezone offset.
 */
::rclcpp::Duration parseTimezoneOffset(const ::std::string& s);

/**
 * \brief Parse the given string as time.
 *
 * This function accepts this general format:
 * <pre>
 * [[YY]YY:MM:DD ]HH:MM:SS[.m][{Z|{+|-}HH[:]MM}]
 * </pre>
 *
 * However, many modifications of the format are supported:
 * - The delimiter can be any of `:-/_` (or empty string) and it doesn't need to be consistent in the string.
 * - The space delimiting date and time can be actually any of ` Tt_-`.
 * - The date part can be omitted. In that case, the date will be taken from `referenceTime`.
 * - If year is given just as two digits, `20YY` is assumed.
 * - If time zone offset is not specified in the string, it will be taken from `timezoneOffset`. But if the string
 *   contains a TZ offset, it will be used instead the one passed as argument. If neither is specified, UTC is assumed.
 * - If nonempty delimiters are used, the fields do not need to be zero-padded and can overflow their natural limit up
 *   to UINT16_MAX.
 * - A special value `now` is recognized and always returns current time. It can be any case (`NOW`, `Now` etc.).
 * - Decimal comma `,` can be used instead of decimal dot `.` .
 *
 * \param[in] s The string to parse.
 * \param[in] timezoneOffset Optional timezone offset to use if no offset is specified in the string.
 * \param[in] referenceDate If the date part is missing in the string, the date from this argument will be used.
 * \param[in] clock The clock to be used in case "now" is passed. Also, the type of the clock defines the type of the
 *                  result (defaults to RCL_SYSTEM_TIME if this clock is nullptr). If the clock is null, current system
 *                  clock will be used.
 * \return The parsed time as seconds from epoch.
 * \throws std::invalid_argument If the string does not represent a date with time.
 */
::rclcpp::Time parseTime(const ::std::string& s, const ::std::optional<::rclcpp::Duration>& timezoneOffset = {},
  const ::rclcpp::Time& referenceDate = ::rclcpp::Time(), const ::rclcpp::Clock::ConstSharedPtr& clock = nullptr);

/**
 * \brief Parse the given string as duration.
 *
 * This function accepts this general format:
 * <pre>
 * [{-|+}][HH:]MM:SS[.m]
 * </pre>
 *
 * However, many modifications of the format are supported:
 * - The delimiter can be any of `:-/_` (or empty string) and it doesn't need to be consistent in the string.
 * - The fields do not need to be zero-padded and can overflow their natural limit up to UINT32_MAX.
 * - If no delimiters are present (except maybe the decimal dot), the number is treated as the number of seconds.
 * - Decimal comma `,` can be used instead of decimal dot `.` .
 *
 * \param[in] s The string to parse.
 * \return The parsed duration.
 * \throws std::invalid_argument If the string does not represent a duration.
 */
::rclcpp::Duration parseDuration(const ::std::string& s);

}
