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

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <ros/message_traits.h>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

template<typename T, typename ::std::enable_if_t<
    ::std::is_same<T, ::ros::Time>::value ||
    ::std::is_same<T, ::ros::WallTime>::value ||
    ::std::is_same<T, ::ros::SteadyTime>::value ||
    ::std::is_same<T, ::ros::Duration>::value ||
    ::std::is_same<T, ::ros::WallDuration>::value
  >* = nullptr>
inline ::std::string to_string(const T& value)
{
  ::std::stringstream ss;
  ss << value;
  return ss.str();
}

template<typename T, typename ::std::enable_if_t<
    ::std::is_same<T, ::ros::Rate>::value ||
    ::std::is_same<T, ::ros::WallRate>::value
  >* = nullptr>
inline ::std::string to_string(const T& value)
{
  ::std::stringstream ss;
  ss << ::cras::frequency(value, true);
  return ss.str();
}

template<typename M, ::std::enable_if_t<::ros::message_traits::IsMessage<M>::value>* = nullptr>
inline std::string to_string(const M& msg)
{
  ::std::stringstream ss;
  ss << msg;
  ::std::string s = ss.str();
  if (!s.empty() && s[s.length() - 1] == '\n')
    s.erase(s.length()-1);
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
 * \tparam D Duration type.
 * \param[in] s The string to parse.
 * \return The duration corresponding to the timezone offset.
 */
template<typename D = ::ros::Duration, typename ::std::enable_if_t<
    ::std::is_same<D, ::ros::Duration>::value ||
    ::std::is_same<D, ::ros::WallDuration>::value
  >* = nullptr>
D parseTimezoneOffset(const ::std::string& s);

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
 * \return The parsed time as seconds from epoch.
 * \throws std::invalid_argument If the string does not represent a date with time.
 */
template<typename T = ::ros::Time, typename ::std::enable_if_t<
    ::std::is_same<T, ::ros::Time>::value ||
    ::std::is_same<T, ::ros::WallTime>::value ||
    ::std::is_same<T, ::ros::SteadyTime>::value
  >* = nullptr>
T parseTime(const ::std::string& s,
  const ::cras::optional<typename ::cras::DurationType<T>::value>& timezoneOffset = {}, const T& referenceDate = {});

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
template<typename D = ::ros::Duration, typename ::std::enable_if_t<
    ::std::is_same<D, ::ros::Duration>::value ||
    ::std::is_same<D, ::ros::WallDuration>::value
  >* = nullptr>
D parseDuration(const ::std::string& s);

}
