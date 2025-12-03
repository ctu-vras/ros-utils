#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Shim implementing std::from_chars support for floating-point number types in GCC < 11.
 * \author Martin Pecka
 */

#include <charconv>
#include <string>

namespace cras
{

/**
 * \brief Convert the given string to its best float representation. The behavior is similar to std::from_chars(),
 *        i.e. no leading space of plus sign is allowed.
 * \param[in] string The string to convert.
 * \param[out] value The parsed value (valid only if parsing succeeded, i.e. `ec` in the result is 0).
 * \param[in] fmt Bitmask of `std::chars_format` values that specify the format rules with which the value should
 *                be interpreted. Not all formats have to be supported.
 * \return Struct containing the error code `ec` - 0 on success, `std::errc::invalid_argument` if no number could be
 *         parsed, and `std::errc::result_out_of_range` if a numeric value was parsed but did not fit into a float.
 *         It is implementation specific, whether `result_out_of_range` or 0/inf will be returned for too small/large
 *         values. `ptr` points to the first character that was not parsed as a part of the numeric value.
 */
inline ::std::from_chars_result from_chars(
  const ::std::string& string, float& value,
  const ::std::chars_format fmt = ::std::chars_format::general) noexcept
{
  return ::std::from_chars(string.data(), string.data() + string.size(), value, fmt);
}

/**
 * \brief Convert the given string to its best double representation. The behavior is similar to std::from_chars(),
 *        i.e. no leading space of plus sign is allowed.
 * \param[in] string The string to convert.
 * \param[out] value The parsed value (valid only if parsing succeeded, i.e. `ec` in the result is 0).
 * \param[in] fmt Bitmask of `std::chars_format` values that specify the format rules with which the value should
 *                be interpreted. Not all formats have to be supported.
 * \return Struct containing the error code `ec` - 0 on success, `std::errc::invalid_argument` if no number could be
 *         parsed, and `std::errc::result_out_of_range` if a numeric value was parsed but did not fit into a double.
 *         It is implementation specific, whether `result_out_of_range` or 0/inf will be returned for too small/large
 *         values. `ptr` points to the first character that was not parsed as a part of the numeric value.
 */
inline ::std::from_chars_result from_chars(const ::std::string& string, double& value,
  const ::std::chars_format fmt = ::std::chars_format::general) noexcept
{
  return ::std::from_chars(string.data(), string.data() + string.size(), value, fmt);
}

}
