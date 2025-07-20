#pragma once

/**
 * \file
 * \brief Shim implementing std::from_chars support for floating-point number types in GCC < 11.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <system_error>

namespace cras
{

enum chars_format
{
  scientific = 1 << 0,
  fixed = 1 << 2,
  hex = 1 << 3,
  general = fixed | scientific,
};

struct from_chars_result
{
  const char* ptr;
  ::std::errc ec;
};

/**
 * \brief Convert the given string to its best float representation. The behavior is similar to std::from_chars(),
 *        i.e. no leading space of plus sign is allowed.
 * \param[in] first Pointer to first char of the string.
 * \param[in] last Pointer to last char of the string.
 * \param[out] value The parsed value (valid only if parsing succeeded, i.e. `ec` in the result is 0).
 * \param[in] fmt Bitmask of `cras::chars_format` values that specify the format rules with which the value should
 *                be interpreted. Not all formats have to be supported.
 * \return Struct containing the error code `ec` - 0 on success, `std::errc::invalid_argument` if no number could be
 *         parsed, and `std::errc::result_out_of_range` if a numeric value was parsed but did not fit into a float.
 *         It is implementation specific, whether `result_out_of_range` or 0/inf will be returned for too small/large
 *         values. `ptr` points to the first character that was not parsed as a part of the numeric value.
 */
::cras::from_chars_result from_chars(
  const char* first, const char* last, float& value,
  ::cras::chars_format fmt = ::cras::chars_format::general) noexcept;

/**
 * \brief Convert the given string to its best double representation. The behavior is similar to std::from_chars(),
 *        i.e. no leading space of plus sign is allowed.
 * \param[in] first Pointer to first char of the string.
 * \param[in] last Pointer to last char of the string.
 * \param[out] value The parsed value (valid only if parsing succeeded, i.e. `ec` in the result is 0).
 * \param[in] fmt Bitmask of `cras::chars_format` values that specify the format rules with which the value should
 *                be interpreted. Not all formats have to be supported.
 * \return Struct containing the error code `ec` - 0 on success, `std::errc::invalid_argument` if no number could be
 *         parsed, and `std::errc::result_out_of_range` if a numeric value was parsed but did not fit into a double.
 *         It is implementation specific, whether `result_out_of_range` or 0/inf will be returned for too small/large
 *         values. `ptr` points to the first character that was not parsed as a part of the numeric value.
 */
::cras::from_chars_result from_chars(
  const char* first, const char* last, double& value,
  ::cras::chars_format fmt = ::cras::chars_format::general) noexcept;

/**
 * \brief Convert the given string to its best float representation. The behavior is similar to std::from_chars(),
 *        i.e. no leading space of plus sign is allowed.
 * \param[in] string The string to convert.
 * \param[out] value The parsed value (valid only if parsing succeeded, i.e. `ec` in the result is 0).
 * \param[in] fmt Bitmask of `cras::chars_format` values that specify the format rules with which the value should
 *                be interpreted. Not all formats have to be supported.
 * \return Struct containing the error code `ec` - 0 on success, `std::errc::invalid_argument` if no number could be
 *         parsed, and `std::errc::result_out_of_range` if a numeric value was parsed but did not fit into a float.
 *         It is implementation specific, whether `result_out_of_range` or 0/inf will be returned for too small/large
 *         values. `ptr` points to the first character that was not parsed as a part of the numeric value.
 */
inline ::cras::from_chars_result from_chars(
  const ::std::string& string, float& value,
  ::cras::chars_format fmt = ::cras::chars_format::general) noexcept
{
  return ::cras::from_chars(string.data(), string.data() + string.size(), value, fmt);
}

/**
 * \brief Convert the given string to its best double representation. The behavior is similar to std::from_chars(),
 *        i.e. no leading space of plus sign is allowed.
 * \param[in] string The string to convert.
 * \param[out] value The parsed value (valid only if parsing succeeded, i.e. `ec` in the result is 0).
 * \param[in] fmt Bitmask of `cras::chars_format` values that specify the format rules with which the value should
 *                be interpreted. Not all formats have to be supported.
 * \return Struct containing the error code `ec` - 0 on success, `std::errc::invalid_argument` if no number could be
 *         parsed, and `std::errc::result_out_of_range` if a numeric value was parsed but did not fit into a double.
 *         It is implementation specific, whether `result_out_of_range` or 0/inf will be returned for too small/large
 *         values. `ptr` points to the first character that was not parsed as a part of the numeric value.
 */
inline ::cras::from_chars_result from_chars(const ::std::string& string, double& value,
  ::cras::chars_format fmt = ::cras::chars_format::general) noexcept
{
  return ::cras::from_chars(string.data(), string.data() + string.size(), value, fmt);
}

}
