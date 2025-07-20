/**
 * \file
 * \brief Shim implementing std::from_chars support for floating-point number types in GCC < 11.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cras_cpp_common/string_utils/from_chars.hpp>

// Passed from CMake as a result of try_compile
#if defined(HAS_FROM_CHARS_FLOAT) && HAS_FROM_CHARS_FLOAT == 1

#include <charconv>

namespace cras
{

cras::from_chars_result from_chars(const char* first, const char* last, float& value,
                                   const cras::chars_format fmt) noexcept
{
  auto result = std::from_chars(first, last, value, static_cast<std::chars_format>(fmt));
  return {result.ptr, result.ec};
}

cras::from_chars_result from_chars(const char* first, const char* last, double& value,
                                   const cras::chars_format fmt) noexcept
{
  auto result = std::from_chars(first, last, value, static_cast<std::chars_format>(fmt));
  return {result.ptr, result.ec};
}

}

#else

#include <cras_cpp_common/external/fast_float/fast_float.h>

namespace cras
{

cras::from_chars_result from_chars(const char* first, const char* last, float& value,
  const cras::chars_format fmt) noexcept
{
  auto result = fast_float::from_chars(first, last, value, static_cast<fast_float::chars_format>(fmt));
  return {result.ptr, result.ec};
}

cras::from_chars_result from_chars(const char* first, const char* last, double& value,
  const cras::chars_format fmt) noexcept
{
  auto result = fast_float::from_chars(first, last, value, static_cast<fast_float::chars_format>(fmt));
  return {result.ptr, result.ec};
}

}

#endif
