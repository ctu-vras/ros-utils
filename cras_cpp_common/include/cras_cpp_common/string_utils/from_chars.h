#pragma once

/**
 * \file
 * \brief Shim implementing std::from_chars support for floating-point number types in GCC < 11.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <system_error>

namespace cras
{
  enum chars_format
  {
    scientific = 1 << 0,
    fixed = 1 << 2,
    hex = 1 << 3,
    general = fixed | scientific
  };
  
  struct from_chars_result
  {
    const char* ptr;
    ::std::errc ec;
  };

  ::cras::from_chars_result from_chars(const char* first, const char* last, float& value,
    ::cras::chars_format fmt = ::cras::chars_format::general) noexcept;

  ::cras::from_chars_result from_chars(const char* first, const char* last, double& value,
    ::cras::chars_format fmt = ::cras::chars_format::general) noexcept;
}
