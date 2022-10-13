/**
 * \file
 * \brief Test availability of std::from_chars() for floating point values.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <charconv>

void fn()
{
  double d;
  const char* s = "1.0";
  std::from_chars(s, s + 3, d);
}