#pragma once

/**
 * \file
 * \brief Support for literal suffix _sz creating size_t values.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cstddef>

/**
 * This operator allows you to write size_t literals like 5_sz.
 */
inline size_t operator "" _sz(unsigned long long int x)
{
  return x;
}
