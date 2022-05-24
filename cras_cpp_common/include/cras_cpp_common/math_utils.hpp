#pragma once

/**
 * \file
 * \brief Math utilities.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

/**
 * \brief Return the sign of the given value (-1, 0 or +1).
 * \tparam T Type of the number.
 * \param val The value to get sign of.
 * \return Sign of the value: -1 for negative, 0 for 0, +1 for positive numbers.
 */
template <typename T> inline int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
