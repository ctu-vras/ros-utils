#pragma once

/**
 * \file
 * \brief Utilities for working with sets.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <algorithm>
#include <iterator>
#include <set>

namespace cras
{

/**
 * \brief Test whether the two given sets have empty intersection.
 * \tparam T Type of the elements in the sets.
 * \param[in] set1 First set to test.
 * \param[in] set2 Second set to test.
 * \return Whether the intersection of set1 and set2 is empty.
 */
template<typename T>
bool isSetIntersectionEmpty(const ::std::set<T>& set1, const ::std::set<T>& set2)
{
  ::std::set<T> tmpSet;
  ::std::set_intersection(
      set1.begin(), set1.end(),
      set2.begin(), set2.end(),
      ::std::inserter(tmpSet, tmpSet.end()));
  return tmpSet.empty();
}

}
