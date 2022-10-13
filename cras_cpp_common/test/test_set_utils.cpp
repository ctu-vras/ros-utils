/**
 * \file
 * \brief Unit test for set_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <set>

#include <cras_cpp_common/set_utils.hpp>

TEST(SetUtils, isSetIntersectionEmpty)
{
  std::set<int> a = { 1, 2, 4, -3 };
  std::set<int> b = { -1, -2, -4, 3 };
  std::set<int> c = { 4, 3 };
  std::set<int> e;

  EXPECT_FALSE(cras::isSetIntersectionEmpty(a, a));
  EXPECT_FALSE(cras::isSetIntersectionEmpty(b, b));
  EXPECT_FALSE(cras::isSetIntersectionEmpty(a, c));
  EXPECT_FALSE(cras::isSetIntersectionEmpty(b, c));
  EXPECT_FALSE(cras::isSetIntersectionEmpty(c, a));
  EXPECT_FALSE(cras::isSetIntersectionEmpty(c, b));

  EXPECT_TRUE(cras::isSetIntersectionEmpty(a, b));
  EXPECT_TRUE(cras::isSetIntersectionEmpty(b, a));

  // empty set handling
  EXPECT_TRUE(cras::isSetIntersectionEmpty(a, e));
  EXPECT_TRUE(cras::isSetIntersectionEmpty(b, e));
  EXPECT_TRUE(cras::isSetIntersectionEmpty(e, e));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
