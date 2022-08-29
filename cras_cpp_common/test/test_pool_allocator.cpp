/**
 * \file
 * \brief Unit test for pool_allocator.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <cras_cpp_common/pool_allocator.hpp>

TEST(PoolAllocator, MakeSharedFast)  // NOLINT
{
  auto value = cras::make_shared_from_fast_pool<int>(1);
  ASSERT_NE(nullptr, value);
  EXPECT_EQ(1, *value);

  value = cras::make_shared_from_fast_pool<int>(2);
  ASSERT_NE(nullptr, value);
  EXPECT_EQ(2, *value);
}

TEST(PoolAllocator, MakeShared)  // NOLINT
{
  auto value = cras::make_shared_from_pool<int>(1);
  ASSERT_NE(nullptr, value);
  EXPECT_EQ(1, *value);

  value = cras::make_shared_from_pool<int>(2);
  ASSERT_NE(nullptr, value);
  EXPECT_EQ(2, *value);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
