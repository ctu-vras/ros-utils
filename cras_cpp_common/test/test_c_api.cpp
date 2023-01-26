// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for c_api.h.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <cras_cpp_common/c_api.h>

using namespace cras;

static size_t lastAllocSize {0u};

void* alloc(const size_t size)
{
  lastAllocSize = size;
  return new uint8_t[size];
}

TEST(CApi, outputString)  // NOLINT
{
  {
    std::string s = "test";
    const auto o = outputString(&alloc, s);
    EXPECT_STREQ(s.c_str(), o);
    EXPECT_EQ(5u, lastAllocSize);
    delete o;
  }
  {
    std::string s = "testatest";
    s[4] = '\0';
    const auto o = outputString(&alloc, s);
    EXPECT_STREQ(s.c_str(), o);
    EXPECT_EQ(4u, strlen(o));
    EXPECT_EQ(10u, lastAllocSize);
    delete o;
  }
  {
    const char* s = "test";
    const auto o = outputString(&alloc, s, strlen(s) + 1);
    EXPECT_STREQ(s, o);
    EXPECT_EQ(5u, lastAllocSize);
    delete o;
  }
  {
    const char* s = "test";
    const auto o = outputString(&alloc, s, strlen(s));
    EXPECT_EQ(0, strncmp(s, o, 4));
    EXPECT_EQ(4u, lastAllocSize);
    delete o;
  }
}

TEST(CApi, outputByteArray)  // NOLINT
{
  {
    std::vector<uint8_t> b = {0, 1, 2, 3, 4, 5};
    const auto o = outputByteBuffer(&alloc, b);
    EXPECT_EQ(0, memcmp(b.data(), o, b.size()));
    EXPECT_EQ(6u, lastAllocSize);
    delete o;
  }
  {
    std::vector<uint8_t> b = {0, 1, 2, 3, 4, 5};
    const auto o = outputByteBuffer(&alloc, b.data(), b.size());
    EXPECT_EQ(0, memcmp(b.data(), o, b.size()));
    EXPECT_EQ(6u, lastAllocSize);
    delete o;
  }
  {
    std::vector<uint8_t> b = {0, 1, 2, 3, 4, 5};
    const auto o = outputByteBuffer(&alloc, b.data(), 3);
    EXPECT_EQ(0, memcmp(b.data(), o, 3));
    EXPECT_EQ(3u, lastAllocSize);
    delete o;
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
