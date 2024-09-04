// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for small_map.hpp.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <string>

#include <cras_cpp_common/small_map.hpp>


struct Data
{
  Data() : x(0.0), y(0) {}
  Data(const double x, const int y, const std::string& s) : x(x), y(y), s(s) {}

  double x;
  int y;
  std::string s;
};

struct AggregateData
{
  double x;
  int y;
  std::string s;
};


TEST(SmallMap, IntInt)  // NOLINT
{
  cras::SmallMap<int, int> map;
  const auto& constMap = map;

  EXPECT_FALSE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_FALSE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_EQ(map.size(), 0u);
  EXPECT_TRUE(map.empty());

  map[0] = 1;

  EXPECT_TRUE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_TRUE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_EQ(map[0], 1);
  EXPECT_EQ(map.at(0), 1);
  EXPECT_EQ(constMap.at(0), 1);
  EXPECT_EQ(map.size(), 1u);
  EXPECT_FALSE(map.empty());

  map[0] = 2;

  EXPECT_TRUE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_TRUE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_EQ(map[0], 2);
  EXPECT_EQ(map.at(0), 2);
  EXPECT_EQ(constMap.at(0), 2);
  EXPECT_EQ(map.size(), 1u);
  EXPECT_FALSE(map.empty());

  map[2] = 3;

  EXPECT_TRUE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_TRUE(map.contains(2));
  EXPECT_TRUE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_TRUE(constMap.contains(2));
  EXPECT_EQ(map[0], 2);
  EXPECT_EQ(map.at(0), 2);
  EXPECT_EQ(constMap.at(0), 2);
  EXPECT_EQ(map[2], 3);
  EXPECT_EQ(map.at(2), 3);
  EXPECT_EQ(constMap.at(2), 3);
  EXPECT_EQ(map.size(), 2u);
  EXPECT_FALSE(map.empty());

  map.insertIfNew(2, 42);
  EXPECT_EQ(map.size(), 2u);
  EXPECT_EQ(map[2], 3);

  map.insertIfNew(2, 42) = 41;
  EXPECT_EQ(map.size(), 2u);
  EXPECT_EQ(map[2], 41);

  map.insertIfNew(8, 9);
  EXPECT_EQ(map.size(), 3u);
  EXPECT_EQ(map[0], 2);
  EXPECT_EQ(map[2], 41);
  EXPECT_EQ(map[8], 9);
  EXPECT_FALSE(map.empty());
}

TEST(SmallMap, IntString)  // NOLINT
{
  cras::SmallMap<int, std::string> map;
  const auto& constMap = map;

  EXPECT_FALSE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_FALSE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_EQ(map.size(), 0u);
  EXPECT_TRUE(map.empty());

  map[0] = "a";

  EXPECT_TRUE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_TRUE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_EQ(map[0], "a");
  EXPECT_EQ(map.at(0), "a");
  EXPECT_EQ(constMap.at(0), "a");
  EXPECT_EQ(map.size(), 1u);
  EXPECT_FALSE(map.empty());

  map[0] = "b";

  EXPECT_TRUE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_TRUE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_EQ(map[0], "b");
  EXPECT_EQ(map.at(0), "b");
  EXPECT_EQ(constMap.at(0), "b");
  EXPECT_EQ(map.size(), 1u);
  EXPECT_FALSE(map.empty());

  map[2] = "c";

  EXPECT_TRUE(map.contains(0));
  EXPECT_FALSE(map.contains(1));
  EXPECT_TRUE(map.contains(2));
  EXPECT_TRUE(constMap.contains(0));
  EXPECT_FALSE(constMap.contains(1));
  EXPECT_TRUE(constMap.contains(2));
  EXPECT_EQ(map[0], "b");
  EXPECT_EQ(map.at(0), "b");
  EXPECT_EQ(constMap.at(0), "b");
  EXPECT_EQ(map[2], "c");
  EXPECT_EQ(map.at(2), "c");
  EXPECT_EQ(constMap.at(2), "c");
  EXPECT_EQ(map.size(), 2u);
  EXPECT_FALSE(map.empty());

  map.insertIfNew(2, "z");
  EXPECT_EQ(map.size(), 2u);
  EXPECT_EQ(map[2], "c");

  map.insertIfNew(2, "z") = "f";
  EXPECT_EQ(map.size(), 2u);
  EXPECT_EQ(map[2], "f");

  map.insertIfNew(8, "h");
  EXPECT_EQ(map.size(), 3u);
  EXPECT_EQ(map[0], "b");
  EXPECT_EQ(map[2], "f");
  EXPECT_EQ(map[8], "h");
  EXPECT_FALSE(map.empty());
}

TEST(SmallMap, ComplexType)  // NOLINT
{
  cras::SmallMap<int, Data> map;
  EXPECT_TRUE(map.empty());

  map.insertIfNew(2, 1.0, 2, "c");
  EXPECT_EQ(map.size(), 1u);
  EXPECT_EQ(map[2].x, 1.0);
  EXPECT_EQ(map[2].y, 2);
  EXPECT_EQ(map[2].s, "c");
  EXPECT_FALSE(map.empty());

  map.insertIfNew(2, 2.0, 4, "z") = {3.0, 8, "f"};
  EXPECT_EQ(map.size(), 1u);
  EXPECT_EQ(map[2].x, 3.0);
  EXPECT_EQ(map[2].y, 8);
  EXPECT_EQ(map[2].s, "f");
  EXPECT_FALSE(map.empty());

  map.insertIfNew(8, 4.0, 5, "h");
  EXPECT_EQ(map.size(), 2u);
  EXPECT_EQ(map[2].x, 3.0);
  EXPECT_EQ(map[2].y, 8);
  EXPECT_EQ(map[2].s, "f");
  EXPECT_EQ(map[8].x, 4.0);
  EXPECT_EQ(map[8].y, 5);
  EXPECT_EQ(map[8].s, "h");
  EXPECT_FALSE(map.empty());
}

TEST(SmallMap, AggregateType)  // NOLINT
{
  cras::SmallMap<int, AggregateData> map;
  EXPECT_TRUE(map.empty());

  map.insertIfNew(2, 1.0, 2, "c");
  EXPECT_EQ(map.size(), 1u);
  EXPECT_EQ(map[2].x, 1.0);
  EXPECT_EQ(map[2].y, 2);
  EXPECT_EQ(map[2].s, "c");
  EXPECT_FALSE(map.empty());

  map.insertIfNew(2, 2.0, 4, "z") = {3.0, 8, "f"};
  EXPECT_EQ(map.size(), 1u);
  EXPECT_EQ(map[2].x, 3.0);
  EXPECT_EQ(map[2].y, 8);
  EXPECT_EQ(map[2].s, "f");
  EXPECT_FALSE(map.empty());

  map.insertIfNew(8, 4.0, 5, "h");
  EXPECT_EQ(map.size(), 2u);
  EXPECT_EQ(map[2].x, 3.0);
  EXPECT_EQ(map[2].y, 8);
  EXPECT_EQ(map[2].s, "f");
  EXPECT_EQ(map[8].x, 4.0);
  EXPECT_EQ(map[8].y, 5);
  EXPECT_EQ(map[8].s, "h");
  EXPECT_FALSE(map.empty());
}

TEST(SmallSet, Int)  // NOLINT
{
  cras::SmallSet<int> set;

  EXPECT_FALSE(set.contains(0));
  EXPECT_FALSE(set.contains(1));
  EXPECT_EQ(set.size(), 0u);
  EXPECT_TRUE(set.empty());

  EXPECT_TRUE(set.insert(0));

  EXPECT_TRUE(set.contains(0));
  EXPECT_FALSE(set.contains(1));
  EXPECT_EQ(set.size(), 1u);
  EXPECT_FALSE(set.empty());

  EXPECT_FALSE(set.insert(0));

  EXPECT_TRUE(set.contains(0));
  EXPECT_FALSE(set.contains(1));
  EXPECT_EQ(set.size(), 1u);
  EXPECT_FALSE(set.empty());

  EXPECT_TRUE(set.insert(2));

  EXPECT_TRUE(set.contains(0));
  EXPECT_FALSE(set.contains(1));
  EXPECT_TRUE(set.contains(2));
  EXPECT_EQ(set.size(), 2u);
  EXPECT_FALSE(set.empty());

  EXPECT_FALSE(set.insert(2));
  EXPECT_EQ(set.size(), 2u);
  EXPECT_TRUE(set.contains(2));
}

TEST(SmallSet, String)  // NOLINT
{
  cras::SmallSet<std::string> set;

  EXPECT_FALSE(set.contains("a"));
  EXPECT_FALSE(set.contains("b"));
  EXPECT_EQ(set.size(), 0u);
  EXPECT_TRUE(set.empty());

  EXPECT_TRUE(set.insert("a"));

  EXPECT_TRUE(set.contains("a"));
  EXPECT_FALSE(set.contains("b"));
  EXPECT_EQ(set.size(), 1u);
  EXPECT_FALSE(set.empty());

  EXPECT_FALSE(set.insert("a"));

  EXPECT_TRUE(set.contains("a"));
  EXPECT_FALSE(set.contains("b"));
  EXPECT_EQ(set.size(), 1u);
  EXPECT_FALSE(set.empty());

  EXPECT_TRUE(set.insert("c"));

  EXPECT_TRUE(set.contains("a"));
  EXPECT_FALSE(set.contains("b"));
  EXPECT_TRUE(set.contains("c"));
  EXPECT_EQ(set.size(), 2u);
  EXPECT_FALSE(set.empty());

  EXPECT_FALSE(set.insert("c"));
  EXPECT_EQ(set.size(), 2u);
  EXPECT_TRUE(set.contains("c"));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
