/**
 * \file
 * \brief Unit test for type_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <vector>

#include <cras_cpp_common/type_utils.hpp>

#if __has_include(<cxxabi.h>)
#define HAS_CXX_ABI 1
#endif

TEST(TypeUtils, sz)  // NOLINT
{
  EXPECT_EQ(5u, 5_sz);
  EXPECT_EQ(typeid(size_t), typeid(5_sz));
}

TEST(TypeUtils, CleanTypeName)  // NOLINT
{
  EXPECT_EQ("int", cras::cleanTypeName("int"));
  EXPECT_EQ("double", cras::cleanTypeName("double"));
  EXPECT_EQ("size_t", cras::cleanTypeName("size_t"));
  EXPECT_EQ("cras::TestClass", cras::cleanTypeName("cras::TestClass"));

  EXPECT_EQ("std::string", cras::cleanTypeName("std::string"));
  EXPECT_EQ("std::string", cras::cleanTypeName("std::__cxx11::basic_string<char >"));
  EXPECT_EQ("std::string",
    cras::cleanTypeName("std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >"));
  EXPECT_EQ("std::string", cras::cleanTypeName(cras::demangle(typeid(std::string).name())));

  EXPECT_EQ("std::vector<int>", cras::cleanTypeName("std::vector<int, std::allocator<int> >"));
  EXPECT_EQ("std::vector<int>", cras::cleanTypeName(cras::demangle(typeid(std::vector<int>).name())));
}

TEST(TypeUtils, Demangle)  // NOLINT
{
  EXPECT_EQ("int", cras::demangle("int"));
  EXPECT_EQ("double", cras::demangle("double"));
  EXPECT_EQ("size_t", cras::demangle("size_t"));

#if HAS_CXX_ABI
  EXPECT_EQ("std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >",
    cras::demangle(typeid(std::string).name()));

  EXPECT_EQ("std::vector<int, std::allocator<int> >", cras::demangle(typeid(std::vector<int>).name()));
#endif
}

TEST(TypeUtils, GetTypeName)  // NOLINT
{
  EXPECT_EQ("int", cras::getTypeName(typeid(1)));
  EXPECT_EQ("int", cras::getTypeName<int>());
  EXPECT_EQ("double", cras::getTypeName(typeid(1.0)));
  EXPECT_EQ("double", cras::getTypeName<double>());
  // there is a litle inevitable difference in the templated and non-templated version
  EXPECT_EQ("unsigned long", cras::getTypeName(typeid(1_sz)));
  EXPECT_EQ("long unsigned int", cras::getTypeName<size_t>());
  
  EXPECT_EQ("std::string", cras::getTypeName(typeid(std::string("test"))));
  EXPECT_EQ("std::string", cras::getTypeName<std::string>());
  
  EXPECT_EQ("std::vector<int>", cras::getTypeName(typeid(std::vector<int>)));
  EXPECT_EQ("std::vector<int>", cras::getTypeName<std::vector<int>>());

  EXPECT_EQ("std::vector<std::string>", cras::getTypeName(typeid(std::vector<std::string>)));
  EXPECT_EQ("std::vector<std::string>", cras::getTypeName<std::vector<std::string>>());
  
  EXPECT_EQ("std::list<int>", cras::getTypeName(typeid(std::list<int>)));
  EXPECT_EQ("std::list<int>", cras::getTypeName<std::list<int>>());

  EXPECT_EQ("std::list<std::string>", cras::getTypeName(typeid(std::list<std::string>)));
  EXPECT_EQ("std::list<std::string>", cras::getTypeName<std::list<std::string>>());

  EXPECT_EQ("std::set<int>", cras::getTypeName(typeid(std::set<int>)));
  EXPECT_EQ("std::set<int>", cras::getTypeName<std::set<int>>());

  EXPECT_EQ("std::unordered_set<int>", cras::getTypeName(typeid(std::unordered_set<int>)));
  EXPECT_EQ("std::unordered_set<int>", cras::getTypeName<std::unordered_set<int>>());

  EXPECT_EQ("std::map<std::string, std::string>", cras::getTypeName(typeid(std::map<std::string, std::string>)));
  EXPECT_EQ("std::map<std::string, std::string>", (cras::getTypeName<std::map<std::string, std::string>>()));

  EXPECT_EQ("std::unordered_map<std::string, int>", cras::getTypeName(typeid(std::unordered_map<std::string, int>)));
  EXPECT_EQ("std::unordered_map<std::string, int>", (cras::getTypeName<std::unordered_map<std::string, int>>()));

  EXPECT_EQ("std::unordered_map<int, std::string>", cras::getTypeName(typeid(std::unordered_map<int, std::string>)));
  EXPECT_EQ("std::unordered_map<int, std::string>", (cras::getTypeName<std::unordered_map<int, std::string>>()));

  EXPECT_EQ("std::map<std::vector<std::string>, int>",
    cras::getTypeName(typeid(std::map<std::vector<std::string>, int>)));
  EXPECT_EQ("std::map<std::vector<std::string>, int>", (cras::getTypeName<std::map<std::vector<std::string>, int>>()));

  EXPECT_EQ("std::map<int, std::vector<std::string>>",
    cras::getTypeName(typeid(std::map<int, std::vector<std::string>>)));
  EXPECT_EQ("std::map<int, std::vector<std::string>>", (cras::getTypeName<std::map<int, std::vector<std::string>>>()));
}

TEST(TypeUtils, IsString)  // NOLINT
{
  EXPECT_EQ(std::true_type::value, cras::is_c_string<char*>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<char*>::value);
  EXPECT_EQ(std::true_type::value, cras::is_c_string<const char*>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<const char*>::value);
  EXPECT_EQ(std::true_type::value, cras::is_c_string<char[1]>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<char[1]>::value);
  EXPECT_EQ(std::true_type::value, cras::is_c_string<const char[1]>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<const char[1]>::value);
  EXPECT_EQ(std::true_type::value, cras::is_c_string<char* const>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<char* const>::value);
  EXPECT_EQ(std::true_type::value, cras::is_c_string<const char* const>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<const char* const>::value);
  EXPECT_EQ(std::true_type::value, cras::is_c_string<char const[1]>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<char const[1]>::value);
  EXPECT_EQ(std::false_type::value, cras::is_c_string<std::string>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<std::string>::value);
  EXPECT_EQ(std::false_type::value, cras::is_c_string<const std::string>::value);
  EXPECT_EQ(std::true_type::value, cras::is_string<const std::string>::value);
  
  EXPECT_EQ(std::false_type::value, cras::is_string<int>::value);
  EXPECT_EQ(std::false_type::value, cras::is_string<double>::value);
  EXPECT_EQ(std::false_type::value, cras::is_string<std::vector<std::string>>::value);
  EXPECT_EQ(std::false_type::value, cras::is_string<std::vector<char*>>::value);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}