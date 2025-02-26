/**
 * \file
 * \brief Unit test for xmlrpc_value_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <array>
#include <list>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cras_cpp_common/xmlrpc_value_utils.hpp>

using namespace cras;

#define TRACE SCOPED_TRACE("Failure happened here");

template <typename T>
void test_converts(
  const XmlRpc::XmlRpcValue& xmlVal, const T& expVal, const bool skip = true, const bool withErrors = false)
{
  T v = {};
  std::list<std::string> e;
  bool success = convert(xmlVal, v, skip, &e);
  EXPECT_TRUE(success);
  if (success) {
    EXPECT_EQ(expVal, v);
    EXPECT_EQ(!withErrors, e.empty());
  }
}

template <typename T>
void test_not_converts(const XmlRpc::XmlRpcValue& xmlVal, const bool skip = true)
{
  T v = {};
  std::list<std::string> e;
  EXPECT_FALSE(convert(xmlVal, v, skip, &e));
  EXPECT_FALSE(e.empty());
  for (const auto& err : e)
    EXPECT_FALSE(err.empty());
}

TEST(XmlRpcValueUtils, ConvertBasic)
{
  test_converts<bool>(true, true);
  test_converts<bool>(false, false);
  test_converts<int>(0, 0);
  test_converts<int>(1, 1);
  test_converts<int>(-1, -1);
  test_converts<int>(INT_MAX, INT_MAX);
  test_converts<int>(INT_MIN, INT_MIN);
  test_converts<double>(0.0, 0.0);
  test_converts<double>(1.0, 1.0);
  test_converts<double>(-1.0, -1.0);
  test_converts<double>(3.14, 3.14);
  test_converts<std::string>("test", "test");
  test_converts<std::string>("", "");

  test_converts<short>(0, 0);  // NOLINT
  test_converts<short>(1, 1);  // NOLINT
  test_converts<short>(-1, -1);  // NOLINT
  test_converts<char>(0, 0);
  test_converts<char>(1, 1);
  test_converts<char>(-1, -1);
  test_converts<signed char>(0, 0);
  test_converts<signed char>(1, 1);
  test_converts<signed char>(-1, -1);
  test_converts<long>(0, 0);  // NOLINT
  test_converts<long>(1, 1);  // NOLINT
  test_converts<long>(-1, -1);  // NOLINT
  test_converts<long>(INT_MAX, INT_MAX);  // NOLINT
  test_converts<long>(INT_MIN, INT_MIN);  // NOLINT
  test_converts<long long>(0, 0);  // NOLINT
  test_converts<long long>(1, 1);  // NOLINT
  test_converts<long long>(-1, -1);  // NOLINT
  test_converts<long long>(INT_MAX, INT_MAX);  // NOLINT
  test_converts<long long>(INT_MIN, INT_MIN);  // NOLINT

  test_converts<unsigned short>(0, 0);  // NOLINT
  test_converts<unsigned short>(1, 1);  // NOLINT
  test_converts<unsigned short>(USHRT_MAX, USHRT_MAX);  // NOLINT
  test_converts<unsigned char>(0, 0);
  test_converts<unsigned char>(1, 1);
  test_converts<unsigned char>(UCHAR_MAX, UCHAR_MAX);
  test_converts<unsigned long>(0, 0);  // NOLINT
  test_converts<unsigned long>(1, 1);  // NOLINT
  test_converts<unsigned long>(INT_MAX, INT_MAX);  // NOLINT
  test_converts<unsigned long long>(0, 0);  // NOLINT
  test_converts<unsigned long long>(1, 1);  // NOLINT
  test_converts<unsigned long long>(INT_MAX, INT_MAX);  // NOLINT
  test_converts<size_t>(0, 0);
  test_converts<size_t>(1, 1);
  test_converts<size_t>(INT_MAX, INT_MAX);

  test_converts<float>(0.0, 0.0f);
  test_converts<float>(1.0, 1.0f);
  test_converts<float>(-1.0, -1.0f);
  test_converts<float>(3.14, 3.14f);
  test_converts<long double>(0.0, (long double)0.0);
  test_converts<long double>(1.0, (long double)1.0);
  test_converts<long double>(-1.0, -(long double)1.0);
  test_converts<long double>(3.14, (long double)3.14);

  test_converts<bool>(0, false);
  test_converts<bool>(1, true);
  test_converts<double>(0, 0.0);
  test_converts<double>(1, 1.0);
  test_converts<double>(-1, -1.0);
  test_converts<float>(-1, -1.0f);

  test_not_converts<bool>(2);
  test_not_converts<bool>(-1);
  test_not_converts<int>(false);
  test_not_converts<int>(true);
  test_not_converts<int>(0.0);
  test_not_converts<int>(1.0);
  test_not_converts<unsigned char>(-1);
  test_not_converts<unsigned short>(-1);  // NOLINT
  test_not_converts<unsigned int>(-1);
  test_not_converts<unsigned long>(-1);  // NOLINT
  test_not_converts<unsigned long long>(-1);  // NOLINT
  test_not_converts<std::string>(-1);
  test_not_converts<bool>("1");
  test_not_converts<int>("1");
  test_not_converts<char>("1");
  test_not_converts<short>("1");  // NOLINT
  test_not_converts<long>("1");  // NOLINT
  test_not_converts<long long>("1");  // NOLINT
  test_not_converts<unsigned char>("1");
  test_not_converts<unsigned short>("1");  // NOLINT
  test_not_converts<unsigned int>("1");
  test_not_converts<size_t>("1");
  test_not_converts<unsigned long>("1");  // NOLINT
  test_not_converts<unsigned long long>("1");  // NOLINT
}

TEST(XmlRpcValueUtils, ConvertArrays)
{
  XmlRpc::XmlRpcValue x;

  x.clear(); x.setSize(2); x[0] = true; x[1] = false;
  {TRACE; test_converts<std::vector<bool>>(x, {true, false}, true, false);}

  x.clear(); x.setSize(3); x[0] = 0; x[1] = 1; x[2] = -1;
  {TRACE; test_converts<std::vector<bool>>(x, {false, true}, true, true);}
  {TRACE; test_converts<std::vector<int>>(x, {0, 1, -1}, true, false);}
  {TRACE; test_converts<std::vector<char>>(x, {0, 1, static_cast<char>(-1)}, true, false);}
  {TRACE; test_converts<std::vector<short>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::vector<long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::vector<long long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::vector<unsigned char>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::vector<unsigned short>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::vector<unsigned int>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::vector<size_t>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::vector<unsigned long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::vector<unsigned long long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::vector<float>>(x, {0.0f, 1.0f, -1.0f}, true, false);}
  {TRACE; test_converts<std::vector<double>>(x, {0.0, 1.0, -1.0}, true, false);}
  {TRACE; test_converts<std::vector<long double>>(x, {0.0, 1.0, -1.0}, true, false);}

  {TRACE; test_converts<std::set<bool>>(x, {false, true}, true, true);}
  {TRACE; test_converts<std::set<int>>(x, {0, 1, -1}, true, false);}
  {TRACE; test_converts<std::set<char>>(x, {0, 1, static_cast<char>(-1)}, true, false);}
  {TRACE; test_converts<std::set<short>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::set<long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::set<long long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::set<unsigned char>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::set<unsigned short>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::set<unsigned int>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::set<size_t>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::set<unsigned long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::set<unsigned long long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::set<float>>(x, {0.0f, 1.0f, -1.0f}, true, false);}
  {TRACE; test_converts<std::set<double>>(x, {0.0, 1.0, -1.0}, true, false);}
  {TRACE; test_converts<std::set<long double>>(x, {0.0, 1.0, -1.0}, true, false);}

  {TRACE; test_converts<std::list<bool>>(x, {false, true}, true, true);}
  {TRACE; test_converts<std::list<int>>(x, {0, 1, -1}, true, false);}
  {TRACE; test_converts<std::list<char>>(x, {0, 1, static_cast<char>(-1)}, true, false);}
  {TRACE; test_converts<std::list<short>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::list<long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::list<long long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::list<unsigned char>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::list<unsigned short>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::list<unsigned int>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::list<size_t>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::list<unsigned long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::list<unsigned long long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::list<float>>(x, {0.0f, 1.0f, -1.0f}, true, false);}
  {TRACE; test_converts<std::list<double>>(x, {0.0, 1.0, -1.0}, true, false);}
  {TRACE; test_converts<std::list<long double>>(x, {0.0, 1.0, -1.0}, true, false);}

  {TRACE; test_converts<std::unordered_set<bool>>(x, {false, true}, true, true);}
  {TRACE; test_converts<std::unordered_set<int>>(x, {0, 1, -1}, true, false);}
  {TRACE; test_converts<std::unordered_set<char>>(x, {0, 1, static_cast<char>(-1)}, true, false);}
  {TRACE; test_converts<std::unordered_set<short>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_set<long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_set<long long>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_set<unsigned char>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::unordered_set<unsigned short>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::unordered_set<unsigned int>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::unordered_set<size_t>>(x, {0, 1}, true, true);}
  {TRACE; test_converts<std::unordered_set<unsigned long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::unordered_set<unsigned long long>>(x, {0, 1}, true, true);}  // NOLINT
  {TRACE; test_converts<std::unordered_set<float>>(x, {0.0f, 1.0f, -1.0f}, true, false);}
  {TRACE; test_converts<std::unordered_set<double>>(x, {0.0, 1.0, -1.0}, true, false);}
  {TRACE; test_converts<std::unordered_set<long double>>(x, {0.0, 1.0, -1.0}, true, false);}

  {TRACE; test_converts<std::array<int, 3>>(x, {0, 1, -1}, true, false);}
  {TRACE; test_converts<std::array<char, 3>>(x, {0, 1, static_cast<char>(-1)}, true, false);}
  {TRACE; test_converts<std::array<short, 3>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::array<long, 3>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::array<long long, 3>>(x, {0, 1, -1}, true, false);}  // NOLINT
  {TRACE; test_converts<std::array<float, 3>>(x, {0.0f, 1.0f, -1.0f}, true, false);}
  {TRACE; test_converts<std::array<double, 3>>(x, {0.0, 1.0, -1.0}, true, false);}
  {TRACE; test_converts<std::array<long double, 3>>(x, {0.0, 1.0, -1.0}, true, false);}
  {TRACE; test_not_converts<std::array<bool, 3>>(x, true);}
  {TRACE; test_not_converts<std::array<unsigned char, 3>>(x, true);}
  {TRACE; test_not_converts<std::array<unsigned short, 3>>(x, true);}  // NOLINT
  {TRACE; test_not_converts<std::array<unsigned int, 3>>(x, true);}
  {TRACE; test_not_converts<std::array<size_t, 3>>(x, true);}
  {TRACE; test_not_converts<std::array<unsigned long, 3>>(x, true);}  // NOLINT
  {TRACE; test_not_converts<std::array<unsigned long long, 3>>(x,true);}  // NOLINT
  {TRACE; test_not_converts<std::array<int, 2>>(x, true);}
  {TRACE; test_not_converts<std::array<int, 4>>(x, true);}


  // test with skipNonConvertible = false should result in complete failure
  {TRACE; test_not_converts<std::vector<unsigned int>>(x, false);}
  {TRACE; test_not_converts<std::list<unsigned int>>(x, false);}
  {TRACE; test_not_converts<std::set<unsigned int>>(x, false);}
  {TRACE; test_not_converts<std::unordered_set<unsigned int>>(x, false);}

  x.clear();
  x.setSize(2);
  x[0].setSize(2); x[1].setSize(1);
  x[0][0] = 0; x[0][1] = 1; x[1][0] = -1;
  {TRACE; test_converts<std::vector<std::vector<int>>>(x, {{0, 1}, {-1}}, true, false);}
  {TRACE; test_converts<std::vector<std::vector<double>>>(x, {{0.0, 1.0}, {-1.0}}, true, false);}
  {TRACE; test_converts<std::set<std::vector<double>>>(x, {{0.0, 1.0}, {-1.0}}, true, false);}
  {TRACE; test_converts<std::set<std::set<double>>>(x, {{0.0, 1.0}, {-1.0}}, true, false);}
  {TRACE; test_converts<std::vector<std::set<double>>>(x, {{0.0, 1.0}, {-1.0}}, true, false);}
  {TRACE; test_converts<std::vector<std::list<double>>>(x, {{0.0, 1.0}, {-1.0}}, true, false);}
  {TRACE; test_converts<std::array<std::list<double>, 2>>(
    x, {std::list<double>({0.0, 1.0}), std::list<double>({-1.0})}, true, false);}
  {TRACE; test_not_converts<std::list<std::array<double, 2>>>(x, false);}

  x[0][0].clear(); x[0][0].setSize(1); x[0][0][0] = 0;
  x[0][1].clear(); x[0][1].setSize(1); x[0][1][0] = 1;
  x[1][0].clear(); x[1][0].setSize(1); x[1][0][0] = -1;
  {TRACE; test_converts<std::vector<std::vector<std::vector<int>>>>(x, {{{0}, {1}}, {{-1}}}, true, false);}
}

TEST(XmlRpcValueUtils, ConvertMaps)
{
  XmlRpc::XmlRpcValue x;

  x.clear(); x["a"] = true; x["b"] = false;
  {TRACE; test_converts<std::map<std::string, bool>>(x, {{"a", true}, {"b", false}}, true, false);}
  {TRACE; test_not_converts<std::vector<int>>(x);}
  {TRACE; test_not_converts<std::vector<unsigned int>>(x);}
  {TRACE; test_not_converts<std::vector<std::string>>(x);}

  x.clear(); x["a"] = 0; x["b"] = 1; x["c"] = -1;
  {TRACE; test_converts<std::map<std::string, bool>>(x, {{"a", false}, {"b", true}}, true, true);}
  {TRACE; test_converts<std::map<std::string, int>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}
  {TRACE; test_converts<std::map<std::string, char>>(
    x, {{"a", 0}, {"b", 1}, {"c", static_cast<char>(-1)}}, true, false);}
  {TRACE; test_converts<std::map<std::string, short>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::map<std::string, long>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::map<std::string, long long>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::map<std::string, unsigned char>>(x, {{"a", 0}, {"b", 1}}, true, true);}
  {TRACE; test_converts<std::map<std::string, unsigned short>>(x, {{"a", 0}, {"b", 1}}, true, true);}  // NOLINT
  {TRACE; test_converts<std::map<std::string, unsigned int>>(x, {{"a", 0}, {"b", 1}}, true, true);}
  {TRACE; test_converts<std::map<std::string, size_t>>(x, {{"a", 0}, {"b", 1}}, true, true);}
  {TRACE; test_converts<std::map<std::string, unsigned long>>(x, {{"a", 0}, {"b", 1}}, true, true);}  // NOLINT
  {TRACE; test_converts<std::map<std::string, unsigned long long>>(x, {{"a", 0}, {"b", 1}}, true, true);}  // NOLINT
  {TRACE; test_converts<std::map<std::string, float>>(x, {{"a", 0.0f}, {"b", 1.0f}, {"c", -1.0f}}, true, false);}
  {TRACE; test_converts<std::map<std::string, double>>(x, {{"a", 0.0}, {"b", 1.0}, {"c", -1.0}}, true, false);}
  {TRACE; test_converts<std::map<std::string, long double>>(x, {{"a", 0.0}, {"b", 1.0}, {"c", -1.0}}, true, false);}

  {TRACE; test_converts<std::unordered_map<std::string, bool>>(x, {{"a", false}, {"b", true}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, int>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, char>>(
    x, {{"a", 0}, {"b", 1}, {"c", static_cast<char>(-1)}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, short>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, long>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, long long>>(x, {{"a", 0}, {"b", 1}, {"c", -1}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, unsigned char>>(x, {{"a", 0}, {"b", 1}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, unsigned short>>(x, {{"a", 0}, {"b", 1}}, true, true);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, unsigned int>>(x, {{"a", 0}, {"b", 1}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, size_t>>(x, {{"a", 0}, {"b", 1}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, unsigned long>>(x, {{"a", 0}, {"b", 1}}, true, true);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, unsigned long long>>(x, {{"a", 0}, {"b", 1}}, true, true);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, float>>(
    x, {{"a", 0.0f}, {"b", 1.0f}, {"c", -1.0f}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, double>>(
    x, {{"a", 0.0}, {"b", 1.0}, {"c", -1.0}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, long double>>(
    x, {{"a", 0.0}, {"b", 1.0}, {"c", -1.0}}, true, false);}

  // test with skipNonConvertible = false should result in complete failure
  {TRACE; test_not_converts<std::map<std::string, unsigned int>>(x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, unsigned int>>(x, false);}

  x.clear();
  x["a"]["c"] = 0; x["a"]["d"] = 1; x["b"]["e"] = -1;
  {TRACE; test_converts<std::map<std::string, std::map<std::string, int>>>(
    x, {{"a", {{"c", 0}, {"d", 1}}}, {"b", {{"e", -1}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::map<std::string, int>>>(
    x, {{"a", {{"c", 0}, {"d", 1}}}, {"b", {{"e", -1}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::unordered_map<std::string, int>>>(
    x, {{"a", {{"c", 0}, {"d", 1}}}, {"b", {{"e", -1}}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::unordered_map<std::string, int>>>(
    x, {{"a", {{"c", 0}, {"d", 1}}}, {"b", {{"e", -1}}}}, true, false);}
}

TEST(XmlRpcValueUtils, ConvertMapArrays)
{
  XmlRpc::XmlRpcValue x;
  x.clear();
  x["a"].setSize(2);
  x["b"].setSize(1);
  x["a"][0] = 0; x["a"][1] = 1; x["b"][0] = -1;
  {TRACE; test_converts<std::map<std::string, std::vector<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::list<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::set<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::unordered_set<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::vector<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}  // NOLINT
  {TRACE; test_converts<std::unordered_map<std::string, std::list<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::set<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::unordered_set<int>>>(x, {{"a", {0, 1}}, {"b", {-1}}}, true, false);}  // NOLINT

  {TRACE; test_converts<std::map<std::string, std::vector<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::map<std::string, std::list<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::map<std::string, std::set<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::map<std::string, std::unordered_set<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::vector<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::list<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::set<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::unordered_set<unsigned int>>>(x, {{"a", {0, 1}}}, true, true);}  // NOLINT

  {TRACE; test_not_converts<std::map<std::string, std::vector<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::map<std::string, std::list<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::map<std::string, std::set<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::map<std::string, std::unordered_set<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::vector<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::list<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::set<unsigned int>>>(x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::unordered_set<unsigned int>>>(x, false);}

  x.clear();
  x["a"].setSize(2);
  x["a"][0]["b"] = 0; x["a"][0]["c"] = 1; x["a"][1]["d"] = -1;
  {TRACE; test_converts<std::map<std::string, std::vector<std::map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::list<std::map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::set<std::map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::vector<std::map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::list<std::map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::set<std::map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::vector<std::unordered_map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::map<std::string, std::list<std::unordered_map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::vector<std::unordered_map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}
  {TRACE; test_converts<std::unordered_map<std::string, std::list<std::unordered_map<std::string, int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}, {{"d", -1}}}}}, true, false);}

  {TRACE; test_converts<std::map<std::string, std::vector<std::map<std::string, unsigned int>>>>(
      x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::map<std::string, std::list<std::map<std::string, unsigned int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::vector<std::map<std::string, unsigned int>>>>(
      x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::list<std::map<std::string, unsigned int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::map<std::string, std::vector<std::unordered_map<std::string, unsigned int>>>>(
      x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::map<std::string, std::list<std::unordered_map<std::string, unsigned int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::vector<std::unordered_map<std::string, unsigned int>>>>(
      x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}
  {TRACE; test_converts<std::unordered_map<std::string, std::list<std::unordered_map<std::string, unsigned int>>>>(
    x, {{"a", {{{"b", 0}, {"c", 1}}}}}, true, true);}

  {TRACE; test_not_converts<std::map<std::string, std::vector<std::map<std::string, unsigned int>>>>(x, false);}
  {TRACE; test_not_converts<std::map<std::string, std::list<std::map<std::string, unsigned int>>>>(x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::vector<std::map<std::string, unsigned int>>>>(
    x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::list<std::map<std::string, unsigned int>>>>(
    x, false);}
  {TRACE; test_not_converts<std::map<std::string, std::vector<std::unordered_map<std::string, unsigned int>>>>(
    x, false);}
  {TRACE; test_not_converts<std::map<std::string, std::list<std::unordered_map<std::string, unsigned int>>>>(
    x, false);}
  {TRACE; test_not_converts<
    std::unordered_map<std::string, std::vector<std::unordered_map<std::string, unsigned int>>>>(
      x, false);}
  {TRACE; test_not_converts<std::unordered_map<std::string, std::list<std::unordered_map<std::string, unsigned int>>>>(
    x, false);}
}

TEST(XmlRpcValueUtils, ConvertDynamicReconfigureConfig)
{
  XmlRpc::XmlRpcValue x;

  dynamic_reconfigure::Config c;
  {dynamic_reconfigure::BoolParameter p; p.name = "a"; p.value = true; c.bools.push_back(p);}
  {dynamic_reconfigure::IntParameter p; p.name = "b"; p.value = 1; c.ints.push_back(p);}
  {dynamic_reconfigure::DoubleParameter p; p.name = "c"; p.value = 1.0; c.doubles.push_back(p);}
  {dynamic_reconfigure::StrParameter p; p.name = "d"; p.value = "test"; c.strs.push_back(p);}

  x.clear(); x["a"] = true; x["b"] = 1; x["c"] = 1.0; x["d"] = "test";
  {TRACE; test_converts<dynamic_reconfigure::Config>(x, c, true, false);}

  x["wrong"][0] = 1;
  {TRACE; test_converts<dynamic_reconfigure::Config>(x, c, true, true);}
  {TRACE; test_not_converts<dynamic_reconfigure::Config>(x, false);}
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
