/**
 * \file
 * \brief Unit test for xmlrpc_value_traits.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <list>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include <cras_cpp_common/xmlrpc_value_traits.hpp>

using namespace cras;

TEST(XmlRpcValueTraits, Type)
{
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeBoolean, XmlRpcValueTraits<bool>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInt, XmlRpcValueTraits<int>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeDouble, XmlRpcValueTraits<double>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeString, XmlRpcValueTraits<std::string>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeDateTime, XmlRpcValueTraits<tm>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::vector<bool>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::vector<int>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::vector<float>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::vector<double>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::vector<std::string>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeBase64, XmlRpcValueTraits<std::vector<char>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::list<bool>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::list<int>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::list<float>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::list<double>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::list<std::string>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::list<char>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<bool>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<int>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<float>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<double>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<std::string>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<char>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::set<bool>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::unordered_set<int>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::unordered_set<float>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::unordered_set<double>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::unordered_set<std::string>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcValueTraits<std::unordered_set<char>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::map<std::string, bool> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::map<std::string, int> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::map<std::string, float> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::map<std::string, double> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::map<std::string, char*> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::map<std::string, std::string> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::unordered_map<std::string, bool> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::unordered_map<std::string, int> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::unordered_map<std::string, float> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::unordered_map<std::string, double> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcValueTraits<std::unordered_map<std::string, char*> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray,
    XmlRpcValueTraits<std::unordered_set<std::list<std::vector<int>>>>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray,
    (XmlRpcValueTraits<std::unordered_set<std::map<std::string, int>>>::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct,
    (XmlRpcValueTraits<std::unordered_map<std::string, std::string> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInt, XmlRpcValueTraits<size_t>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInt, XmlRpcValueTraits<unsigned int>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInt, XmlRpcValueTraits<char>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeString, XmlRpcValueTraits<char*>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeString, XmlRpcValueTraits<const char*>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeDouble, XmlRpcValueTraits<float>::xmlRpcType);
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::map<bool, std::string> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::map<int, std::string> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::map<double, int> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::unordered_map<bool, std::string> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::unordered_map<int, std::string> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::unordered_map<double, int> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct,
    (XmlRpcValueTraits<std::map<std::string, std::map<std::string, int>> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct,
    (XmlRpcValueTraits<std::map<std::string, std::map<std::string, float>> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct,
    (XmlRpcValueTraits<std::unordered_map<std::string, std::map<std::string, int>> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct,
    (XmlRpcValueTraits<std::map<std::string, std::unordered_map<std::string, float>> >::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::mutex>::xmlRpcType));
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcValueTraits<std::lock_guard<std::mutex>>::xmlRpcType));
}

TEST(XmlRpcValueTraits, String)
{
  EXPECT_STREQ("bool", XmlRpcValueTraits<bool>::stringType);
  EXPECT_STREQ("int", XmlRpcValueTraits<int>::stringType);
  EXPECT_STREQ("double", XmlRpcValueTraits<double>::stringType);
  EXPECT_STREQ("string", XmlRpcValueTraits<std::string>::stringType);
  EXPECT_STREQ("datetime", XmlRpcValueTraits<tm>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::vector<bool>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::vector<int>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::vector<float>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::vector<double>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::vector<std::string>>::stringType);
  EXPECT_STREQ("binary", XmlRpcValueTraits<std::vector<char>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::list<bool>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::list<int>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::list<float>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::list<double>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::list<std::string>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::list<char>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<bool>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<int>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<float>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<double>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<std::string>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<char>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::set<bool>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::unordered_set<int>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::unordered_set<float>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::unordered_set<double>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::unordered_set<std::string>>::stringType);
  EXPECT_STREQ("array", XmlRpcValueTraits<std::unordered_set<char>>::stringType);
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::map<std::string, bool> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::map<std::string, int> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::map<std::string, float> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::map<std::string, double> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::map<std::string, char*> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::map<std::string, std::string> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::unordered_map<std::string, bool> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::unordered_map<std::string, int> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::unordered_map<std::string, float> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::unordered_map<std::string, double> >::stringType));
  EXPECT_STREQ("struct", (XmlRpcValueTraits<std::unordered_map<std::string, char*> >::stringType));
  EXPECT_STREQ("array",
    XmlRpcValueTraits<std::unordered_set<std::list<std::vector<int>>>>::stringType);
  EXPECT_STREQ("array",
    (XmlRpcValueTraits<std::unordered_set<std::map<std::string, int>>>::stringType));
  EXPECT_STREQ("struct",
    (XmlRpcValueTraits<std::unordered_map<std::string, std::string> >::stringType));
  EXPECT_STREQ("int", XmlRpcValueTraits<size_t>::stringType);
  EXPECT_STREQ("int", XmlRpcValueTraits<unsigned int>::stringType);
  EXPECT_STREQ("int", XmlRpcValueTraits<char>::stringType);
  EXPECT_STREQ("string", XmlRpcValueTraits<char*>::stringType);
  EXPECT_STREQ("string", XmlRpcValueTraits<const char*>::stringType);
  EXPECT_STREQ("double", XmlRpcValueTraits<float>::stringType);
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::map<bool, std::string> >::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::map<int, std::string> >::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::map<double, int> >::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::unordered_map<bool, std::string> >::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::unordered_map<int, std::string> >::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::unordered_map<double, int> >::stringType));
  EXPECT_STREQ("struct",
    (XmlRpcValueTraits<std::map<std::string, std::map<std::string, int>> >::stringType));
  EXPECT_STREQ("struct",
    (XmlRpcValueTraits<std::map<std::string, std::map<std::string, float>> >::stringType));
  EXPECT_STREQ("struct",
    (XmlRpcValueTraits<std::unordered_map<std::string, std::map<std::string, int>> >::stringType));
  EXPECT_STREQ("struct",
    (XmlRpcValueTraits<std::map<std::string, std::unordered_map<std::string, float>> >::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::mutex>::stringType));
  EXPECT_STREQ("invalid", (XmlRpcValueTraits<std::lock_guard<std::mutex>>::stringType));
}

TEST(XmlRpcValueTraits, IsCanonical)
{
  EXPECT_TRUE(XmlRpcValueTraits<bool>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<int>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<double>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<std::string>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<tm>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<XmlRpc::XmlRpcValue::BinaryData>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<std::vector<bool>>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<std::vector<int>>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<std::vector<double>>::isCanonical);
  EXPECT_TRUE(XmlRpcValueTraits<std::vector<std::string>>::isCanonical);
  // the following would normally be false, but vector<char> is typedefed as BinaryData, which is canonical
  EXPECT_TRUE(XmlRpcValueTraits<std::vector<char>>::isCanonical);
  EXPECT_TRUE((XmlRpcValueTraits<std::map<std::string, bool>>::isCanonical));
  EXPECT_TRUE((XmlRpcValueTraits<std::map<std::string, int>>::isCanonical));
  EXPECT_TRUE((XmlRpcValueTraits<std::map<std::string, double>>::isCanonical));
  EXPECT_TRUE((XmlRpcValueTraits<std::map<std::string, std::string>>::isCanonical));
  EXPECT_TRUE((XmlRpcValueTraits<std::map<std::string, std::map<std::string, int>>>::isCanonical));

  EXPECT_FALSE(XmlRpcValueTraits<size_t>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<unsigned int>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<char>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<signed char>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<unsigned char>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<char*>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<const char*>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<short>::isCanonical);  // NOLINT
  EXPECT_FALSE(XmlRpcValueTraits<long>::isCanonical);  // NOLINT
  EXPECT_FALSE(XmlRpcValueTraits<float>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<std::vector<float>>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<std::vector<signed char>>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<std::vector<unsigned char>>::isCanonical);
  EXPECT_FALSE(XmlRpcValueTraits<std::vector<char*>>::isCanonical);
  EXPECT_FALSE((XmlRpcValueTraits<std::map<std::string, float>>::isCanonical));
  EXPECT_FALSE((XmlRpcValueTraits<std::map<std::string, char*>>::isCanonical));
  EXPECT_FALSE((XmlRpcValueTraits<std::map<bool, std::string>>::isCanonical));
  EXPECT_FALSE((XmlRpcValueTraits<std::map<int, std::string>>::isCanonical));
  EXPECT_FALSE((XmlRpcValueTraits<std::map<double, int>>::isCanonical));
  EXPECT_FALSE((XmlRpcValueTraits<std::map<std::string, std::map<std::string, float>>>::isCanonical));
}

TEST(XmlRpcValueTraits, ToString)
{
  EXPECT_EQ("bool", to_string(XmlRpc::XmlRpcValue::TypeBoolean));
  EXPECT_EQ("int", to_string(XmlRpc::XmlRpcValue::TypeInt));
  EXPECT_EQ("double", to_string(XmlRpc::XmlRpcValue::TypeDouble));
  EXPECT_EQ("string", to_string(XmlRpc::XmlRpcValue::TypeString));
  EXPECT_EQ("datetime", to_string(XmlRpc::XmlRpcValue::TypeDateTime));
  EXPECT_EQ("binary", to_string(XmlRpc::XmlRpcValue::TypeBase64));
  EXPECT_EQ("struct", to_string(XmlRpc::XmlRpcValue::TypeStruct));
  EXPECT_EQ("array", to_string(XmlRpc::XmlRpcValue::TypeArray));
  EXPECT_EQ("invalid", to_string(XmlRpc::XmlRpcValue::TypeInvalid));
}

TEST(XmlRpcValueTraits, ToCString)
{
  EXPECT_STREQ("bool", to_cstring(XmlRpc::XmlRpcValue::TypeBoolean));
  EXPECT_STREQ("int", to_cstring(XmlRpc::XmlRpcValue::TypeInt));
  EXPECT_STREQ("double", to_cstring(XmlRpc::XmlRpcValue::TypeDouble));
  EXPECT_STREQ("string", to_cstring(XmlRpc::XmlRpcValue::TypeString));
  EXPECT_STREQ("datetime", to_cstring(XmlRpc::XmlRpcValue::TypeDateTime));
  EXPECT_STREQ("binary", to_cstring(XmlRpc::XmlRpcValue::TypeBase64));
  EXPECT_STREQ("struct", to_cstring(XmlRpc::XmlRpcValue::TypeStruct));
  EXPECT_STREQ("array", to_cstring(XmlRpc::XmlRpcValue::TypeArray));
  EXPECT_STREQ("invalid", to_cstring(XmlRpc::XmlRpcValue::TypeInvalid));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}