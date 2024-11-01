/**
 * \file
 * \brief Unit test for string_utils.hpp
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <array>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/MultiArrayLayout.h>

#include <cras_cpp_common/string_utils.hpp>

using namespace cras;

TEST(StringUtils, StripLeadingInplace)  // NOLINT
{
  std::string s;

  s = "test"; stripLeading(s, 'a'); EXPECT_EQ("test", s);
  s = "atest"; stripLeading(s, 'a'); EXPECT_EQ("test", s);
  s = ""; stripLeading(s, 'a'); EXPECT_EQ("", s);
  s = "a"; stripLeading(s, 'a'); EXPECT_EQ("", s);
  s = "aa"; stripLeading(s, 'a'); EXPECT_EQ("a", s);
  s = "atestatesta"; stripLeading(s, 'a'); EXPECT_EQ("testatesta", s);
}

TEST(StringUtils, StripLeading)  // NOLINT
{
  EXPECT_EQ("test", stripLeading("test", 'a'));
  EXPECT_EQ("test", stripLeading("atest", 'a'));
  EXPECT_EQ("", stripLeading("", 'a'));
  EXPECT_EQ("", stripLeading("a", 'a'));
  EXPECT_EQ("a", stripLeading("aa", 'a'));
  EXPECT_EQ("testatesta", stripLeading("atestatesta", 'a'));
}

TEST(StringUtils, StripTrailingInplace)  // NOLINT
{
  std::string s;

  s = "test"; stripTrailing(s, 'a'); EXPECT_EQ("test", s);
  s = "testa"; stripTrailing(s, 'a'); EXPECT_EQ("test", s);
  s = ""; stripTrailing(s, 'a'); EXPECT_EQ("", s);
  s = "a"; stripTrailing(s, 'a'); EXPECT_EQ("", s);
  s = "aa"; stripTrailing(s, 'a'); EXPECT_EQ("a", s);
  s = "atestatesta"; stripTrailing(s, 'a'); EXPECT_EQ("atestatest", s);
}

TEST(StringUtils, StripTrailing)  // NOLINT
{
  EXPECT_EQ("test", stripTrailing("test", 'a'));
  EXPECT_EQ("test", stripTrailing("testa", 'a'));
  EXPECT_EQ("", stripTrailing("", 'a'));
  EXPECT_EQ("", stripTrailing("a", 'a'));
  EXPECT_EQ("a", stripTrailing("aa", 'a'));
  EXPECT_EQ("atestatest", stripTrailing("atestatesta", 'a'));
}

TEST(StringUtils, StripInplace)  // NOLINT
{
  std::string s;

  s = "test"; strip(s, 'a'); EXPECT_EQ("test", s);
  s = "testa"; strip(s, 'a'); EXPECT_EQ("test", s);
  s = ""; strip(s, 'a'); EXPECT_EQ("", s);
  s = "a"; strip(s, 'a'); EXPECT_EQ("", s);
  s = "aa"; strip(s, 'a'); EXPECT_EQ("", s);
  s = "aaa"; strip(s, 'a'); EXPECT_EQ("a", s);
  s = "atestatesta"; strip(s, 'a'); EXPECT_EQ("testatest", s);
  s = "aatestatestaa"; strip(s, 'a'); EXPECT_EQ("atestatesta", s);
}

TEST(StringUtils, Strip)  // NOLINT
{
  EXPECT_EQ("test", strip("test", 'a'));
  EXPECT_EQ("test", strip("testa", 'a'));
  EXPECT_EQ("", strip("", 'a'));
  EXPECT_EQ("", strip("a", 'a'));
  EXPECT_EQ("", strip("aa", 'a'));
  EXPECT_EQ("a", strip("aaa", 'a'));
  EXPECT_EQ("testatest", strip("atestatesta", 'a'));
  EXPECT_EQ("atestatesta", strip("aatestatestaa", 'a'));
}

TEST(StringUtils, StripLeadingSlashInplace)  // NOLINT
{
  std::string s;

  s = "test"; stripLeadingSlash(s); EXPECT_EQ("test", s);
  s = "/test"; stripLeadingSlash(s); EXPECT_EQ("test", s);
  s = ""; stripLeadingSlash(s); EXPECT_EQ("", s);
  s = "/"; stripLeadingSlash(s); EXPECT_EQ("", s);
  s = "//"; stripLeadingSlash(s); EXPECT_EQ("/", s);
  s = "/test/test/"; stripLeadingSlash(s); EXPECT_EQ("test/test/", s);
  s = "/"; stripLeadingSlash(s, true); EXPECT_EQ("", s);
}

TEST(StringUtils, StripLeadingSlash)  // NOLINT
{
  EXPECT_EQ("test", stripLeadingSlash("test"));
  EXPECT_EQ("test", stripLeadingSlash("/test"));
  EXPECT_EQ("", stripLeadingSlash(""));
  EXPECT_EQ("", stripLeadingSlash("/"));
  EXPECT_EQ("/", stripLeadingSlash("//"));
  EXPECT_EQ("test/test/", stripLeadingSlash("/test/test/"));
  EXPECT_EQ("", stripLeadingSlash("/", true));
}

TEST(StringUtils, PrependIfNonempty)  // NOLINT
{
  EXPECT_EQ("/test", prependIfNonEmpty("test", "/"));
  EXPECT_EQ("", prependIfNonEmpty("", "/"));
  EXPECT_EQ("//", prependIfNonEmpty("/", "/"));
  EXPECT_EQ("test/", prependIfNonEmpty("/", "test"));
  EXPECT_EQ("prefixtest", prependIfNonEmpty("test", "prefix"));
}

TEST(StringUtils, AppendIfNonempty)  // NOLINT
{
  EXPECT_EQ("test/", appendIfNonEmpty("test", "/"));
  EXPECT_EQ("", appendIfNonEmpty("", "/"));
  EXPECT_EQ("//", appendIfNonEmpty("/", "/"));
  EXPECT_EQ("/test", appendIfNonEmpty("/", "test"));
  EXPECT_EQ("testpostfix", appendIfNonEmpty("test", "postfix"));
}

TEST(StringUtils, ToStringBasic)  // NOLINT
{
  EXPECT_EQ("5", to_string(5));
  EXPECT_EQ("-5", to_string(-5));
  EXPECT_EQ("5", to_string(5.0));
  EXPECT_EQ("-5", to_string(-5.0));
  EXPECT_EQ("3.14", to_string(3.14));
  EXPECT_EQ("-3.14", to_string(-3.14));
  EXPECT_EQ("False", to_string(false));
  EXPECT_EQ("True", to_string(true));
  EXPECT_EQ("test", to_string(std::string("test")));
  EXPECT_EQ("[1, 2, 3]", to_string(std::vector<int>({1, 2, 3})));
  EXPECT_EQ("[True, False]", to_string(std::vector<bool>({true, false})));
  EXPECT_EQ("[]", to_string(std::vector<bool>()));
  EXPECT_EQ("[\"a\", \"b\"]", to_string(std::vector<std::string>({"a", "b"})));
  EXPECT_EQ("{1, 2, 3}", to_string(std::set<int>({1, 2, 3})));  // set is ordered
  EXPECT_EQ("{False, True}", to_string(std::set<bool>({true, false})));  // set is ordered
  EXPECT_EQ("{}", to_string(std::set<bool>()));  // set is ordered
  EXPECT_EQ("[1, 2, 3]", to_string(std::list<int>({1, 2, 3})));
  EXPECT_EQ("[True, False]", to_string(std::list<bool>({true, false})));
  EXPECT_EQ("[]", to_string(std::list<bool>()));
  EXPECT_EQ("[\"a\"]", to_string(std::list<std::string>({"a"})));
  EXPECT_EQ("[True, False]", to_string(std::array<bool, 2>({true, false})));
  EXPECT_EQ("{1}", to_string(std::unordered_set<int>({1})));
  EXPECT_EQ("{False}", to_string(std::unordered_set<bool>({false})));
  EXPECT_EQ("{}", to_string(std::unordered_set<bool>()));
  EXPECT_EQ("{\"a\": 1, \"b\": 2}", to_string(std::map<std::string, int>({{"a", 1}, {"b", 2}})));  // map is ordered
  EXPECT_EQ("{\"a\": \"1\", \"b\": \"2\"}",
    to_string(std::map<std::string, std::string>({{"a", "1"}, {"b", "2"}})));  // map is ordered
  EXPECT_EQ("{True: False}", to_string(std::map<bool, bool>({{true, false}})));
  EXPECT_EQ("{}", to_string(std::map<std::string, int>()));
  EXPECT_EQ("{\"a\": 1}", to_string(std::unordered_map<std::string, int>({{"a", 1}})));
  EXPECT_EQ("{\"a\": \"1\"}", to_string(std::unordered_map<std::string, std::string>({{"a", "1"}})));
  EXPECT_EQ("{True: False}", to_string(std::unordered_map<bool, bool>({{true, false}})));
  EXPECT_EQ("{}", to_string(std::unordered_map<std::string, int>()));
  EXPECT_EQ("{1: {2: 3}}", to_string(std::map<int, std::unordered_map<int, int>>({{1, {{2, 3}}}})));
  EXPECT_EQ("{1: [2, 3]}", to_string(std::map<int, std::vector<int>>({{1, {2, 3}}})));
  EXPECT_EQ("{1: [2, 3]}", to_string(std::map<int, std::list<int>>({{1, {2, 3}}})));
  EXPECT_EQ("{1: {2, 3}}", to_string(std::map<int, std::set<int>>({{1, {2, 3}}})));
  EXPECT_EQ("{1: {2}}", to_string(std::map<int, std::unordered_set<int>>({{1, {2}}})));
}

TEST(StringUtils, ToStringRos)  // NOLINT
{
  EXPECT_EQ("1.500000000", to_string(ros::Time(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::WallTime(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::SteadyTime(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::Duration(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::WallDuration(1, 500000000)));

  std_msgs::Bool b;
  EXPECT_EQ("data: 0", to_string(b));
  b.data = true;
  EXPECT_EQ("data: 1", to_string(b));

  std_msgs::Header h;
  EXPECT_EQ("seq: 0, stamp: 0.000000000, frame_id: ", to_string(h));
  h.frame_id = "cras";
  EXPECT_EQ("seq: 0, stamp: 0.000000000, frame_id: cras", to_string(h));
  h.stamp.sec = 42;
  h.stamp.nsec = 42;
  EXPECT_EQ("seq: 0, stamp: 42.000000042, frame_id: cras", to_string(h));
  h.seq = 42;
  EXPECT_EQ("seq: 42, stamp: 42.000000042, frame_id: cras", to_string(h));

  std_msgs::MultiArrayLayout m;
  EXPECT_EQ("dim[], data_offset: 0", to_string(m));
  m.dim.resize(1);
  EXPECT_EQ("dim[],   dim[0]: ,     label: ,     size: 0,     stride: 0, data_offset: 0", to_string(m));
  m.dim[0].size = 42;
  m.dim[0].label = "cras";
  m.dim[0].stride = 42;
  EXPECT_EQ("dim[],   dim[0]: ,     label: cras,     size: 42,     stride: 42, data_offset: 0", to_string(m));
  m.dim.resize(2);
  m.dim[1].size = 43;
  m.dim[1].label = "cras2";
  m.dim[1].stride = 44;
  EXPECT_EQ("dim[],   dim[0]: ,     label: cras,     size: 42,     stride: 42,   "
                     "dim[1]: ,     label: cras2,     size: 43,     stride: 44, data_offset: 0", to_string(m));
}

TEST(StringUtils, ToStringEigen)  // NOLINT
{
  EXPECT_EQ("[[1]; [2]; [3]]", to_string(Eigen::Vector3d(1, 2, 3)));
  EXPECT_EQ("[[1]; [2]; [3]; [3.14]]", to_string(Eigen::Vector4d(1, 2, 3, 3.14)));
  EXPECT_EQ("[]", to_string(Eigen::VectorXd(0)));
  Eigen::VectorXd v1(1); v1(0) = 0;
  EXPECT_EQ("[[0]]", to_string(v1));
  Eigen::VectorXd v2(1, 1); v2(0, 0) = 0;
  EXPECT_EQ("[[0]]", to_string(v2));
  Eigen::MatrixXd m1(1, 1); m1(0, 0) = 0;
  EXPECT_EQ("[[0]]", to_string(m1));

  EXPECT_EQ("[[1, 0]; [0, 1]]", replace(to_string(Eigen::Rotation2Dd(0)), "-0", "0"));
  EXPECT_EQ("[[1, 0, 0]; [0, 1, 0]; [0, 0, 1]]", to_string(Eigen::Quaterniond(1, 0, 0, 0)));

  EXPECT_EQ("[[1, 0, 0]; [0, 1, 0]; [0, 0, 1]]", to_string(Eigen::Affine2d::Identity()));
  EXPECT_EQ("[[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0]; [0, 0, 0, 1]]", to_string(Eigen::Isometry3d::Identity()));
}

TEST(StringUtils, ToStringTF2)  // NOLINT
{
  EXPECT_EQ("[1.000000, 2.000000, 3.000000]", to_string(tf2::Vector3(1, 2, 3)));

  EXPECT_EQ("[x=0.000000, y=0.000000, z=0.000000, w=1.000000 (r=0.000, p=0.000, y=0.000)]",
    replace(to_string(tf2::Quaternion(0, 0, 0, 1)), "-0", "0"));

  EXPECT_EQ("[[1.000000, 2.000000, 3.000000]; [4.000000, 5.000000, 6.000000]; [7.000000, 8.000000, 9.000000]]",
    to_string(tf2::Matrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9)));

  EXPECT_EQ("Transform(t=[0.000000, 0.000000, 0.000000], "
            "r=[x=0.000000, y=0.000000, z=0.000000, w=1.000000 (r=0.000, p=0.000, y=0.000)])",
            replace(to_string(tf2::Transform(tf2::Quaternion(0, 0, 0, 1))), "-0", "0"));
}

TEST(StringUtils, ToStringXmlRpcValue)  // NOLINT
{
  EXPECT_EQ("<value><double>2</double></value>", to_string(XmlRpc::XmlRpcValue(2.0)));
  EXPECT_EQ("<value><i4>2</i4></value>", to_string(XmlRpc::XmlRpcValue(2)));
  EXPECT_EQ("<value><boolean>0</boolean></value>", to_string(XmlRpc::XmlRpcValue(false)));
  EXPECT_EQ("<value><boolean>1</boolean></value>", to_string(XmlRpc::XmlRpcValue(true)));
  EXPECT_EQ("<value>aa</value>", to_string(XmlRpc::XmlRpcValue("aa")));

  {
    tm time;
    time.tm_hour = 1;
    time.tm_min = 2;
    time.tm_sec = 3;
    time.tm_mday = 4;
    time.tm_mon = 5;
    time.tm_year = 2006;
    EXPECT_EQ("<value><dateTime.iso8601>20060504T01:02:03</dateTime.iso8601></value>",
      to_string(XmlRpc::XmlRpcValue(&time)));
  }
  {
    int offset = 0;
    EXPECT_EQ("<value><double>2</double></value>",
      to_string(XmlRpc::XmlRpcValue("<value><double>2.0</double></value>", &offset)));
  }
  {
    char bytes[] = "123";
    EXPECT_EQ("<value><base64>MTIz\n</base64></value>", to_string(XmlRpc::XmlRpcValue(bytes, 3)));
  }
  {
    XmlRpc::XmlRpcValue v;
    v[0] = 1;
    v[1] = 2;
    v[2] = 3;
    EXPECT_EQ("<value><array><data>"
                "<value><i4>1</i4></value>"
                "<value><i4>2</i4></value>"
                "<value><i4>3</i4></value>"
              "</data></array></value>", to_string(v));
  }
  {
    XmlRpc::XmlRpcValue v;
    v["0"] = 1;
    v["1"] = 2;
    v["2"] = 3;
    EXPECT_EQ("<value><struct>"
                "<member><name>0</name><value><i4>1</i4></value></member>"
                "<member><name>1</name><value><i4>2</i4></value></member>"
                "<member><name>2</name><value><i4>3</i4></value></member>"
              "</struct></value>", to_string(v));
  }
}

TEST(StringUtils, StartsWith)  // NOLINT
{
  EXPECT_TRUE(startsWith("", ""));
  EXPECT_FALSE(startsWith("", "prefix"));
  EXPECT_FALSE(startsWith("p", "prefix"));
  EXPECT_FALSE(startsWith("pr", "prefix"));
  EXPECT_FALSE(startsWith("pre", "prefix"));
  EXPECT_FALSE(startsWith("pref", "prefix"));
  EXPECT_FALSE(startsWith("prefi", "prefix"));
  EXPECT_TRUE(startsWith("prefix", "prefix"));
  EXPECT_TRUE(startsWith("prefixS", "prefix"));
  EXPECT_TRUE(startsWith("prefixSt", "prefix"));
  EXPECT_TRUE(startsWith("prefixStr", "prefix"));
  EXPECT_TRUE(startsWith("prefixStri", "prefix"));
  EXPECT_TRUE(startsWith("prefixStrin", "prefix"));
  EXPECT_TRUE(startsWith("prefixString", "prefix"));
  EXPECT_FALSE(startsWith("sprefix", "prefix"));
  EXPECT_FALSE(startsWith("stprefix", "prefix"));
  EXPECT_FALSE(startsWith("strprefix", "prefix"));
  EXPECT_FALSE(startsWith("striprefix", "prefix"));
  EXPECT_FALSE(startsWith("strinprefix", "prefix"));
  EXPECT_FALSE(startsWith("stringprefix", "prefix"));
  EXPECT_FALSE(startsWith("string_prefix", "prefix"));
}

TEST(StringUtils, EndsWith)  // NOLINT
{
  EXPECT_TRUE(endsWith("", ""));
  EXPECT_FALSE(endsWith("", "prefix"));
  EXPECT_FALSE(endsWith("p", "prefix"));
  EXPECT_FALSE(endsWith("pr", "prefix"));
  EXPECT_FALSE(endsWith("pre", "prefix"));
  EXPECT_FALSE(endsWith("pref", "prefix"));
  EXPECT_FALSE(endsWith("prefi", "prefix"));
  EXPECT_TRUE(endsWith("prefix", "prefix"));
  EXPECT_FALSE(endsWith("prefixS", "prefix"));
  EXPECT_FALSE(endsWith("prefixSt", "prefix"));
  EXPECT_FALSE(endsWith("prefixStr", "prefix"));
  EXPECT_FALSE(endsWith("prefixStri", "prefix"));
  EXPECT_FALSE(endsWith("prefixStrin", "prefix"));
  EXPECT_FALSE(endsWith("prefixString", "prefix"));
  EXPECT_TRUE(endsWith("sprefix", "prefix"));
  EXPECT_TRUE(endsWith("stprefix", "prefix"));
  EXPECT_TRUE(endsWith("strprefix", "prefix"));
  EXPECT_TRUE(endsWith("striprefix", "prefix"));
  EXPECT_TRUE(endsWith("strinprefix", "prefix"));
  EXPECT_TRUE(endsWith("stringprefix", "prefix"));
  EXPECT_TRUE(endsWith("string_prefix", "prefix"));
}

TEST(StringUtils, ContainsChar)  // NOLINT
{
  EXPECT_FALSE(contains("", 'p'));
  EXPECT_TRUE(contains("p", 'p'));
  EXPECT_TRUE(contains("pr", 'p'));
  EXPECT_TRUE(contains("pre", 'p'));
  EXPECT_TRUE(contains("pref", 'p'));
  EXPECT_TRUE(contains("prefi", 'p'));
  EXPECT_TRUE(contains("prefix", 'p'));
  EXPECT_TRUE(contains("prefixS", 'p'));
  EXPECT_TRUE(contains("prefixSt", 'p'));
  EXPECT_TRUE(contains("prefixStr", 'p'));
  EXPECT_TRUE(contains("prefixStri", 'p'));
  EXPECT_TRUE(contains("prefixStrin", 'p'));
  EXPECT_TRUE(contains("prefixString", 'p'));
  EXPECT_TRUE(contains("sprefix", 'p'));
  EXPECT_TRUE(contains("stprefix", 'p'));
  EXPECT_TRUE(contains("strprefix", 'p'));
  EXPECT_TRUE(contains("striprefix", 'p'));
  EXPECT_TRUE(contains("strinprefix", 'p'));
  EXPECT_TRUE(contains("stringprefix", 'p'));
  EXPECT_TRUE(contains("string_prefix", 'p'));
}

TEST(StringUtils, ContainsString)  // NOLINT
{
  EXPECT_TRUE(contains("", ""));
  EXPECT_FALSE(contains("", "prefix"));
  EXPECT_FALSE(contains("p", "prefix"));
  EXPECT_FALSE(contains("pr", "prefix"));
  EXPECT_FALSE(contains("pre", "prefix"));
  EXPECT_FALSE(contains("pref", "prefix"));
  EXPECT_FALSE(contains("prefi", "prefix"));
  EXPECT_TRUE(contains("prefix", "prefix"));
  EXPECT_TRUE(contains("prefixS", "prefix"));
  EXPECT_TRUE(contains("prefixSt", "prefix"));
  EXPECT_TRUE(contains("prefixStr", "prefix"));
  EXPECT_TRUE(contains("prefixStri", "prefix"));
  EXPECT_TRUE(contains("prefixStrin", "prefix"));
  EXPECT_TRUE(contains("prefixString", "prefix"));
  EXPECT_TRUE(contains("sprefix", "prefix"));
  EXPECT_TRUE(contains("stprefix", "prefix"));
  EXPECT_TRUE(contains("strprefix", "prefix"));
  EXPECT_TRUE(contains("striprefix", "prefix"));
  EXPECT_TRUE(contains("strinprefix", "prefix"));
  EXPECT_TRUE(contains("stringprefix", "prefix"));
  EXPECT_TRUE(contains("string_prefix", "prefix"));
}

TEST(StringUtils, RemovePrefix)  // NOLINT
{
  bool hadPrefix;
  EXPECT_EQ("", removePrefix("", ""));
  EXPECT_EQ("", removePrefix("", "prefix"));
  EXPECT_EQ("p", removePrefix("p", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("pr", removePrefix("pr", "prefix"));
  EXPECT_EQ("pre", removePrefix("pre", "prefix"));
  EXPECT_EQ("pref", removePrefix("pref", "prefix"));
  EXPECT_EQ("prefi", removePrefix("prefi", "prefix"));
  EXPECT_EQ("", removePrefix("prefix", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("S", removePrefix("prefixS", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("St", removePrefix("prefixSt", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("Str", removePrefix("prefixStr", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("Stri", removePrefix("prefixStri", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("Strin", removePrefix("prefixStrin", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("String", removePrefix("prefixString", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("sprefix", removePrefix("sprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("stprefix", removePrefix("stprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("strprefix", removePrefix("strprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("striprefix", removePrefix("striprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("strinprefix", removePrefix("strinprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("stringprefix", removePrefix("stringprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("string_prefix", removePrefix("string_prefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
}

TEST(StringUtils, RemoveSuffix)  // NOLINT
{
  bool hadSuffix;
  EXPECT_EQ("", removeSuffix("", ""));
  EXPECT_EQ("", removeSuffix("", "suffix"));
  EXPECT_EQ("s", removeSuffix("s", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("su", removeSuffix("su", "suffix"));
  EXPECT_EQ("suf", removeSuffix("suf", "suffix"));
  EXPECT_EQ("suff", removeSuffix("suff", "suffix"));
  EXPECT_EQ("suffi", removeSuffix("suffi", "suffix"));
  EXPECT_EQ("", removeSuffix("suffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("suffixS", removeSuffix("suffixS", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixSt", removeSuffix("suffixSt", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixStr", removeSuffix("suffixStr", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixStri", removeSuffix("suffixStri", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixStrin", removeSuffix("suffixStrin", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixString", removeSuffix("suffixString", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("s", removeSuffix("ssuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("st", removeSuffix("stsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("str", removeSuffix("strsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("stri", removeSuffix("strisuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("strin", removeSuffix("strinsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("string", removeSuffix("stringsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("string_", removeSuffix("string_suffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
}

TEST(StringUtils, Replace)  // NOLINT
{
  EXPECT_EQ("test", replace("test", "a", ""));
  EXPECT_EQ("test", replace("atest", "a", ""));
  EXPECT_EQ("", replace("", "a", ""));
  EXPECT_EQ("", replace("a", "a", ""));
  EXPECT_EQ("", replace("aa", "a", ""));
  EXPECT_EQ("testtest", replace("atestatesta", "a", ""));
  EXPECT_EQ("štečřstštestš", replace("čřžtečřstčřžtestčřž", "čřž", "š"));

  EXPECT_EQ("", replace("test", "test", ""));
  EXPECT_EQ("a", replace("atest", "test", ""));
  EXPECT_EQ("", replace("", "test", ""));
  EXPECT_EQ("a", replace("a", "test", ""));
  EXPECT_EQ("aa", replace("aa", "test", ""));
  EXPECT_EQ("aaa", replace("atestatesta", "test", ""));

  EXPECT_EQ("test", replace("test", "a", "longer"));
  EXPECT_EQ("longertest", replace("atest", "a", "longer"));
  EXPECT_EQ("", replace("", "a", "longer"));
  EXPECT_EQ("longer", replace("a", "a", "longer"));
  EXPECT_EQ("longerlonger", replace("aa", "a", "longer"));
  EXPECT_EQ("longertestlongertestlonger", replace("atestatesta", "a", "longer"));

  EXPECT_EQ("ta", replace("tatata", "tata", ""));
  EXPECT_EQ("tata", replace("tatata", "tata", "ta"));  // it is not "recursive"
  EXPECT_EQ("abcd", replace("ababcdcd", "abcd", ""));  // it is not "recursive"

  EXPECT_EQ("test", replace("test", "a", "", cras::ReplacePosition::START));
  EXPECT_EQ("test", replace("atest", "a", "", cras::ReplacePosition::START));
  EXPECT_EQ("test", replace("čřžtest", "čřž", "", cras::ReplacePosition::START));
  EXPECT_EQ("", replace("", "a", "", cras::ReplacePosition::START));
  EXPECT_EQ("", replace("a", "a", "", cras::ReplacePosition::START));
  EXPECT_EQ("", replace("aa", "a", "", cras::ReplacePosition::START));
  EXPECT_EQ("testatesta", replace("atestatesta", "a", "", cras::ReplacePosition::START));
  EXPECT_EQ("cababtestababab", replace("abababtestababab", "ab", "c", cras::ReplacePosition::START));

  EXPECT_EQ("test", replace("test", "a", "", cras::ReplacePosition::END));
  EXPECT_EQ("test", replace("testa", "a", "", cras::ReplacePosition::END));
  EXPECT_EQ("test", replace("testčřž", "čřž", "", cras::ReplacePosition::END));
  EXPECT_EQ("", replace("", "a", "", cras::ReplacePosition::END));
  EXPECT_EQ("", replace("a", "a", "", cras::ReplacePosition::END));
  EXPECT_EQ("a", replace("aa", "a", "", cras::ReplacePosition::END));
  EXPECT_EQ("atestatest", replace("atestatesta", "a", "", cras::ReplacePosition::END));
  EXPECT_EQ("abababtestababc", replace("abababtestababab", "ab", "c", cras::ReplacePosition::END));
}

TEST(StringUtils, ReplaceInplace)  // NOLINT
{
  std::string s;

  s = "test"; replace(s, "a", ""); EXPECT_EQ("test", s);
  s = "atest"; replace(s, "a", ""); EXPECT_EQ("test", s);
  s = ""; replace(s, "a", ""); EXPECT_EQ("", s);
  s = "a"; replace(s, "a", ""); EXPECT_EQ("", s);
  s = "aa"; replace(s, "a", ""); EXPECT_EQ("", s);
  s = "atestatesta"; replace(s, "a", ""); EXPECT_EQ("testtest", s);
  s = "test"; replace(s, "test", ""); EXPECT_EQ("", s);
  s = "atest"; replace(s, "test", ""); EXPECT_EQ("a", s);
  s = ""; replace(s, "test", ""); EXPECT_EQ("", s);
  s = "a"; replace(s, "test", ""); EXPECT_EQ("a", s);
  s = "aa"; replace(s, "test", ""); EXPECT_EQ("aa", s);
  s = "atestatesta"; replace(s, "test", ""); EXPECT_EQ("aaa", s);
  s = "test"; replace(s, "a", "longer"); EXPECT_EQ("test", s);
  s = "atest"; replace(s, "a", "longer"); EXPECT_EQ("longertest", s);
  s = ""; replace(s, "a", "longer"); EXPECT_EQ("", s);
  s = "a"; replace(s, "a", "longer"); EXPECT_EQ("longer", s);
  s = "aa"; replace(s, "a", "longer"); EXPECT_EQ("longerlonger", s);
  s = "atestatesta"; replace(s, "a", "longer"); EXPECT_EQ("longertestlongertestlonger", s);
  s = "tatata"; replace(s, "tata", ""); EXPECT_EQ("ta", s);
  s = "tatata"; replace(s, "tata", "ta"); EXPECT_EQ("tata", s);  // it is not "recursive"
  s = "ababcdcd"; replace(s, "abcd", ""); EXPECT_EQ("abcd", s);  // it is not "recursive"
}

TEST(StringUtils, Split)  // NOLINT
{
  using v = std::vector<std::string>;

  EXPECT_EQ(v({""}), split("", "t"));
  EXPECT_EQ(v({"test"}), split("test", "a"));
  EXPECT_EQ(v({"", "es", ""}), split("test", "t"));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t"));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t", -1));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t", 2));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t", 100));
  EXPECT_EQ(v({"a", "esta"}), split("atesta", "t", 1));
  EXPECT_EQ(v({"atesta"}), split("atesta", "t", 0));

  EXPECT_EQ(v({""}), split("", "te"));
  EXPECT_EQ(v({"test"}), split("test", "ae"));
  EXPECT_EQ(v({"", "st"}), split("test", "te"));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te"));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", -1));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", 2));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", 100));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", 1));
  EXPECT_EQ(v({"atesta"}), split("atesta", "te", 0));

  EXPECT_EQ(v({"", "home", "cras", "file", "path"}), split("/home/cras/file/path", "/"));
}

TEST(StringUtils, ToUpper)  // NOLINT
{
  EXPECT_EQ("TEST", cras::toUpper("test"));
  EXPECT_EQ("TEST", cras::toUpper("Test"));
  EXPECT_EQ("TEST", cras::toUpper("TEST"));
  EXPECT_EQ("TEST", cras::toUpper("TeST"));
  EXPECT_EQ("", cras::toUpper(""));
  EXPECT_EQ("1234567890", cras::toUpper("1234567890"));
//  EXPECT_EQ("ĚŠČŘŽÝÁÍÉĎŤŇÚŮ", cras::toUpper("ěščřžýáíéďťňúů"));  // not yet working
}

TEST(StringUtils, ToLower)  // NOLINT
{
  EXPECT_EQ("test", cras::toLower("test"));
  EXPECT_EQ("test", cras::toLower("Test"));
  EXPECT_EQ("test", cras::toLower("TEST"));
  EXPECT_EQ("test", cras::toLower("TeST"));
  EXPECT_EQ("", cras::toUpper(""));
  EXPECT_EQ("1234567890", cras::toUpper("1234567890"));
//  EXPECT_EQ("ěščřžýáíéďťňúů", cras::toLower("ĚŠČŘŽÝÁÍÉĎŤŇÚŮ"));  // not yet working
}

void test_va_format(const std::string& res, const char* format, ...)
{
  va_list(args);
  va_start(args, format);
  EXPECT_EQ(res, cras::format(format, args));
  va_end(args);
}

TEST(StringUtils, FormatVaList)  // NOLINT
{
  {SCOPED_TRACE("1"); test_va_format("cras", "%s", "cras"); }
  {SCOPED_TRACE("2"); test_va_format("-42", "%i", -42); }
  {SCOPED_TRACE("3"); test_va_format("42", "%u", 42); }
  {SCOPED_TRACE("4"); test_va_format("42", "%g", 42.0); }
  {SCOPED_TRACE("5"); test_va_format("42.000000", "%f", 42.0); }
  {SCOPED_TRACE("6"); test_va_format("3.14", "%g", 3.14); }
  {SCOPED_TRACE("7"); test_va_format("cras -42 42 3.14", "%s %i %u %g", "cras", -42, 42, 3.14); }

  std::string longString(300000, '*');  // generates a string of length 300.000 asterisks
  {SCOPED_TRACE("8"); test_va_format(longString, "%s", longString.c_str()); }
}

TEST(StringUtils, FormatCharPtr)  // NOLINT
{
  EXPECT_EQ("cras", cras::format("%s", "cras"));
  EXPECT_EQ("-42", cras::format("%i", -42));
  EXPECT_EQ("42", cras::format("%u", 42));
  EXPECT_EQ("42", cras::format("%g", 42.0));
  EXPECT_EQ("42.000000", cras::format("%f", 42.0));
  EXPECT_EQ("3.14", cras::format("%g", 3.14));
  EXPECT_EQ("cras -42 42 3.14", cras::format("%s %i %u %g", "cras", -42, 42, 3.14));

  std::string longString(300000, '*');  // generates a string of length 300.000 asterisks
  EXPECT_EQ(longString, cras::format("%s", longString.c_str()));
}

TEST(StringUtils, FormatString)  // NOLINT
{
  using s = std::string;

  EXPECT_EQ("cras", cras::format(s("%s"), "cras"));
  EXPECT_EQ("-42", cras::format(s("%i"), -42));
  EXPECT_EQ("42", cras::format(s("%u"), 42));
  EXPECT_EQ("42", cras::format(s("%g"), 42.0));
  EXPECT_EQ("42.000000", cras::format(s("%f"), 42.0));
  EXPECT_EQ("3.14", cras::format(s("%g"), 3.14));
  EXPECT_EQ("cras -42 42 3.14", cras::format(s("%s %i %u %g"), "cras", -42, 42, 3.14));

  std::string longString(300000, '*');  // generates a string of length 300.000 asterisks
  EXPECT_EQ(longString, cras::format(s("%s"), longString.c_str()));
}

TEST(StringUtils, QuoteIfStringType)  // NOLINT
{
  const std::string s("a");
  EXPECT_EQ("\"cras\"", cras::quoteIfStringType("cras", "a"));
  EXPECT_EQ("\"cras\"", cras::quoteIfStringType("cras", s));
  EXPECT_EQ("\"cras\"", cras::quoteIfStringType("cras", std::string("a")));
  EXPECT_EQ("\"cras\"", cras::quoteIfStringType("cras", new char));
  EXPECT_EQ("cras", cras::quoteIfStringType("cras", 1));
  EXPECT_EQ("cras", cras::quoteIfStringType("cras", 1.0));
  EXPECT_EQ("cras", cras::quoteIfStringType("cras", false));
  EXPECT_EQ("cras", cras::quoteIfStringType("cras", ros::DURATION_MAX));
}

TEST(StringUtils, Join)  // NOLINT
{
  EXPECT_EQ("1,2,3", cras::join(std::vector<int>({1, 2, 3}), ","));
  EXPECT_EQ("1, 2, 3", cras::join(std::vector<int>({1, 2, 3}), ", "));
  EXPECT_EQ("", cras::join(std::vector<int>(), ","));
  EXPECT_EQ("1", cras::join(std::vector<int>({1}), ","));
  EXPECT_EQ("[1, 2],[3],[4, 5]", cras::join(std::vector<std::vector<int>>({{1, 2}, {3}, {4, 5}}), ","));
  EXPECT_EQ("1,2,3", cras::join(std::list<int>({1, 2, 3}), ","));
  EXPECT_EQ("1,2,3", cras::join(std::set<int>({1, 2, 3}), ","));
  EXPECT_EQ("abc-def-ghi", cras::join(std::vector<std::string>({"abc", "def", "ghi"}), "-"));
  EXPECT_EQ("0.000000000;1.000000000", cras::join(std::vector<ros::Duration>({{0, 0}, {1, 0}}), ";"));
}

TEST(StringUtils, ParseInt8)  // NOLINT
{
  EXPECT_EQ(0, cras::parseInt8("0"));
  EXPECT_EQ(0, cras::parseInt8("0", 2));
  EXPECT_EQ(0, cras::parseInt8("0", 8));
  EXPECT_EQ(0, cras::parseInt8("0", 10));
  EXPECT_EQ(0, cras::parseInt8("0", 16));

  EXPECT_EQ(1, cras::parseInt8("1"));
  EXPECT_EQ(1, cras::parseInt8("+1"));
  EXPECT_EQ(1, cras::parseInt8(" 1"));
  EXPECT_EQ(1, cras::parseInt8(" +1"));
  EXPECT_EQ(1, cras::parseInt8(" +1 "));
  EXPECT_EQ(1, cras::parseInt8("+1 "));
  EXPECT_EQ(1, cras::parseInt8("1 "));
  EXPECT_EQ(1, cras::parseInt8(" 1  "));
  EXPECT_EQ(10, cras::parseInt8("10"));
  EXPECT_EQ(42, cras::parseInt8("42"));
  EXPECT_EQ(127, cras::parseInt8("127"));
  EXPECT_EQ(127, cras::parseInt8("127", 10));
  EXPECT_EQ(0b0000'0001, cras::parseInt8("0b00000001"));
  EXPECT_EQ(0b0000'0001, cras::parseInt8("0B00000001"));
  EXPECT_EQ(0b0111'1111, cras::parseInt8("0b01111111"));
  EXPECT_EQ(0b0111'1111, cras::parseInt8("0B01111111"));
  EXPECT_EQ(0b0111'1111, cras::parseInt8("01111111", 2));
  EXPECT_EQ(01, cras::parseInt8("01"));
  EXPECT_EQ(0001, cras::parseInt8("0001"));
  EXPECT_EQ(0177, cras::parseInt8("0177"));
  EXPECT_EQ(0177, cras::parseInt8("0177", 8));
  EXPECT_EQ(0177, cras::parseInt8("177", 8));
  EXPECT_EQ(0x01, cras::parseInt8("0x01"));
  EXPECT_EQ(0x01, cras::parseInt8("0X01"));
  EXPECT_EQ(0x7f, cras::parseInt8("0x7f"));
  EXPECT_EQ(0x7f, cras::parseInt8("0x7F"));
  EXPECT_EQ(0x7f, cras::parseInt8("0X7f"));
  EXPECT_EQ(0x7f, cras::parseInt8("0X7F"));
  EXPECT_EQ(0x7f, cras::parseInt8("7F", 16));

  EXPECT_EQ(-1, cras::parseInt8("-1"));
  EXPECT_EQ(-1, cras::parseInt8("-1"));
  EXPECT_EQ(-1, cras::parseInt8(" -1"));
  EXPECT_EQ(-1, cras::parseInt8(" -1 "));
  EXPECT_EQ(-1, cras::parseInt8("-1 "));
  EXPECT_EQ(-10, cras::parseInt8("-10"));
  EXPECT_EQ(-42, cras::parseInt8("-42"));
  EXPECT_EQ(-128, cras::parseInt8("-128"));
  EXPECT_EQ(-128, cras::parseInt8("-128", 10));
  EXPECT_EQ(-0b0000'0001, cras::parseInt8("-0b00000001"));
  EXPECT_EQ(-0b0000'0001, cras::parseInt8("-0B00000001"));
  EXPECT_EQ(-0b1000'0000, cras::parseInt8("-0b10000000"));
  EXPECT_EQ(-0b1000'0000, cras::parseInt8("-0B10000000"));
  EXPECT_EQ(-0b1000'0000, cras::parseInt8("-10000000", 2));
  EXPECT_EQ(-01, cras::parseInt8("-01"));
  EXPECT_EQ(-0001, cras::parseInt8("-0001"));
  EXPECT_EQ(-0200, cras::parseInt8("-0200"));
  EXPECT_EQ(-0200, cras::parseInt8("-0200", 8));
  EXPECT_EQ(-0x01, cras::parseInt8("-0x01"));
  EXPECT_EQ(-0x01, cras::parseInt8("-0X01"));
  EXPECT_EQ(-0x80, cras::parseInt8("-0x80"));
  EXPECT_EQ(-0x80, cras::parseInt8("-0X80"));
  EXPECT_EQ(-0x80, cras::parseInt8("-80", 16));

  EXPECT_THROW(cras::parseInt8("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("128"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("-129"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("0b100000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("-0b100000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("0B01111111", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("-0700"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("0700"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("-0700"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("0XFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("-0XFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("0X7F", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseInt8("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseUInt8)  // NOLINT
{
  EXPECT_EQ(0, cras::parseUInt8("0"));

  EXPECT_EQ(1, cras::parseUInt8("1"));
  EXPECT_EQ(1, cras::parseUInt8("+1"));
  EXPECT_EQ(1, cras::parseUInt8(" 1"));
  EXPECT_EQ(1, cras::parseUInt8(" +1"));
  EXPECT_EQ(1, cras::parseUInt8(" +1 "));
  EXPECT_EQ(1, cras::parseUInt8("+1 "));
  EXPECT_EQ(1, cras::parseUInt8("1 "));
  EXPECT_EQ(1, cras::parseUInt8(" 1  "));
  EXPECT_EQ(10, cras::parseUInt8("10"));
  EXPECT_EQ(42, cras::parseUInt8("42"));
  EXPECT_EQ(127, cras::parseUInt8("127"));
  EXPECT_EQ(128, cras::parseUInt8("128"));
  EXPECT_EQ(255, cras::parseUInt8("255"));
  EXPECT_EQ(255, cras::parseUInt8("255", 10));
  EXPECT_EQ(0b0000'0001, cras::parseUInt8("0b00000001"));
  EXPECT_EQ(0b0000'0001, cras::parseUInt8("0B00000001"));
  EXPECT_EQ(0b0111'1111, cras::parseUInt8("0b01111111"));
  EXPECT_EQ(0b0111'1111, cras::parseUInt8("0B01111111"));
  EXPECT_EQ(0b0111'1111, cras::parseUInt8("01111111", 2));
  EXPECT_EQ(01, cras::parseUInt8("01"));
  EXPECT_EQ(0001, cras::parseUInt8("0001"));
  EXPECT_EQ(0377, cras::parseUInt8("0377"));
  EXPECT_EQ(0377, cras::parseUInt8("0377", 8));
  EXPECT_EQ(0377, cras::parseUInt8("377", 8));
  EXPECT_EQ(0x01, cras::parseUInt8("0x01"));
  EXPECT_EQ(0x01, cras::parseUInt8("0X01"));
  EXPECT_EQ(0xff, cras::parseUInt8("0xff"));
  EXPECT_EQ(0xff, cras::parseUInt8("0xFF"));
  EXPECT_EQ(0xff, cras::parseUInt8("0xFf"));
  EXPECT_EQ(0xff, cras::parseUInt8("0xfF"));
  EXPECT_EQ(0xff, cras::parseUInt8("0Xff"));
  EXPECT_EQ(0xff, cras::parseUInt8("0XFF"));
  EXPECT_EQ(0xff, cras::parseUInt8("0XFf"));
  EXPECT_EQ(0xff, cras::parseUInt8("0XfF"));
  EXPECT_EQ(0xff, cras::parseUInt8("fF", 16));

  EXPECT_THROW(cras::parseUInt8("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8(" -1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8(" -1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-10"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-42"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-128"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-128", 10), std::invalid_argument);

  EXPECT_THROW(cras::parseUInt8("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("256"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-129"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("0b100000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-0b100000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("0B01111111", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-0700"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("0700"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-0700"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("0X100"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("-0X100"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("0XFF", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt8("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseInt16)  // NOLINT
{
  EXPECT_EQ(0, cras::parseInt16("0"));
  EXPECT_EQ(0, cras::parseInt16("0", 2));
  EXPECT_EQ(0, cras::parseInt16("0", 8));
  EXPECT_EQ(0, cras::parseInt16("0", 10));
  EXPECT_EQ(0, cras::parseInt16("0", 16));

  EXPECT_EQ(1, cras::parseInt16("1"));
  EXPECT_EQ(1, cras::parseInt16("+1"));
  EXPECT_EQ(1, cras::parseInt16(" 1"));
  EXPECT_EQ(1, cras::parseInt16(" +1"));
  EXPECT_EQ(1, cras::parseInt16(" +1 "));
  EXPECT_EQ(1, cras::parseInt16("+1 "));
  EXPECT_EQ(1, cras::parseInt16("1 "));
  EXPECT_EQ(1, cras::parseInt16(" 1  "));
  EXPECT_EQ(10, cras::parseInt16("10"));
  EXPECT_EQ(42, cras::parseInt16("42"));
  EXPECT_EQ(32767, cras::parseInt16("32767"));
  EXPECT_EQ(32767, cras::parseInt16("32767", 10));
  EXPECT_EQ(0b0000'0000'0000'0001, cras::parseInt16("0b0000000000000001"));
  EXPECT_EQ(0b0000'0000'0000'0001, cras::parseInt16("0B0000000000000001"));
  EXPECT_EQ(0b0111'1111'1111'1111, cras::parseInt16("0b0111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111, cras::parseInt16("0B0111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111, cras::parseInt16("0111111111111111", 2));
  EXPECT_EQ(01, cras::parseInt16("01"));
  EXPECT_EQ(0000001, cras::parseInt16("0000001"));
  EXPECT_EQ(0077777, cras::parseInt16("0077777"));
  EXPECT_EQ(0077777, cras::parseInt16("0077777", 8));
  EXPECT_EQ(0077777, cras::parseInt16("77777", 8));
  EXPECT_EQ(0x0001, cras::parseInt16("0x0001"));
  EXPECT_EQ(0x0001, cras::parseInt16("0X0001"));
  EXPECT_EQ(0x7fff, cras::parseInt16("0x7fff"));
  EXPECT_EQ(0x7fff, cras::parseInt16("0x7Fff"));
  EXPECT_EQ(0x7fff, cras::parseInt16("0X7fff"));
  EXPECT_EQ(0x7fff, cras::parseInt16("0X7Fff"));
  EXPECT_EQ(0x7fff, cras::parseInt16("7Fff", 16));

  EXPECT_EQ(-1, cras::parseInt16("-1"));
  EXPECT_EQ(-1, cras::parseInt16("-1"));
  EXPECT_EQ(-1, cras::parseInt16(" -1"));
  EXPECT_EQ(-1, cras::parseInt16(" -1 "));
  EXPECT_EQ(-1, cras::parseInt16("-1 "));
  EXPECT_EQ(-10, cras::parseInt16("-10"));
  EXPECT_EQ(-42, cras::parseInt16("-42"));
  EXPECT_EQ(-32768, cras::parseInt16("-32768"));
  EXPECT_EQ(-32768, cras::parseInt16("-32768", 10));
  EXPECT_EQ(-0b0000'0000'0000'0001, cras::parseInt16("-0b0000000000000001"));
  EXPECT_EQ(-0b0000'0000'0000'0001, cras::parseInt16("-0B0000000000000001"));
  EXPECT_EQ(-0b1000'0000'0000'0000, cras::parseInt16("-0b1000000000000000"));
  EXPECT_EQ(-0b1000'0000'0000'0000, cras::parseInt16("-0B1000000000000000"));
  EXPECT_EQ(-0b1000'0000'0000'0000, cras::parseInt16("-1000000000000000", 2));
  EXPECT_EQ(-01, cras::parseInt16("-01"));
  EXPECT_EQ(-000001, cras::parseInt16("-000001"));
  EXPECT_EQ(-0100000, cras::parseInt16("-0100000"));
  EXPECT_EQ(-0100000, cras::parseInt16("-0100000", 8));
  EXPECT_EQ(-0100000, cras::parseInt16("-100000", 8));
  EXPECT_EQ(-0x0001, cras::parseInt16("-0x0001"));
  EXPECT_EQ(-0x0001, cras::parseInt16("-0X0001"));
  EXPECT_EQ(-0x8000, cras::parseInt16("-0x8000"));
  EXPECT_EQ(-0x8000, cras::parseInt16("-0X8000"));
  EXPECT_EQ(-0x8000, cras::parseInt16("-8000", 16));

  EXPECT_THROW(cras::parseInt16("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("32768"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("-32769"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("0b1000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("-0b1000000000000001"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("0B0111111111111111", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("0100000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("-0100001"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("0XFFFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("-0XFFFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("0XFFFF", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseInt16("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseUInt16)  // NOLINT
{
  EXPECT_EQ(0, cras::parseUInt16("0"));
  EXPECT_EQ(0, cras::parseUInt16("0", 2));
  EXPECT_EQ(0, cras::parseUInt16("0", 8));
  EXPECT_EQ(0, cras::parseUInt16("0", 10));
  EXPECT_EQ(0, cras::parseUInt16("0", 16));

  EXPECT_EQ(1, cras::parseUInt16("1"));
  EXPECT_EQ(1, cras::parseUInt16("+1"));
  EXPECT_EQ(1, cras::parseUInt16(" 1"));
  EXPECT_EQ(1, cras::parseUInt16(" +1"));
  EXPECT_EQ(1, cras::parseUInt16(" +1 "));
  EXPECT_EQ(1, cras::parseUInt16("+1 "));
  EXPECT_EQ(1, cras::parseUInt16("1 "));
  EXPECT_EQ(1, cras::parseUInt16(" 1  "));
  EXPECT_EQ(10, cras::parseUInt16("10"));
  EXPECT_EQ(42, cras::parseUInt16("42"));
  EXPECT_EQ(32767, cras::parseUInt16("32767"));
  EXPECT_EQ(32768u, cras::parseUInt16("32768"));
  EXPECT_EQ(65535u, cras::parseUInt16("65535"));
  EXPECT_EQ(65535u, cras::parseUInt16("65535", 10));
  EXPECT_EQ(0b0000'0000'0000'0001, cras::parseUInt16("0b0000000000000001"));
  EXPECT_EQ(0b0000'0000'0000'0001, cras::parseUInt16("0B0000000000000001"));
  EXPECT_EQ(0b0111'1111'1111'1111, cras::parseUInt16("0b0111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111, cras::parseUInt16("0B0111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111, cras::parseUInt16("0111111111111111", 2));
  EXPECT_EQ(01, cras::parseUInt16("01"));
  EXPECT_EQ(0001, cras::parseUInt16("0001"));
  EXPECT_EQ(0177777, cras::parseUInt16("0177777"));
  EXPECT_EQ(0177777, cras::parseUInt16("0177777", 8));
  EXPECT_EQ(0177777, cras::parseUInt16("177777", 8));
  EXPECT_EQ(0x0001, cras::parseUInt16("0x0001"));
  EXPECT_EQ(0x0001, cras::parseUInt16("0X0001"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0xffff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0xFFff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0xFfff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0xfFff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0Xffff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0XFFff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0XFfff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("0XfFff"));
  EXPECT_EQ(0xffff, cras::parseUInt16("fFff", 16));

  EXPECT_THROW(cras::parseUInt16("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16(" -1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16(" -1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-10"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-42"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-32768"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-32768", 10), std::invalid_argument);

  EXPECT_THROW(cras::parseUInt16("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("65536"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-32769"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("0b10000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-0b10000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("0B0111111111111111", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("0200000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-0200000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("0X10000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("-0X10000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("0XFFFF", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt16("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseInt32)  // NOLINT
{
  EXPECT_EQ(0, cras::parseInt32("0"));
  EXPECT_EQ(0, cras::parseInt32("0", 2));
  EXPECT_EQ(0, cras::parseInt32("0", 8));
  EXPECT_EQ(0, cras::parseInt32("0", 10));
  EXPECT_EQ(0, cras::parseInt32("0", 16));

  EXPECT_EQ(1, cras::parseInt32("1"));
  EXPECT_EQ(1, cras::parseInt32("+1"));
  EXPECT_EQ(1, cras::parseInt32(" 1"));
  EXPECT_EQ(1, cras::parseInt32(" +1"));
  EXPECT_EQ(1, cras::parseInt32(" +1 "));
  EXPECT_EQ(1, cras::parseInt32("+1 "));
  EXPECT_EQ(1, cras::parseInt32("1 "));
  EXPECT_EQ(1, cras::parseInt32(" 1  "));
  EXPECT_EQ(10, cras::parseInt32("10"));
  EXPECT_EQ(42, cras::parseInt32("42"));
  EXPECT_EQ(2147483647, cras::parseInt32("2147483647"));
  EXPECT_EQ(2147483647, cras::parseInt32("2147483647", 10));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0001, cras::parseInt32("0b00000000000000000000000000000001"));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0001, cras::parseInt32("0B00000000000000000000000000000001"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111, cras::parseInt32("0b01111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111, cras::parseInt32("0B01111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111, cras::parseInt32("01111111111111111111111111111111", 2));
  EXPECT_EQ(01, cras::parseInt32("01"));
  EXPECT_EQ(000000000001, cras::parseInt32("000000000001"));
  EXPECT_EQ(007777777777, cras::parseInt32("007777777777"));
  EXPECT_EQ(007777777777, cras::parseInt32("007777777777", 8));
  EXPECT_EQ(007777777777, cras::parseInt32("7777777777", 8));
  EXPECT_EQ(0x00000001, cras::parseInt32("0x00000001"));
  EXPECT_EQ(0x00000001, cras::parseInt32("0X00000001"));
  EXPECT_EQ(0x7fffffff, cras::parseInt32("0x7fffffff"));
  EXPECT_EQ(0x7fffffff, cras::parseInt32("0x7Fffffff"));
  EXPECT_EQ(0x7fffffff, cras::parseInt32("0X7fffffff"));
  EXPECT_EQ(0x7fffffff, cras::parseInt32("0X7Fffffff"));
  EXPECT_EQ(0x7fffffff, cras::parseInt32("7Fffffff", 16));

  EXPECT_EQ(-1, cras::parseInt32("-1"));
  EXPECT_EQ(-1, cras::parseInt32("-1"));
  EXPECT_EQ(-1, cras::parseInt32(" -1"));
  EXPECT_EQ(-1, cras::parseInt32(" -1 "));
  EXPECT_EQ(-1, cras::parseInt32("-1 "));
  EXPECT_EQ(-10, cras::parseInt32("-10"));
  EXPECT_EQ(-42, cras::parseInt32("-42"));
  EXPECT_EQ(-2147483648, cras::parseInt32("-2147483648"));
  EXPECT_EQ(-2147483648, cras::parseInt32("-2147483648", 10));
  EXPECT_EQ(-0b0000'0000'0000'0000'0000'0000'0000'0001, cras::parseInt32("-0b00000000000000000000000000000001"));
  EXPECT_EQ(-0b0000'0000'0000'0000'0000'0000'0000'0001, cras::parseInt32("-0B00000000000000000000000000000001"));
  EXPECT_EQ(-0b1000'0000'0000'0000'0000'0000'0000'0000, cras::parseInt32("-0b10000000000000000000000000000000"));
  EXPECT_EQ(-0b1000'0000'0000'0000'0000'0000'0000'0000, cras::parseInt32("-0B10000000000000000000000000000000"));
  EXPECT_EQ(-0b1000'0000'0000'0000'0000'0000'0000'0000, cras::parseInt32("-10000000000000000000000000000000", 2));
  EXPECT_EQ(-01, cras::parseInt32("-01"));
  EXPECT_EQ(-0000000001, cras::parseInt32("-0000000001"));
  EXPECT_EQ(-010000000000, cras::parseInt32("-010000000000"));
  EXPECT_EQ(-010000000000, cras::parseInt32("-010000000000", 8));
  EXPECT_EQ(-010000000000, cras::parseInt32("-10000000000", 8));
  EXPECT_EQ(-0x00000001, cras::parseInt32("-0x00000001"));
  EXPECT_EQ(-0x00000001, cras::parseInt32("-0X00000001"));
  EXPECT_EQ(-0x80000000, cras::parseInt32("-0x80000000"));
  EXPECT_EQ(-0x80000000, cras::parseInt32("-0X80000000"));
  EXPECT_EQ(-0x80000000, cras::parseInt32("-80000000", 16));

  EXPECT_THROW(cras::parseInt32("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("2147483648"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("-2147483649"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("0b100000000000000000000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("-0b100000000000000000000000000000001"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("0B01111111111111111111111111111111", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("020000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("-020000000001"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("0XFFFFFFFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("-0XFFFFFFFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("0X7Fffffff", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseInt32("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseUInt32)  // NOLINT
{
  EXPECT_EQ(0, cras::parseUInt32("0"));
  EXPECT_EQ(0, cras::parseUInt32("0", 2));
  EXPECT_EQ(0, cras::parseUInt32("0", 8));
  EXPECT_EQ(0, cras::parseUInt32("0", 10));
  EXPECT_EQ(0, cras::parseUInt32("0", 16));

  EXPECT_EQ(1, cras::parseUInt32("1"));
  EXPECT_EQ(1, cras::parseUInt32("+1"));
  EXPECT_EQ(1, cras::parseUInt32(" 1"));
  EXPECT_EQ(1, cras::parseUInt32(" +1"));
  EXPECT_EQ(1, cras::parseUInt32(" +1 "));
  EXPECT_EQ(1, cras::parseUInt32("+1 "));
  EXPECT_EQ(1, cras::parseUInt32("1 "));
  EXPECT_EQ(1, cras::parseUInt32(" 1  "));
  EXPECT_EQ(10, cras::parseUInt32("10"));
  EXPECT_EQ(42, cras::parseUInt32("42"));
  EXPECT_EQ(2147483647, cras::parseUInt32("2147483647"));
  EXPECT_EQ(2147483648u, cras::parseUInt32("2147483648"));
  EXPECT_EQ(4294967295u, cras::parseUInt32("4294967295"));
  EXPECT_EQ(4294967295u, cras::parseUInt32("4294967295", 10));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0001, cras::parseUInt32("0b00000000000000000000000000000001"));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0001, cras::parseUInt32("0B00000000000000000000000000000001"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111, cras::parseUInt32("0b01111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111, cras::parseUInt32("0B01111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111, cras::parseUInt32("01111111111111111111111111111111", 2));
  EXPECT_EQ(01, cras::parseUInt32("01"));
  EXPECT_EQ(000000000001, cras::parseUInt32("000000000001"));
  EXPECT_EQ(037777777777, cras::parseUInt32("037777777777"));
  EXPECT_EQ(037777777777, cras::parseUInt32("037777777777", 8));
  EXPECT_EQ(037777777777, cras::parseUInt32("37777777777", 8));
  EXPECT_EQ(0x00000001, cras::parseUInt32("0x00000001"));
  EXPECT_EQ(0x00000001, cras::parseUInt32("0X00000001"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0xffffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0xFFffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0xFfffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0xfFffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0Xffffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0XFFffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0XFfffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("0XfFffffff"));
  EXPECT_EQ(0xffffffff, cras::parseUInt32("fFffffff", 16));

  EXPECT_THROW(cras::parseUInt32("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32(" -1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32(" -1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-10"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-42"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-2147483648"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-2147483648", 10), std::invalid_argument);

  EXPECT_THROW(cras::parseUInt32("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("4294967296"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-2147483649"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("0b100000000000000000000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-0b100000000000000000000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("0B01111111111111111111111111111111", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("040000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-040000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("0X100000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("-0X100000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("0XfFffffff", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt32("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseInt64)  // NOLINT
{
  EXPECT_EQ(0, cras::parseInt64("0"));
  EXPECT_EQ(0, cras::parseInt64("0", 2));
  EXPECT_EQ(0, cras::parseInt64("0", 8));
  EXPECT_EQ(0, cras::parseInt64("0", 10));
  EXPECT_EQ(0, cras::parseInt64("0", 16));

  EXPECT_EQ(1, cras::parseInt64("1"));
  EXPECT_EQ(1, cras::parseInt64("+1"));
  EXPECT_EQ(1, cras::parseInt64(" 1"));
  EXPECT_EQ(1, cras::parseInt64(" +1"));
  EXPECT_EQ(1, cras::parseInt64(" +1 "));
  EXPECT_EQ(1, cras::parseInt64("+1 "));
  EXPECT_EQ(1, cras::parseInt64("1 "));
  EXPECT_EQ(1, cras::parseInt64(" 1  "));
  EXPECT_EQ(10, cras::parseInt64("10"));
  EXPECT_EQ(42, cras::parseInt64("42"));
  EXPECT_EQ(9223372036854775807LL, cras::parseInt64("9223372036854775807"));
  EXPECT_EQ(9223372036854775807LL, cras::parseInt64("9223372036854775807", 10));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0001,
    cras::parseInt64("0b0000000000000000000000000000000000000000000000000000000000000001"));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0001,
    cras::parseInt64("0B0000000000000000000000000000000000000000000000000000000000000001"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111,
    cras::parseInt64("0b0111111111111111111111111111111111111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111,
    cras::parseInt64("0B0111111111111111111111111111111111111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111,
    cras::parseInt64("0111111111111111111111111111111111111111111111111111111111111111", 2));
  EXPECT_EQ(01, cras::parseInt64("01"));
  EXPECT_EQ(00000000000000000000001, cras::parseInt64("00000000000000000000001"));
  EXPECT_EQ(00777777777777777777777, cras::parseInt64("00777777777777777777777"));
  EXPECT_EQ(00777777777777777777777, cras::parseInt64("00777777777777777777777", 8));
  EXPECT_EQ(00777777777777777777777, cras::parseInt64("777777777777777777777", 8));
  EXPECT_EQ(0x0000000000000001, cras::parseInt64("0x0000000000000001"));
  EXPECT_EQ(0x0000000000000001, cras::parseInt64("0X0000000000000001"));
  EXPECT_EQ(0x7fffffffffffffff, cras::parseInt64("0x7fffffffffffffff"));
  EXPECT_EQ(0x7fffffffffffffff, cras::parseInt64("0x7Fffffffffffffff"));
  EXPECT_EQ(0x7fffffffffffffff, cras::parseInt64("0X7fffffffffffffff"));
  EXPECT_EQ(0x7fffffffffffffff, cras::parseInt64("0X7Fffffffffffffff"));
  EXPECT_EQ(0x7fffffffffffffff, cras::parseInt64("7Fffffffffffffff", 16));

  EXPECT_EQ(-1, cras::parseInt64("-1"));
  EXPECT_EQ(-1, cras::parseInt64("-1"));
  EXPECT_EQ(-1, cras::parseInt64(" -1"));
  EXPECT_EQ(-1, cras::parseInt64(" -1 "));
  EXPECT_EQ(-1, cras::parseInt64("-1 "));
  EXPECT_EQ(-10, cras::parseInt64("-10"));
  EXPECT_EQ(-42, cras::parseInt64("-42"));
  EXPECT_EQ(std::numeric_limits<int64_t>::min(), cras::parseInt64("-9223372036854775808"));
  EXPECT_EQ(std::numeric_limits<int64_t>::min(), cras::parseInt64("-9223372036854775808", 10));
  EXPECT_EQ(-0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0001,
    cras::parseInt64("-0b0000000000000000000000000000000000000000000000000000000000000001"));
  EXPECT_EQ(-0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0001,
    cras::parseInt64("-0B0000000000000000000000000000000000000000000000000000000000000001"));
  EXPECT_EQ(-0b1000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000,
    cras::parseInt64("-0b1000000000000000000000000000000000000000000000000000000000000000"));
  EXPECT_EQ(-0b1000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000,
    cras::parseInt64("-0B1000000000000000000000000000000000000000000000000000000000000000"));
  EXPECT_EQ(-0b1000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000,
    cras::parseInt64("-1000000000000000000000000000000000000000000000000000000000000000", 2));
  EXPECT_EQ(-01, cras::parseInt64("-01"));
  EXPECT_EQ(-00000000000000000000001, cras::parseInt64("-00000000000000000000001"));
  EXPECT_EQ(-01000000000000000000000, cras::parseInt64("-01000000000000000000000"));
  EXPECT_EQ(-01000000000000000000000, cras::parseInt64("-01000000000000000000000", 8));
  EXPECT_EQ(-01000000000000000000000, cras::parseInt64("-1000000000000000000000", 8));
  EXPECT_EQ(-0x0000000000000001, cras::parseInt64("-0x0000000000000001"));
  EXPECT_EQ(-0x0000000000000001, cras::parseInt64("-0X0000000000000001"));
  EXPECT_EQ(-0x8000000000000000, cras::parseInt64("-0x8000000000000000"));
  EXPECT_EQ(-0x8000000000000000, cras::parseInt64("-0X8000000000000000"));
  EXPECT_EQ(-0x8000000000000000, cras::parseInt64("-8000000000000000", 16));

  EXPECT_THROW(cras::parseInt64("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("9223372036854775808"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("-9223372036854775809"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("0b10000000000000000000000000000000000000000000000000000000000000000"),
               // NOLINT
               std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("-0b10000000000000000000000000000000000000000000000000000000000000001"),
    std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("0B0111111111111111111111111111111111111111111111111111111111111111", 2),
    std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("01000000000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("-01000000000000000000001"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("0XFFFFFFFFFFFFFFFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("-0XFFFFFFFFFFFFFFFF"), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("0X7Fffffffffffffff", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseInt64("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseUInt64)  // NOLINT
{
  EXPECT_EQ(0, cras::parseUInt64("0"));
  EXPECT_EQ(0, cras::parseUInt64("0", 2));
  EXPECT_EQ(0, cras::parseUInt64("0", 8));
  EXPECT_EQ(0, cras::parseUInt64("0", 10));
  EXPECT_EQ(0, cras::parseUInt64("0", 16));

  EXPECT_EQ(1, cras::parseUInt64("1"));
  EXPECT_EQ(1, cras::parseUInt64("+1"));
  EXPECT_EQ(1, cras::parseUInt64(" 1"));
  EXPECT_EQ(1, cras::parseUInt64(" +1"));
  EXPECT_EQ(1, cras::parseUInt64(" +1 "));
  EXPECT_EQ(1, cras::parseUInt64("+1 "));
  EXPECT_EQ(1, cras::parseUInt64("1 "));
  EXPECT_EQ(1, cras::parseUInt64(" 1  "));
  EXPECT_EQ(10, cras::parseUInt64("10"));
  EXPECT_EQ(42, cras::parseUInt64("42"));
  EXPECT_EQ(9223372036854775807, cras::parseUInt64("9223372036854775807"));
  EXPECT_EQ(0x8000000000000000, cras::parseUInt64("9223372036854775808"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("18446744073709551615"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("18446744073709551615", 10));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0001,
    cras::parseUInt64("0b0000000000000000000000000000000000000000000000000000000000000001"));
  EXPECT_EQ(0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0001,
    cras::parseUInt64("0B0000000000000000000000000000000000000000000000000000000000000001"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111,
    cras::parseUInt64("0b0111111111111111111111111111111111111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111,
    cras::parseUInt64("0B0111111111111111111111111111111111111111111111111111111111111111"));
  EXPECT_EQ(0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111,
    cras::parseUInt64("0111111111111111111111111111111111111111111111111111111111111111", 2));
  EXPECT_EQ(01, cras::parseUInt64("01"));
  EXPECT_EQ(00000000000000000000001, cras::parseUInt64("00000000000000000000001"));
  EXPECT_EQ(01777777777777777777777, cras::parseUInt64("01777777777777777777777"));
  EXPECT_EQ(01777777777777777777777, cras::parseUInt64("01777777777777777777777", 8));
  EXPECT_EQ(01777777777777777777777, cras::parseUInt64("1777777777777777777777", 8));
  EXPECT_EQ(0x0000000000000001, cras::parseUInt64("0x0000000000000001"));
  EXPECT_EQ(0x0000000000000001, cras::parseUInt64("0X0000000000000001"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0xffffffffffffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0xFFffFFffFFffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0xFfffFfffFfffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0xfFfffFfffFffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0Xffffffffffffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0XFFffFFffFFffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0XFfffFfffFfffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("0XfFfffFfffFffffff"));
  EXPECT_EQ(0xffffffffffffffff, cras::parseUInt64("fFfffFfffFffffff", 16));

  EXPECT_THROW(cras::parseUInt64("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64(" -1"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64(" -1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-1 "), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-10"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-42"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-9223372036854775808"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-9223372036854775808", 10), std::invalid_argument);

  EXPECT_THROW(cras::parseUInt64("1.0"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("3.14"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("18446744073709551616"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-9223372036854775809"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("0b10000000000000000000000000000000000000000000000000000000000000000"),
    std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-0b10000000000000000000000000000000000000000000000000000000000000000"),
    std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("0B0111111111111111111111111111111111111111111111111111111111111111", 2),
    std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("02000000000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("02000000000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("0X10000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("-0X10000000000000000"), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("0XfFfffFfffFffffff", 16), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("2", 2), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("9", 8), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("a", 10), std::invalid_argument);
  EXPECT_THROW(cras::parseUInt64("g", 16), std::invalid_argument);
}

TEST(StringUtils, ParseFloat)  // NOLINT
{
  EXPECT_EQ(1.0f, cras::parseFloat("1"));
  EXPECT_EQ(1.0f, cras::parseFloat("1.0"));
  EXPECT_EQ(1.0f, cras::parseFloat("+1.0"));
  EXPECT_EQ(1.0f, cras::parseFloat(" +1.0"));
  EXPECT_EQ(1.0f, cras::parseFloat(" 1.0"));
  EXPECT_EQ(1.0f, cras::parseFloat(" 1.0 "));
  EXPECT_EQ(1.0f, cras::parseFloat("1.0e0"));
  EXPECT_EQ(1.0f, cras::parseFloat("0.1e1"));
  EXPECT_EQ(1.0f, cras::parseFloat("0.1e+1"));
  EXPECT_EQ(1.0f, cras::parseFloat("10e-1"));
  EXPECT_EQ(3.141592f, cras::parseFloat("3.141592"));
  EXPECT_EQ(-1.0f, cras::parseFloat("-1.0"));
  EXPECT_EQ(-1.0f, cras::parseFloat(" -1.0"));
  EXPECT_EQ(-1.0f, cras::parseFloat(" -1.0"));
  EXPECT_EQ(-1.0f, cras::parseFloat(" -1.0 "));
  EXPECT_EQ(-1.0f, cras::parseFloat("-1.0e0"));
  EXPECT_EQ(-1.0f, cras::parseFloat("-0.1e1"));
  EXPECT_EQ(-1.0f, cras::parseFloat("-0.1e+1"));
  EXPECT_EQ(-1.0f, cras::parseFloat("-10e-1"));
  EXPECT_EQ(-3.141592f, cras::parseFloat("-3.141592"));
  EXPECT_EQ(3.402823466e+38f, cras::parseFloat("3.402823466e+38"));
  EXPECT_EQ(-3.402823466e+38f, cras::parseFloat("-3.402823466e+38"));
  EXPECT_THROW(cras::parseFloat("3.5e+38"), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat("-3.5e+38"), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat(" 1.0  "), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat("1 .0"), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat("1. 0"), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat("a"), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat("1,0"), std::invalid_argument);
  EXPECT_THROW(cras::parseFloat("3,14"), std::invalid_argument);
}

TEST(StringUtils, ParseDouble)  // NOLINT
{
  EXPECT_EQ(1.0, cras::parseDouble("1"));
  EXPECT_EQ(1.0, cras::parseDouble("1.0"));
  EXPECT_EQ(1.0, cras::parseDouble("+1.0"));
  EXPECT_EQ(1.0, cras::parseDouble(" +1.0"));
  EXPECT_EQ(1.0, cras::parseDouble(" 1.0"));
  EXPECT_EQ(1.0, cras::parseDouble(" 1.0 "));
  EXPECT_EQ(1.0, cras::parseDouble("1.0e0"));
  EXPECT_EQ(1.0, cras::parseDouble("0.1e1"));
  EXPECT_EQ(1.0, cras::parseDouble("0.1e+1"));
  EXPECT_EQ(1.0, cras::parseDouble("10e-1"));
  EXPECT_EQ(3.141592, cras::parseDouble("3.141592"));
  EXPECT_EQ(-1.0, cras::parseDouble("-1.0"));
  EXPECT_EQ(-1.0, cras::parseDouble(" -1.0"));
  EXPECT_EQ(-1.0, cras::parseDouble(" -1.0"));
  EXPECT_EQ(-1.0, cras::parseDouble(" -1.0 "));
  EXPECT_EQ(-1.0, cras::parseDouble("-1.0e0"));
  EXPECT_EQ(-1.0, cras::parseDouble("-0.1e1"));
  EXPECT_EQ(-1.0, cras::parseDouble("-0.1e+1"));
  EXPECT_EQ(-1.0, cras::parseDouble("-10e-1"));
  EXPECT_EQ(-3.141592, cras::parseDouble("-3.141592"));
  EXPECT_EQ(1.79769e+308, cras::parseDouble("1.79769e+308"));
  EXPECT_EQ(-1.79769e+308, cras::parseDouble("-1.79769e+308"));
  EXPECT_THROW(cras::parseDouble("1.8e+308"), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble("-1.8e+308"), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble(" 1.0  "), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble("1 .0"), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble("1. 0"), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble("a"), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble("1,0"), std::invalid_argument);
  EXPECT_THROW(cras::parseDouble("3,14"), std::invalid_argument);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
