/**
 * \file
 * \brief Unit test for filter_base.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"
#include <cras_cpp_common/filter_base.hpp>

using namespace cras;

#define test(param, def, expected, defUsed) \
{ auto r = this->getParamVerbose((param), (def)); EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); } \
{ auto r = this->getParamVerbose((param), cras::optional(def)); EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); } \
EXPECT_EQ((expected), this->getParam((param), (def)));\
EXPECT_EQ((expected), this->getParam((param), cras::optional(def)));

#define test_s(param, def, expected, defUsed) \
{ auto r = this->getParamVerbose((param), (def)); EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); } \
{ auto r = this->getParamVerbose((param), cras::optional(def)); EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); } \
EXPECT_EQ((expected), this->getParam((param), (def)));\
EXPECT_EQ((expected), this->getParam((param), cras::optional(def)));

#define test_near(param, def, expected, defUsed) \
{ auto r = this->getParamVerbose((param), (def)); EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); } \
{ auto r = this->getParamVerbose((param), cras::optional(def)); EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); } \
EXPECT_GT(1e-6, distance((expected), this->getParam((param), (def))));\
EXPECT_GT(1e-6, distance((expected), this->getParam((param), cras::optional(def))));

#define test_defaults(param, def) test_s((param), (def), (def), true) 

double distance(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2)
{
  return std::sqrt(std::pow(q1.x - q2.x, 2) + std::pow(q1.y - q2.y, 2) + std::pow(q1.z - q2.z, 2) + std::pow(q1.w - q2.w, 2));
}

double distance(const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2)
{
  return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2) + std::pow(v1.z - v2.z, 2));
}

double distance(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2)
{
  return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2) + std::pow(v1.z - v2.z, 2));
}

double distance(const geometry_msgs::Point32& v1, const geometry_msgs::Point32& v2)
{
  return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2) + std::pow(v1.z - v2.z, 2));
}

double distance(const geometry_msgs::Transform& t1, const geometry_msgs::Transform& t2)
{
  return distance(t1.translation, t2.translation) + distance(t1.rotation, t2.rotation);
}

double distance(const tf2::Quaternion& q1, const tf2::Quaternion& q2)
{
  return q1.angleShortestPath(q2);
}

double distance(const tf2::Vector3& v1, const tf2::Vector3& v2)
{
  return v1.distance(v2);
}

double distance(const tf2::Transform& v1, const tf2::Transform& v2)
{
  return distance(v1.getOrigin(), v2.getOrigin()) + distance(v1.getRotation(), v2.getRotation());
}

namespace ros
{
bool operator==(const ros::Rate& r1, const ros::Rate& r2)
{
  return r1.expectedCycleTime().operator==(r2.expectedCycleTime());
}

bool operator==(const ros::WallRate& r1, const ros::WallRate& r2)
{
  return r1.expectedCycleTime().operator==(r2.expectedCycleTime());
}
}

class FilterUtils : public testing::TestWithParam<std::tuple<bool>> {};

class TestFilter : public cras::FilterBase<std::string>
{
  private: bool isChainConfig;
  public: explicit TestFilter(bool isChainConfig) : isChainConfig(isChainConfig) {}
  public: bool update(const std::string &data_in, std::string &data_out) override
  {
    return false;
  }
  
  protected: bool configure() override
  {
    return true;
  }

  public: void testXmlRpcValue()
  {
    // XMLRPCVALUE PARAMETERS

    XmlRpc::XmlRpcValue dummyXml0(false);
    XmlRpc::XmlRpcValue dummyXml1(true);

    // test(param, def, expected, defUsed)
    test("bool_True", dummyXml0, XmlRpc::XmlRpcValue(true), false)
    test("bool_False", dummyXml1, XmlRpc::XmlRpcValue(false), false)
    test("bool_true", dummyXml0, XmlRpc::XmlRpcValue(true), false)
    test("bool_false", dummyXml1, XmlRpc::XmlRpcValue(false), false)
    // the XmlRpc type of the default value is not checked 
    test("int_0", dummyXml1, XmlRpc::XmlRpcValue(0), false)
    test("int_1", dummyXml0, XmlRpc::XmlRpcValue(1), false)
    test("int_2", dummyXml0, XmlRpc::XmlRpcValue(2), false)
    test("int_minus_1", dummyXml0, XmlRpc::XmlRpcValue(-1), false)
    test("int_max", dummyXml0, XmlRpc::XmlRpcValue(INT_MAX), false)
    test("int_min", dummyXml0, XmlRpc::XmlRpcValue(INT_MIN), false)
    test("double_0", dummyXml0, XmlRpc::XmlRpcValue(0.0), false)
    test("double_1", dummyXml0, XmlRpc::XmlRpcValue(1.0), false)
    test("double_3_14", dummyXml0, XmlRpc::XmlRpcValue(3.14), false)
    test("double_minus_1", dummyXml0, XmlRpc::XmlRpcValue(-1.0), false)
    test("double_minus_3_14", dummyXml0, XmlRpc::XmlRpcValue(-3.14), false)
    test("str_empty", dummyXml0, XmlRpc::XmlRpcValue(""), false)
    test("str_a", dummyXml0, XmlRpc::XmlRpcValue("a"), false)
    test("str_asd", dummyXml0, XmlRpc::XmlRpcValue("asd"), false)
    {XmlRpc::XmlRpcValue v; v.setSize(0); test("list_empty", dummyXml0, v, false)}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = true; v[1] = false; v[2] = true; test("list_bool", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = 0; v[1] = 1; v[2] = -1; test("list_int", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = 0; v[1] = 1; v[2] = 2; test("list_uint", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = 0.0; v[1] = 1.0; v[2] = -1.0; test("list_double", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = "a"; v[1] = "b"; v[2] = "cde"; test("list_str", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v.begin(); test("dict_empty", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v["a"] = true; v["b"] = false; v["c"] = true; test("dict_bool", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v["a"] = 0; v["b"] = 1; v["c"] = -1; test("dict_int", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v["a"] = 0; v["b"] = 1; v["c"] = 2; test("dict_uint", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v["aaa"] = 0.0; v["bbb"] = 1.0; v["ccc"] = -1.0; test("dict_double", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {XmlRpc::XmlRpcValue v; v["aaa"] = "a"; v["bbb"] = "b"; v["ccc"] = "c"; test("dict_str", dummyXml0, XmlRpc::XmlRpcValue(v), false)}
    {
      // dict_mixed has one more value with key "", but ROS param server cannot hold this kind of key
      // in a dict config (but it can in chain config)
      XmlRpc::XmlRpcValue v; v["bbb"] = 1.0; v["ccc"] = false;
      if (this->isChainConfig) v[""] = 0;
      test_s("dict_mixed", dummyXml0, XmlRpc::XmlRpcValue(v), false)
    }
    {
      XmlRpc::XmlRpcValue va; va["b"] = 1; va["c"] = 2;
      XmlRpc::XmlRpcValue vd; vd["e"] = 3; vd["f"] = 4; vd["g"] = 5;
      XmlRpc::XmlRpcValue v; v["a"] = va; v["d"] = vd;
      test_s("dict_str_recursive", dummyXml0, XmlRpc::XmlRpcValue(v), false)
    }
    test_defaults("nonexistent", dummyXml0)
    test_defaults("nonexistent", dummyXml1)
  }
  
  public: void testBool()
  {
    // BOOL PARAMETERS
    
    // test that getParam result type is the same as the default value
    static_assert(std::is_same<bool, decltype(this->getParam("bool_True", false))>::value);
    static_assert(std::is_same<bool, decltype(this->getParam("bool_True", cras::optional(false)))>::value);

    // test non-verbose getParam
    EXPECT_EQ(true, this->getParam("bool_True", false));
    // test explicit conversion of GetParamResult to the contained type
    EXPECT_EQ(true, static_cast<bool>(this->getParamVerbose("bool_True", false)));
    // test implicit conversion of GetParamResult to the contained type
    { bool r = this->getParamVerbose("bool_True", false); EXPECT_EQ(true, r); }
    
    // test(param, def, expected, defUsed)
    test("bool_True", false, true, false)
    test("bool_False", true, false, false)
    test("bool_true", false, true, false)
    test("bool_false", true, false, false)
    test("int_0", true, false, false)
    test("int_1", false, true, false)
    test_defaults("int_2", false)
    test_defaults("int_minus_1", false)
    test_defaults("int_max", false)
    test_defaults("int_min", false)
    test_defaults("double_0", true)
    test_defaults("double_1", false)
    test_defaults("double_3_14", false)
    test_defaults("double_minus_1", false)
    test_defaults("double_minus_3_14", false)
    test_defaults("str_empty", true)
    test_defaults("str_a", false)
    test_defaults("str_asd", false)
    test_defaults("list_empty", true)
    test_defaults("list_bool", false)
    test_defaults("list_int", false)
    test_defaults("list_uint", false)
    test_defaults("list_double", false)
    test_defaults("list_str", false)
    test_defaults("dict_empty", true)
    test_defaults("dict_bool", false)
    test_defaults("dict_int", false)
    test_defaults("dict_uint", false)
    test_defaults("dict_double", false)
    test_defaults("dict_str", false)
    test_defaults("dict_mixed", false)
    test_defaults("dict_str_recursive", false)
    test_defaults("nonexistent", false)
    test_defaults("nonexistent", true)
  }
    
  public: void testInt()
  {
    // INT PARAMETERS
    // test(param, def, expected, defUsed)
    test_defaults("bool_True", 0)
    test_defaults("bool_False", 1)
    test_defaults("bool_true", 0)
    test_defaults("bool_false", 1)
    test("int_0", 1, 0, false)
    test("int_1", 0, 1, false)
    test("int_2", 0, 2, false)
    test("int_minus_1", 0, -1, false)
    test("int_max", 0, INT_MAX, false)
    test("int_min", 0, INT_MIN, false)
    test_defaults("double_0", 1)
    test_defaults("double_1", 0)
    test_defaults("double_3_14", 0)
    test_defaults("double_minus_1", 0)
    test_defaults("double_minus_3_14", 0)
    test_defaults("double_lowest", 0)
    test_defaults("double_min", 0)
    test_defaults("double_max", 0)
    test_defaults("str_empty", 1)
    test_defaults("str_a", 0)
    test_defaults("str_asd", 0)
    test_defaults("list_empty", 0)
    test_defaults("list_bool", 0)
    test_defaults("list_int", 0)
    test_defaults("list_uint", 0)
    test_defaults("list_double", 0)
    test_defaults("list_str", 0)
    test_defaults("dict_empty", 0)
    test_defaults("dict_bool", 0)
    test_defaults("dict_int", 0)
    test_defaults("dict_uint", 0)
    test_defaults("dict_double", 0)
    test_defaults("dict_str", 0)
    test_defaults("dict_mixed", 0)
    test_defaults("dict_str_recursive", 0)
    test_defaults("nonexistent", 0)
    test_defaults("nonexistent", 1)
    test_defaults("nonexistent", -1)
    test_defaults("nonexistent", INT_MAX)
    test_defaults("nonexistent", INT_MIN)
    test_defaults("nonexistent", (long)INT_MAX+1)

    test("int_0", 1u, 0u, false)
    test("int_1", 0u, 1u, false)
    test("int_2", 0u, 2u, false)
    test_defaults("int_minus_1", 0u)
    test("int_max", 0u, (unsigned int)INT_MAX, false)
    test_defaults("int_min", 0u)

    test("int_0", (signed char)1, (signed char)0, false)
    test("int_1", (signed char)0, (signed char)1, false)
    test("int_2", (signed char)0, (signed char)2, false)
    test("int_minus_1", (signed char)0, (signed char)-1, false)
    test_defaults("int_max", (signed char)0)
    test_defaults("int_min", (signed char)0)

    test("int_0", (unsigned char)1, (unsigned char)0, false)
    test("int_1", (unsigned char)0, (unsigned char)1, false)
    test("int_2", (unsigned char)0, (unsigned char)2, false)
    test_defaults("int_minus_1", (unsigned char)0)
    test_defaults("int_max", (unsigned char)0)
    test_defaults("int_min", (unsigned char)0)

    test("int_0", (short)1, (short)0, false)
    test("int_1", (short)0, (short)1, false)
    test("int_2", (short)0, (short)2, false)
    test("int_minus_1", (short)0, (short)-1, false)
    test_defaults("int_max", (short)0)
    test_defaults("int_min", (short)0)

    test("int_0", (unsigned short)1, (unsigned short)0, false)
    test("int_1", (unsigned short)0, (unsigned short)1, false)
    test("int_2", (unsigned short)0, (unsigned short)2, false)
    test_defaults("int_minus_1", (unsigned short)0)
    test_defaults("int_max", (unsigned short)0)
    test_defaults("int_min", (unsigned short)0)

    test("int_0", (long)1, (long)0, false)
    test("int_1", (long)0, (long)1, false)
    test("int_2", (long)0, (long)2, false)
    test("int_minus_1", (long)0, (long)-1, false)
    test("int_max", (long)0, (long)INT_MAX, false)
    test("int_min", (long)0, (long)INT_MIN, false)

    test("int_0", (unsigned long)1, (unsigned long)0, false)
    test("int_1", (unsigned long)0, (unsigned long)1, false)
    test("int_2", (unsigned long)0, (unsigned long)2, false)
    test_defaults("int_minus_1", (unsigned long)0)
    test("int_max", (unsigned long)0, (unsigned long)INT_MAX, false)
    test_defaults("int_min", (unsigned long)0)

    test("int_0", (long long)1, (long long)0, false)
    test("int_1", (long long)0, (long long)1, false)
    test("int_2", (long long)0, (long long)2, false)
    test("int_minus_1", (long long)0, (long long)-1, false)
    test("int_max", (long long)0, (long long)INT_MAX, false)
    test("int_min", (long long)0, (long long)INT_MIN, false)

    test("int_0", (unsigned long long)1, (unsigned long long)0, false)
    test("int_1", (unsigned long long)0, (unsigned long long)1, false)
    test("int_2", (unsigned long long)0, (unsigned long long)2, false)
    test_defaults("int_minus_1", (unsigned long long)0)
    test("int_max", (unsigned long long)0, (unsigned long long)INT_MAX, false)
    test_defaults("int_min", (unsigned long long)0)
  }
    
  public: void testFloat()
  {
    // FLOAT PARAMETERS
    // test(param, def, expected, defUsed)
    test_defaults("bool_True", 0.0f)
    test_defaults("bool_False", 1.0f)
    test_defaults("bool_true", 0.0f)
    test_defaults("bool_false", 1.0f)
    test("int_0", 1.0f, 0.0f, false)
    test("int_1", 0.0f, 1.0f, false)
    test("int_2", 0.0f, 2.0f, false)
    test("int_minus_1", 0.0f, -1.0f, false)
    test("int_max", 0.0f, INT_MAX, false)
    test("int_min", 0.0f, INT_MIN, false)
    test("double_0", 1.0f, 0.0f, false)
    test("double_1", 0.0f, 1.0f, false)
    test("double_3_14", 0.0f, 3.14f, false)
    test("double_minus_1", 0.0f, -1.0f, false)
    test("double_minus_3_14", 0.0f, -3.14f, false)
    test_defaults("double_lowest", 0.0f)
    test("double_min", 0.0f, 0.0f, false)
    test_defaults("double_max", 0.0f)
    test("double_inf", 0.0f, std::numeric_limits<float>::infinity(), false)
    test("double_minus_inf", 0.0f, -std::numeric_limits<float>::infinity(), false)
    test_defaults("str_empty", 1.0f)
    test_defaults("str_a", 0.0f)
    test_defaults("str_asd", 0.0f)
    test_defaults("list_empty", 0.0f)
    test_defaults("list_bool", 0.0f)
    test_defaults("list_int", 0.0f)
    test_defaults("list_uint", 0.0f)
    test_defaults("list_double", 0.0f)
    test_defaults("list_str", 0.0f)
    test_defaults("dict_empty", 0.0f)
    test_defaults("dict_bool", 0.0f)
    test_defaults("dict_int", 0.0f)
    test_defaults("dict_uint", 0.0f)
    test_defaults("dict_double", 0.0f)
    test_defaults("dict_str", 0.0f)
    test_defaults("dict_mixed", 0.0f)
    test_defaults("dict_str_recursive", 0.0f)
    test_defaults("nonexistent", 0.0f)
    test_defaults("nonexistent", 1.0f)
    test_defaults("nonexistent", -1.0f)
    test_defaults("nonexistent", FLT_MAX)
    test_defaults("nonexistent", FLT_MIN)
  }
    
  public: void testDouble()
  {
    // DOUBLE PARAMETERS
    // test(param, def, expected, defUsed)
    test_defaults("bool_True", 0.0)
    test_defaults("bool_False", 1.0)
    test_defaults("bool_true", 0.0)
    test_defaults("bool_false", 1.0)
    test("int_0", 1.0, 0.0, false)
    test("int_1", 0.0, 1.0, false)
    test("int_2", 0.0, 2.0, false)
    test("int_minus_1", 0.0, -1.0, false)
    test("int_max", 0.0, INT_MAX, false)
    test("int_min", 0.0, INT_MIN, false)
    test("double_0", 1.0, 0.0, false)
    test("double_1", 0.0, 1.0, false)
    test("double_3_14", 0.0, 3.14, false)
    test("double_minus_1", 0.0, -1.0, false)
    test("double_minus_3_14", 0.0, -3.14, false)
    test("double_lowest", 0.0, -1.79769e+308, false)
    test("double_min", 0.0, 2.22507e-308, false)
    test("double_max", 0.0, 1.79769e+308, false)
    test("double_inf", 0.0, std::numeric_limits<double>::infinity(), false)
    test("double_minus_inf", 0.0, -std::numeric_limits<double>::infinity(), false)
    test_defaults("str_empty", 1.0)
    test_defaults("str_a", 0.0)
    test_defaults("str_asd", 0.0)
    test_defaults("list_empty", 0.0)
    test_defaults("list_bool", 0.0)
    test_defaults("list_int", 0.0)
    test_defaults("list_uint", 0.0)
    test_defaults("list_double", 0.0)
    test_defaults("list_str", 0.0)
    test_defaults("dict_empty", 0.0)
    test_defaults("dict_bool", 0.0)
    test_defaults("dict_int", 0.0)
    test_defaults("dict_uint", 0.0)
    test_defaults("dict_double", 0.0)
    test_defaults("dict_str", 0.0)
    test_defaults("dict_mixed", 0.0)
    test_defaults("dict_str_recursive", 0.0)
    test_defaults("nonexistent", 0.0)
    test_defaults("nonexistent", 1.0)
    test_defaults("nonexistent", -1.0)
    test_defaults("nonexistent", DBL_MAX)
    test_defaults("nonexistent", DBL_MIN)

    { auto r = this->getParamVerbose("double_nan", 0.0); EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    { auto r = this->getParamVerbose("double_nan", cras::optional(0.0)); EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->getParam("double_nan", 0.0)));
    EXPECT_TRUE(std::isnan(this->getParam("double_nan", cras::optional(0.0))));
    
    { auto r = this->getParamVerbose("nonexistent", std::numeric_limits<double>::quiet_NaN()); EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    { auto r = this->getParamVerbose("nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN())); EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->getParam("nonexistent", std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(this->getParam("nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()))));
    
    test("int_0", (long double)1.0, (long double)0.0, false)
    test("int_1", (long double)0.0, (long double)1.0, false)
    test("int_2", (long double)0.0, (long double)2.0, false)
    test("int_minus_1", (long double)0.0, (long double)-1.0, false)
    test("int_max", (long double)0.0, (long double)INT_MAX, false)
    test("int_min", (long double)0.0, (long double)INT_MIN, false)
    test("double_0", (long double)1.0, (long double)0.0, false)
    test("double_1", (long double)0.0, (long double)1.0, false)
    test("double_3_14", (long double)0.0, (long double)3.14, false)
    test("double_minus_1", (long double)0.0, (long double)-1.0, false)
    test("double_minus_3_14", (long double)0.0, (long double)-3.14, false)
    test("double_lowest", (long double)0.0, (long double)-1.79769e+308, false)
    test("double_min", (long double)0.0, (long double)2.22507e-308, false)
    test("double_max", (long double)0.0, (long double)1.79769e+308, false)
    test("double_inf", (long double)0.0, std::numeric_limits<long double>::infinity(), false)
    test("double_minus_inf", (long double)0.0, -std::numeric_limits<long double>::infinity(), false)
  }
    
  public: void testCstring()
  {
    // C-STRING PARAMETERS
    // test(param, def, expected, defUsed)
    // verify the std::string <-> c-string interop works
    static_assert(std::is_same<std::string, decltype(this->getParam("bool_True", "0"))>::value);
    static_assert(std::is_same<std::string, decltype(this->getParam("bool_True", cras::optional("")))>::value);
    
    test_defaults("bool_True", "0")
    test_defaults("bool_False", "1")
    test_defaults("bool_true", "0")
    test_defaults("bool_false", "1")
    test_defaults("int_0", "1")
    test_defaults("int_1", "0")
    test_defaults("int_2", "0")
    test_defaults("int_minus_1", "0")
    test_defaults("int_max", "0")
    test_defaults("int_min", "0")
    test_defaults("double_0", "1")
    test_defaults("double_1", "0")
    test_defaults("double_3_14", "0")
    test_defaults("double_minus_1", "0")
    test_defaults("double_minus_3_14", "0")
    test_s("str_empty", "a", "", false)
    test_s("str_a", "", "a", false)
    test_s("str_asd", "a", "asd", false)
    test_defaults("list_empty", "0")
    test_defaults("list_bool", "0")
    test_defaults("list_int", "0")
    test_defaults("list_uint", "0")
    test_defaults("list_double", "0")
    test_defaults("list_str", "0")
    test_defaults("dict_empty", "0")
    test_defaults("dict_bool", "0")
    test_defaults("dict_int", "0")
    test_defaults("dict_uint", "0")
    test_defaults("dict_double", "0")
    test_defaults("dict_str", "0")
    test_defaults("dict_mixed", "0")
    test_defaults("dict_str_recursive", "0")
    test_defaults("nonexistent", "")
    test_defaults("nonexistent", "a")
    test_defaults("nonexistent", "asd")
  }
    
  public: void testString()
  {
    // STD::STRING PARAMETERS
    // test(param, def, expected, defUsed)
    test_defaults("bool_True", std::string("0"))
    test_defaults("bool_False", std::string("1"))
    test_defaults("bool_true", std::string("0"))
    test_defaults("bool_false", std::string("1"))
    test_defaults("int_0", std::string("1"))
    test_defaults("int_1", std::string("0"))
    test_defaults("int_2", std::string("0"))
    test_defaults("int_minus_1", std::string("0"))
    test_defaults("int_max", std::string("0"))
    test_defaults("int_min", std::string("0"))
    test_defaults("double_0", std::string("1"))
    test_defaults("double_1", std::string("0"))
    test_defaults("double_3_14", std::string("0"))
    test_defaults("double_minus_1", std::string("0"))
    test_defaults("double_minus_3_14", std::string("0"))
    test_s("str_empty", std::string("a"), std::string(""), false)
    test_s("str_a", std::string(""), std::string("a"), false)
    test_s("str_asd", std::string("a"), std::string("asd"), false)
    test_defaults("list_empty", std::string("0"))
    test_defaults("list_bool", std::string("0"))
    test_defaults("list_int", std::string("0"))
    test_defaults("list_uint", std::string("0"))
    test_defaults("list_double", std::string("0"))
    test_defaults("list_str", std::string("0"))
    test_defaults("dict_empty", std::string("0"))
    test_defaults("dict_bool", std::string("0"))
    test_defaults("dict_int", std::string("0"))
    test_defaults("dict_uint", std::string("0"))
    test_defaults("dict_double", std::string("0"))
    test_defaults("dict_str", std::string("0"))
    test_defaults("dict_mixed", std::string("0"))
    test_defaults("dict_str_recursive", std::string("0"))
    test_defaults("nonexistent", std::string(""))
    test_defaults("nonexistent", std::string("a"))
    test_defaults("nonexistent", std::string("asd"))
  }
    
  public: void testList()
  {
    // LIST PARAMETERS
    // test(param, def, expected, defUsed)
    const std::vector<std::string> dummyVec({"a"});
    test_defaults("bool_True", dummyVec)
    test_defaults("bool_False", dummyVec)
    test_defaults("bool_true", dummyVec)
    test_defaults("bool_false", dummyVec)
    test_defaults("int_0", dummyVec)
    test_defaults("int_1", dummyVec)
    test_defaults("int_2", dummyVec)
    test_defaults("int_minus_1", dummyVec)
    test_defaults("int_max", dummyVec)
    test_defaults("int_min", dummyVec)
    test_defaults("double_0", dummyVec)
    test_defaults("double_1", dummyVec)
    test_defaults("double_3_14", dummyVec)
    test_defaults("double_minus_1", dummyVec)
    test_defaults("double_minus_3_14", dummyVec)
    test_defaults("str_empty", dummyVec)
    test_defaults("str_a", dummyVec)
    test_defaults("str_asd", dummyVec)
    test_s("list_empty", std::vector<std::string>({"a"}), std::vector<std::string>(), false)
    test_s("list_bool", std::vector<bool>(), std::vector<bool>({true, false, true}), false)
    test_s("list_int", std::vector<int>(), std::vector<int>({0, 1, -1}), false)
    test_s("list_uint", std::vector<unsigned int>(), std::vector<unsigned int>({0u, 1u, 2u}), false)
    test_s("list_double", std::vector<double>(), std::vector<double>({0.0, 1.0, -1.0}), false)
    test_s("list_str", std::vector<std::string>(), std::vector<std::string>({"a", "b", "cde"}), false)
    test_defaults("dict_empty", dummyVec)
    test_defaults("dict_bool", dummyVec)
    test_defaults("dict_int", dummyVec)
    test_defaults("dict_uint", dummyVec)
    test_defaults("dict_double", dummyVec)
    test_defaults("dict_str", dummyVec)
    test_defaults("dict_mixed", dummyVec)
    test_defaults("dict_str_recursive", dummyVec)
    test_defaults("nonexistent", std::vector<std::string>())
    test_defaults("nonexistent", std::vector<bool>({true, false, true}))
    test_defaults("nonexistent", std::vector<int>({0, 1, -1}))
    test_defaults("nonexistent", std::vector<unsigned int>({0u, 1u, 2u}))
    test_defaults("nonexistent", std::vector<double>({0.0, 1.0, -1.0}))
    test_defaults("nonexistent", std::vector<std::string>({"a", "b", "cde"}))
    
    test_s("list_empty", std::list<std::string>({"a"}), std::list<std::string>(), false)
    test_s("list_bool", std::list<bool>(), std::list<bool>({true, false, true}), false)
    test_s("list_int", std::list<int>(), std::list<int>({0, 1, -1}), false)
    test_s("list_uint", std::list<unsigned int>(), std::list<unsigned int>({0u, 1u, 2u}), false)
    test_s("list_double", std::list<double>(), std::list<double>({0.0, 1.0, -1.0}), false)
    test_s("list_str", std::list<std::string>(), std::list<std::string>({"a", "b", "cde"}), false)
    
    test_s("list_empty", std::set<std::string>({"a"}), std::set<std::string>(), false)
    test_s("list_bool", std::set<bool>(), std::set<bool>({true, false, true}), false)
    test_s("list_int", std::set<int>(), std::set<int>({0, 1, -1}), false)
    test_s("list_uint", std::set<unsigned int>(), std::set<unsigned int>({0u, 1u, 2u}), false)
    test_s("list_double", std::set<double>(), std::set<double>({0.0, 1.0, -1.0}), false)
    test_s("list_str", std::set<std::string>(), std::set<std::string>({"a", "b", "cde"}), false)
    
    test_s("list_empty", std::unordered_set<std::string>({"a"}), std::unordered_set<std::string>(), false)
    test_s("list_bool", std::unordered_set<bool>(), std::unordered_set<bool>({true, false, true}), false)
    test_s("list_int", std::unordered_set<int>(), std::unordered_set<int>({0, 1, -1}), false)
    test_s("list_uint", std::unordered_set<unsigned int>(), std::unordered_set<unsigned int>({0u, 1u, 2u}), false)
    test_s("list_double", std::unordered_set<double>(), std::unordered_set<double>({0.0, 1.0, -1.0}), false)
    test_s("list_str", std::unordered_set<std::string>(), std::unordered_set<std::string>({"a", "b", "cde"}), false)
  }
    
  public: void testDict()
  {
    // DICT PARAMETERS
    // test(param, def, expected, defUsed)
    const std::map<std::string, std::string> dummyMap({{"a", "a"}});
    test_defaults("bool_True", dummyMap)
    test_defaults("bool_False", dummyMap)
    test_defaults("bool_true", dummyMap)
    test_defaults("bool_false", dummyMap)
    test_defaults("int_0", dummyMap)
    test_defaults("int_1", dummyMap)
    test_defaults("int_2", dummyMap)
    test_defaults("int_minus_1", dummyMap)
    test_defaults("int_max", dummyMap)
    test_defaults("int_min", dummyMap)
    test_defaults("double_0", dummyMap)
    test_defaults("double_1", dummyMap)
    test_defaults("double_3_14", dummyMap)
    test_defaults("double_minus_1", dummyMap)
    test_defaults("double_minus_3_14", dummyMap)
    test_defaults("str_empty", dummyMap)
    test_defaults("str_a", dummyMap)
    test_defaults("str_asd", dummyMap)
    test_defaults("list_empty", dummyMap)
    test_defaults("list_bool", dummyMap)
    test_defaults("list_int", dummyMap)
    test_defaults("list_uint", dummyMap)
    test_defaults("list_double", dummyMap)
    test_defaults("list_str", dummyMap)
    test_s("dict_empty", dummyMap, (std::map<std::string, std::string>()), false)
    test_s("dict_bool", (std::map<std::string, bool>()), (std::map<std::string, bool>({{"a", true}, {"b", false}, {"c", true}})), false)
    test_s("dict_int", (std::map<std::string, int>()), (std::map<std::string, int>({{"a", 0}, {"b", 1}, {"c", -1}})), false)
    test_s("dict_uint", (std::map<std::string, unsigned int>()), (std::map<std::string, unsigned int>({{"a", 0}, {"b", 1}, {"c", 2}})), false)
    test_s("dict_double", (std::map<std::string, double>()), (std::map<std::string, double>({{"aaa", 0.0}, {"bbb", 1.0}, {"ccc", -1.0}})), false)
    test_s("dict_str", (std::map<std::string, std::string>()), (std::map<std::string, std::string>({{"aaa", "a"}, {"bbb", "b"}, {"ccc", "c"}})), false)
    test_defaults("dict_mixed", dummyMap)
    test_s("dict_str_recursive", (std::map<std::string, std::map<std::string, int>>()),
           (std::map<std::string, std::map<std::string, int>>({{"a", {{"b", 1}, {"c", 2}}}, {"d", {{"e", 3}, {"f", 4}, {"g", 5}}}})), false)
    test_defaults("nonexistent", (std::map<std::string, std::string>()))
    test_defaults("nonexistent", (std::map<std::string, bool>({{"a", true}, {"b", false}, {"c", true}})))
    test_defaults("nonexistent", (std::map<std::string, int>({{"a", 0}, {"b", 1}, {"c", -1}})))
    test_defaults("nonexistent", (std::map<std::string, unsigned int>({{"a", 0u}, {"b", 1u}, {"c", 2u}})))
    test_defaults("nonexistent", (std::map<std::string, double>({{"a", 0.0}, {"b", 1.1}, {"c", -1.1}})))
    test_defaults("nonexistent", (std::map<std::string, std::string>({{"a", "a"}, {"b", "b"}, {"cde", "cde"}})))
    test_defaults("dict_bool", (std::map<std::string, int>({{"a", 1}})))
    test_defaults("dict_bool", (std::map<std::string, double>({{"a", 1.0}})))
    test_defaults("dict_bool", (std::map<std::string, std::string>({{"a", "a"}})))
    
    // More crazy recursive stuff
    test_defaults("nonexistent", (std::map<std::string, std::vector<int>>({{"a", {0, 1}}, {"b", {2, 3}}})))
    test_defaults("nonexistent", (std::map<std::string, std::map<std::string, std::vector<size_t>>>({
      {"a", {{"aa", {0_sz, 1_sz}}}},
      {"b", {{"bb", {2_sz, 3_sz}}}}
    })))
    // the dict contains -2 and -3, so reading that member as size_t shouldn't work; but 0 and 1 member should load
    test_s("dict_crazy", (std::map<std::string, std::map<std::string, std::vector<size_t>>>()),
      (std::map<std::string, std::map<std::string, std::vector<size_t>>>({
        {"a", {{"aa", {0_sz, 1_sz}}}}
    })), false)
    // reading as int should work
    test_s("dict_crazy", (std::map<std::string, std::map<std::string, std::vector<int>>>()),
      (std::map<std::string, std::map<std::string, std::vector<int>>>({
        {"a", {{"aa", {0, 1}}}},
        {"b", {{"bb", {-2, -3}}}}
    })), false)
    
    test_s("dict_empty", (std::unordered_map<std::string, std::string>({{"a", "a"}})), (std::unordered_map<std::string, std::string>()), false)
    test_s("dict_bool", (std::unordered_map<std::string, bool>()), (std::unordered_map<std::string, bool>({{"a", true}, {"b", false}, {"c", true}})), false)
    test_s("dict_int", (std::unordered_map<std::string, int>()), (std::unordered_map<std::string, int>({{"a", 0}, {"b", 1}, {"c", -1}})), false)
    test_s("dict_uint", (std::unordered_map<std::string, unsigned int>()), (std::unordered_map<std::string, unsigned int>({{"a", 0}, {"b", 1}, {"c", 2}})), false)
    test_s("dict_double", (std::unordered_map<std::string, double>()), (std::unordered_map<std::string, double>({{"aaa", 0.0}, {"bbb", 1.0}, {"ccc", -1.0}})), false)
    test_s("dict_str", (std::unordered_map<std::string, std::string>()), (std::unordered_map<std::string, std::string>({{"aaa", "a"}, {"bbb", "b"}, {"ccc", "c"}})), false)
    test_defaults("dict_mixed", (std::unordered_map<std::string, std::string>({{"a", "a"}})))
  }
    
  public: void testRos()
  {
   test_defaults("nonexistent", ros::Time(30))
   test_defaults("nonexistent", ros::Duration(30))
   test_defaults("nonexistent", ros::Rate(30))
   test_defaults("nonexistent", ros::WallTime(30))
   test_defaults("nonexistent", ros::WallDuration(30))
   test_defaults("nonexistent", ros::WallRate(30))
   test_defaults("nonexistent", ros::SteadyTime(30))
   
   test("double_3_14", ros::Duration(30), ros::Duration(3.14), false)
  }
    
  public: void testGeometryMsgs()
  {
    geometry_msgs::Vector3 v; v.x = 0; v.y = 1; v.z = 2;
    test_defaults("nonexistent", v)

    geometry_msgs::Vector3 v2; v2.x = M_PI;
    test_near("quat3", v, v2, false)

    geometry_msgs::Point p; p.x = 0; p.y = 1; p.z = 2;
    geometry_msgs::Point p2; p2.x = M_PI;
    test_near("quat3", p, p2, false)

    geometry_msgs::Point32 pp; pp.x = 0; pp.y = 1; pp.z = 2;
    geometry_msgs::Point32 pp2; pp2.x = M_PI;
    test_near("quat3", pp, pp2, false)

    geometry_msgs::Quaternion q; q.x = 0; q.y = 1; q.z = 2; q.w = 3; // it doesn't need to be a valid quaternion
    test_defaults("nonexistent", q)
    
    geometry_msgs::Quaternion q2; q2.x = 1;
    test_near("quat3", q, q2, false)
    test_near("quat4", q, q2, false)
    
    geometry_msgs::Transform t; t.translation = v; t.rotation = q;
    test_defaults("nonexistent", t)

    geometry_msgs::Vector3 v3; v3.x = 1; v3.y = 2; v3.z = 3;
    geometry_msgs::Transform t2; t2.translation = v3; t2.rotation = q2;
    test_near("tf6", t, t2, false)
    test_near("tf7", t, t2, false)
    test_near("tf16", t, t2, false)
  }
    
  public: void testTF2()
  {
    tf2::Vector3 v(0, 1, 2);
    test_defaults("nonexistent", v)

    tf2::Vector3 v2(M_PI, 0, 0);
    test_near("quat3", v, v2, false)

    tf2::Quaternion q(0, 1, 2, 3); // it doesn't need to be a valid quaternion
    test_defaults("nonexistent", q)
    
    tf2::Quaternion q2(1, 0, 0, 0);
    test_near("quat3", q, q2, false)
    test_near("quat4", q, q2, false)
    
    tf2::Transform t; t.setOrigin(v); t.setRotation(q);
    test_defaults("nonexistent", t)

    tf2::Vector3 v3(1, 2, 3);
    tf2::Transform t2; t2.setOrigin(v3); t2.setRotation(q2);
    test_near("tf6", t, t2, false)
    test_near("tf7", t, t2, false)
    test_near("tf16", t, t2, false)
  }
  
  public: void testNested()
  {
    test("debug/pcl/inside", true, false, false)
    test_defaults("debug/pcl/nonexistent", true)
    test("test/negative", 1, -1, false)
    test("sensor/min_distance", 0.01, 0.1, false)
    test_s("frames/fixed", std::string("fixed"), std::string("odom"), false)
    test_s("frames/fixed", "fixed", "odom", false)
    test_defaults("frames/fixed", 1) // wrong value type
    test_defaults("test/negative", static_cast<uint64_t>(1))
    test_defaults("test/negative", static_cast<unsigned int>(1))
    test_defaults("non/existent", static_cast<unsigned int>(1))
    test("transforms/buffer_length", ros::Duration(30), ros::Duration(60), false)
    test_s("ignored_links/bounding_sphere", std::vector<std::string>(), std::vector<std::string>({"antenna", "base_link::big_collision_box"}), false)
    test_s("ignored_links/bounding_sphere", std::set<std::string>(), std::set<std::string>({"antenna", "base_link::big_collision_box"}), false)
    test_s("body_model/per_link/scale", (std::map<std::string, double>()), 
      (std::map<std::string, double>({
        {"antenna::contains", 1.2},
        {"antenna::bounding_sphere", 1.2},
        {"antenna::bounding_box", 1.2},
        {"*::big_collision_box::contains", 2.0},
        {"*::big_collision_box::bounding_sphere", 2.0},
        {"*::big_collision_box::bounding_box", 2.0},
        {"*::big_collision_box::shadow", 3.0}
      })), false)
    test_s("body_model/per_link/padding", (std::map<std::string, double>()),
         (std::map<std::string, double>({{"laser::shadow", 0.015}, {"base_link", 0.05}})), false)
    test_defaults("body_model/per_link/all_wrong", (std::map<std::string, double>({{"a", 1}, {"b", 2}})))
  }
  
  public: void testOptions()
  {
    GetParamOptions<bool, bool> opts;
    GetParamResult<bool> r(false, {});
    const auto& ns = this->param->getNamespace();

    class Appender : public ros::console::LogAppender
    {
      public: void log(::ros::console::Level level, const char* str, const char* file, const char* function, int line) override
      {
        this->str = str;
        this->level = level;
      }
      public: void reset()
      {
        this->str = "";
        this->level = ::ros::console::Level::Count;
      }
      public: std::string str {};
      public: ros::console::Level level {::ros::console::Level::Count};
    };

    Appender log;
    ros::console::register_appender(&log);

    // test reading an existing param with default config
    opts = {}; log.reset();
    r = this->getParamVerbose("bool_False", true, "", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
    
    // test printing reports about using default value as info messages
    opts = {}; log.reset();
    r = this->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
    
    // test printing reports about using default value as warning messages
    opts = {}; opts.printDefaultAsWarn = true; log.reset();
    r = this->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Warn, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
    
    // test not printing log messages
    opts = {}; opts.printMessages = false; log.reset();
    r = this->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ("", log.str);
    EXPECT_EQ(ros::console::Level::Count, log.level);
    
    // required parameter is missing
    opts = {}; log.reset();
    EXPECT_THROW(this->getParamVerbose<bool>("nonexistent", std::nullopt, "", opts), GetParamException);
    try
    {
      this->getParamVerbose<bool>("nonexistent", std::nullopt, "", opts);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_TRUE(e.info.requiredMissing);
      EXPECT_FALSE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent.", e.info.message);
      EXPECT_EQ(e.info.message, log.str);
      EXPECT_EQ(e.info.messageLevel, log.level);
    }
    
    // test throwing exception on convert failure
    opts = {}; opts.throwIfConvertFails = true; log.reset();
    EXPECT_THROW(this->getParamVerbose("str_a", false, "", opts), GetParamException);
    try
    {
      this->getParamVerbose("str_a", false, "", opts);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_FALSE(e.info.requiredMissing);
      EXPECT_TRUE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                     "Expected type bool, got type string with value <value>a</value>.", e.info.message);
      EXPECT_EQ(e.info.message, log.str);
      EXPECT_EQ(e.info.messageLevel, log.level);
    }

    // test failed conversion without throwing exception
    opts = {}; opts.throwIfConvertFails = false; log.reset();
    r = this->getParamVerbose("str_a", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                   "Expected type bool, got type string with value <value>a</value>. "
                   "Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test printing custom units
    opts = {}; log.reset();
    r = this->getParamVerbose("bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test custom toString and toResult methods
    opts = {}; log.reset();
    opts.paramToStr = [](const bool& v) -> std::string {return v ? "tRuE" : "fAlSe"; };
    opts.resultToStr = [](const bool& v) -> std::string {return v ? "TRUE" : "FALSE"; };
    opts.toResult = [](const bool& v) -> bool { if (v) throw std::runtime_error("fail"); return v; };
    // reading false succeeds
    r = this->getParamVerbose("bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
    // reading true throws exception
    r = this->getParamVerbose("bool_True", false, "bools", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot convert value 'tRuE' of parameter bool_True to requested type bool (error: fail). "
                   "Assigning default: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test reading a parameter whose conversion from XmlRpc type fails
    log.reset();
    auto r2 = this->getParamVerbose("int_max", static_cast<short>(1), "", {});
    EXPECT_TRUE(r2.info.defaultUsed);
    EXPECT_FALSE(r2.info.requiredMissing);
    EXPECT_TRUE(r2.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r2.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter int_max found with correct XmlRpc type int and value "
                   "<value><i4>2147483647</i4></value>, but its conversion to type short int has failed "
                   "due to the following errors: [\"Value 2147483647 is out of bounds <-32768, 32767>.\"]. "
                   "Assigning default: 1.", r2.info.message);
    EXPECT_EQ(r2.info.message, log.str);
    EXPECT_EQ(r2.info.messageLevel, log.level);

    // test reading a parameter whose conversion from XmlRpc type fails using a custom toParam function
    opts = {}; log.reset();
    opts.toParam =
      [](const XmlRpc::XmlRpcValue& x, bool& v, bool skipNonConvertible = false, std::list<std::string>* errors = nullptr) -> bool {
        if (static_cast<bool>(x))
        {
          if (errors)
            errors->push_back("cannot convert");
          return false;
        }
        v = false;
        return true;
      };
    r = this->getParamVerbose("bool_True", false, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter bool_True found with correct XmlRpc type bool and value "
                   "<value><boolean>1</boolean></value>, but its conversion to type bool has failed "
                   "due to the following errors: [\"cannot convert\"]. "
                   "Assigning default: False.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
  }
};

std::shared_ptr<TestFilter> getFilter(bool paramsFromChain)
{
  static std::map<bool, std::shared_ptr<TestFilter>> cache;
  if (cache.find(paramsFromChain) != cache.end())
    return cache[paramsFromChain];
  
  static ros::NodeHandle nh;  // without the static modifier, rosconsole logging turns off and logging tests fail
  auto filter = std::make_shared<TestFilter>(paramsFromChain);
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<std::string>>(filter);
  if (paramsFromChain)
  {
    XmlRpc::XmlRpcValue value;
    nh.getParam("test_chain_config", value);
    filterBase->configure(value[0]);
  }
  else
  {
    // rosparam load cannot load empty dict parameter from the YAML file
    nh.setParam("test_dict_config/params/dict_empty", std::map<std::string, std::string>());
    filterBase->configure("test_dict_config", nh);
  }
  cache[paramsFromChain] = filter;

  return filter;
}

TEST_P(FilterUtils, XmlRpcValue)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testXmlRpcValue();
}

TEST_P(FilterUtils, Bool)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testBool();
}

TEST_P(FilterUtils, Int)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testInt();
}

TEST_P(FilterUtils, Float)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testFloat();
}

TEST_P(FilterUtils, Double)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testDouble();
}

TEST_P(FilterUtils, Cstring)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testCstring();
}

TEST_P(FilterUtils, String)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testString();
}

TEST_P(FilterUtils, List)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testList();
}

TEST_P(FilterUtils, Dict)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testDict();
}

TEST_P(FilterUtils, Ros)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testRos();
}

TEST_P(FilterUtils, GeometryMsgs)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testGeometryMsgs();
}

TEST_P(FilterUtils, TF2)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testTF2();
}

TEST_P(FilterUtils, Nested)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testNested();
}

TEST_P(FilterUtils, Options)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testOptions();
}

INSTANTIATE_TEST_CASE_P(TestConfigs, FilterUtils, ::testing::Values(false, true));

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_filter_utils");
  return RUN_ALL_TESTS();
}