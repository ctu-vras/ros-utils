/**
 * \file
 * \brief A template for performing tests of various getParam() variants.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>
#include <ros/rate.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <XmlRpcValue.h>

#include "log_appender.inc"

double distance(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2)
{
  return std::sqrt(std::pow(q1.x - q2.x, 2) + std::pow(q1.y - q2.y, 2) +
    std::pow(q1.z - q2.z, 2) + std::pow(q1.w - q2.w, 2));
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

template <typename Scalar>
double distance(const Eigen::Quaternion<Scalar>& q1, const Eigen::Quaternion<Scalar>& q2)
{
  return q1.angularDistance(q2);
}

template <typename Scalar, int Rows, int Cols, int Options, int RowsAtCompileTime, int ColsAtCompileTime>
double distance(const Eigen::Matrix<Scalar, Rows, Cols, Options, RowsAtCompileTime, ColsAtCompileTime>& v1,
  const Eigen::Matrix<Scalar, Rows, Cols, Options, RowsAtCompileTime, ColsAtCompileTime>& v2)
{
  return (v1 - v2).norm();
}

template <typename XprType, int BlockRows, int BlockCols, bool InnerPanel>
double distance(const Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>& v1,
  const Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>& v2)
{
  if (v1.rows() != v2.rows() || v1.cols() != v2.cols())
    return std::numeric_limits<double>::quiet_NaN();
  double d = 0;
  for (Eigen::Index r = 0; r < v1.rows(); ++r)
    for (Eigen::Index c = 0; c < v1.cols(); ++c)
      d += std::pow(v1.coeff(r, c) - v2.coeff(r, c), 2);
  return std::sqrt(d);
}

template <typename Scalar>
double distance(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& v1,
  const Eigen::Transform<Scalar, 3, Eigen::Isometry>& v2)
{
  return distance(v1.translation(), v2.translation()) +
    distance(Eigen::Quaternion<Scalar>(v1.linear()), Eigen::Quaternion<Scalar>(v2.linear()));
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

namespace Eigen
{
template <typename Scalar>
bool operator==(const Eigen::Quaternion<Scalar>& q1, const Eigen::Quaternion<Scalar>& q2)
{
  return q1.angularDistance(q2) == 0;
}
template <typename Scalar>
bool operator==(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& i1,
  const Eigen::Transform<Scalar, 3, Eigen::Isometry>& i2)
{
  return i1.translation() == i2.translation() &&
    Eigen::Quaternion<Scalar>(i1.linear()).angularDistance(Eigen::Quaternion<Scalar>(i2.linear())) == 0;
}
template <typename Scalar, int Rows, int Cols, int Options, int RowsAtCompileTime, int ColsAtCompileTime>
bool operator==(const Eigen::Matrix<Scalar, Rows, Cols, Options, RowsAtCompileTime, ColsAtCompileTime>& m1,
  const Eigen::Matrix<Scalar, Rows, Cols, Options, RowsAtCompileTime, ColsAtCompileTime>& m2)
{
  return distance(m1, m2) == 0;
}
}

#define TRACE SCOPED_TRACE("Failure happened here");

/**
 * \brief A template for performing tests of various getParam() variants. This class should be extended by a
 *        class that defines functions `test()`, `test_s()`, `test_near()` and `test_defaults()`.
 * \tparam T The test class defining functions `test()`, `test_s()`, `test_near()` and `test_defaults()`.
 */
template<typename T>
struct GetParamTest
{
  inline void testXmlRpcValue(const bool supportsEmptyDictKeys = false)
  {
    XmlRpc::XmlRpcValue dummyXml0(false);
    XmlRpc::XmlRpcValue dummyXml1(true);
  
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test("bool_True", dummyXml0, XmlRpc::XmlRpcValue(true), false);}
    {TRACE ((T*)this)->test("bool_False", dummyXml1, XmlRpc::XmlRpcValue(false), false);}
    {TRACE ((T*)this)->test("bool_true", dummyXml0, XmlRpc::XmlRpcValue(true), false);}
    {TRACE ((T*)this)->test("bool_false", dummyXml1, XmlRpc::XmlRpcValue(false), false);}
    // the XmlRpc type of the default value is not checked; 
    {TRACE ((T*)this)->test("int_0", dummyXml1, XmlRpc::XmlRpcValue(0), false);}
    {TRACE ((T*)this)->test("int_1", dummyXml0, XmlRpc::XmlRpcValue(1), false);}
    {TRACE ((T*)this)->test("int_2", dummyXml0, XmlRpc::XmlRpcValue(2), false);}
    {TRACE ((T*)this)->test("int_minus_1", dummyXml0, XmlRpc::XmlRpcValue(-1), false);}
    {TRACE ((T*)this)->test("int_max", dummyXml0, XmlRpc::XmlRpcValue(INT_MAX), false);}
    {TRACE ((T*)this)->test("int_min", dummyXml0, XmlRpc::XmlRpcValue(INT_MIN), false);}
    {TRACE ((T*)this)->test("double_0", dummyXml0, XmlRpc::XmlRpcValue(0.0), false);}
    {TRACE ((T*)this)->test("double_1", dummyXml0, XmlRpc::XmlRpcValue(1.0), false);}
    {TRACE ((T*)this)->test("double_3_14", dummyXml0, XmlRpc::XmlRpcValue(3.14), false);}
    {TRACE ((T*)this)->test("double_minus_1", dummyXml0, XmlRpc::XmlRpcValue(-1.0), false);}
    {TRACE ((T*)this)->test("double_minus_3_14", dummyXml0, XmlRpc::XmlRpcValue(-3.14), false);}
    {TRACE ((T*)this)->test("str_empty", dummyXml0, XmlRpc::XmlRpcValue(""), false);}
    {TRACE ((T*)this)->test("str_a", dummyXml0, XmlRpc::XmlRpcValue("a"), false);}
    {TRACE ((T*)this)->test("str_asd", dummyXml0, XmlRpc::XmlRpcValue("asd"), false);}
    {XmlRpc::XmlRpcValue v; v.setSize(0);
      {TRACE ((T*)this)->test("list_empty", dummyXml0, v, false);}}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = true; v[1] = false; v[2] = true;
      {TRACE ((T*)this)->test("list_bool", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = 0; v[1] = 1; v[2] = -1;
      {TRACE ((T*)this)->test("list_int", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = 0; v[1] = 1; v[2] = 2;
      {TRACE ((T*)this)->test("list_uint", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = 0.0; v[1] = 1.0; v[2] = -1.0;
      {TRACE ((T*)this)->test("list_double", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v.setSize(3); v[0] = "a"; v[1] = "b"; v[2] = "cde";
      {TRACE ((T*)this)->test("list_str", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v.begin();
      {TRACE ((T*)this)->test("dict_empty", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v["a"] = true; v["b"] = false; v["c"] = true;
      {TRACE ((T*)this)->test("dict_bool", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v["a"] = 0; v["b"] = 1; v["c"] = -1;
      {TRACE ((T*)this)->test("dict_int", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v["a"] = 0; v["b"] = 1; v["c"] = 2;
      {TRACE ((T*)this)->test("dict_uint", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v["aaa"] = 0.0; v["bbb"] = 1.0; v["ccc"] = -1.0;
      {TRACE ((T*)this)->test("dict_double", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {XmlRpc::XmlRpcValue v; v["aaa"] = "a"; v["bbb"] = "b"; v["ccc"] = "c";
      {TRACE ((T*)this)->test("dict_str", dummyXml0, XmlRpc::XmlRpcValue(v), false);}}
    {;
      XmlRpc::XmlRpcValue v; v["bbb"] = 1.0; v["ccc"] = false;
      if (supportsEmptyDictKeys)
        v[""] = 0;
      {TRACE ((T*)this)->test_s("dict_mixed", dummyXml0, XmlRpc::XmlRpcValue(v), false);}
    };
    {;
      XmlRpc::XmlRpcValue va; va["b"] = 1; va["c"] = 2;
      XmlRpc::XmlRpcValue vd; vd["e"] = 3; vd["f"] = 4; vd["g"] = 5;
      XmlRpc::XmlRpcValue v; v["a"] = va; v["d"] = vd;
      {TRACE ((T*)this)->test_s("dict_str_recursive", dummyXml0, XmlRpc::XmlRpcValue(v), false);}
    };
    {TRACE ((T*)this)->test_defaults("nonexistent", dummyXml0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", dummyXml1);}
  }
  
  inline void testBool()
  {
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test("bool_True", false, true, false);}
    {TRACE ((T*)this)->test("bool_False", true, false, false);}
    {TRACE ((T*)this)->test("bool_true", false, true, false);}
    {TRACE ((T*)this)->test("bool_false", true, false, false);}
    {TRACE ((T*)this)->test("int_0", true, false, false);}
    {TRACE ((T*)this)->test("int_1", false, true, false);}
    {TRACE ((T*)this)->test_defaults("int_2", false);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", false);}
    {TRACE ((T*)this)->test_defaults("int_max", false);}
    {TRACE ((T*)this)->test_defaults("int_min", false);}
    {TRACE ((T*)this)->test_defaults("double_0", true);}
    {TRACE ((T*)this)->test_defaults("double_1", false);}
    {TRACE ((T*)this)->test_defaults("double_3_14", false);}
    {TRACE ((T*)this)->test_defaults("double_minus_1", false);}
    {TRACE ((T*)this)->test_defaults("double_minus_3_14", false);}
    {TRACE ((T*)this)->test_defaults("str_empty", true);}
    {TRACE ((T*)this)->test_defaults("str_a", false);}
    {TRACE ((T*)this)->test_defaults("str_asd", false);}
    {TRACE ((T*)this)->test_defaults("list_empty", true);}
    {TRACE ((T*)this)->test_defaults("list_bool", false);}
    {TRACE ((T*)this)->test_defaults("list_int", false);}
    {TRACE ((T*)this)->test_defaults("list_uint", false);}
    {TRACE ((T*)this)->test_defaults("list_double", false);}
    {TRACE ((T*)this)->test_defaults("list_str", false);}
    {TRACE ((T*)this)->test_defaults("dict_empty", true);}
    {TRACE ((T*)this)->test_defaults("dict_bool", false);}
    {TRACE ((T*)this)->test_defaults("dict_int", false);}
    {TRACE ((T*)this)->test_defaults("dict_uint", false);}
    {TRACE ((T*)this)->test_defaults("dict_double", false);}
    {TRACE ((T*)this)->test_defaults("dict_str", false);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", false);}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", false);}
    {TRACE ((T*)this)->test_defaults("nonexistent", false);}
    {TRACE ((T*)this)->test_defaults("nonexistent", true);}
  }

  inline void testInt()
  {
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test_defaults("bool_True", 0);}
    {TRACE ((T*)this)->test_defaults("bool_False", 1);}
    {TRACE ((T*)this)->test_defaults("bool_true", 0);}
    {TRACE ((T*)this)->test_defaults("bool_false", 1);}
    {TRACE ((T*)this)->test("int_0", 1, 0, false);}
    {TRACE ((T*)this)->test("int_1", 0, 1, false);}
    {TRACE ((T*)this)->test("int_2", 0, 2, false);}
    {TRACE ((T*)this)->test("int_minus_1", 0, -1, false);}
    {TRACE ((T*)this)->test("int_max", 0, INT_MAX, false);}
    {TRACE ((T*)this)->test("int_min", 0, INT_MIN, false);}
    {TRACE ((T*)this)->test_defaults("double_0", 1);}
    {TRACE ((T*)this)->test_defaults("double_1", 0);}
    {TRACE ((T*)this)->test_defaults("double_3_14", 0);}
    {TRACE ((T*)this)->test_defaults("double_minus_1", 0);}
    {TRACE ((T*)this)->test_defaults("double_minus_3_14", 0);}
    {TRACE ((T*)this)->test_defaults("double_lowest", 0);}
    {TRACE ((T*)this)->test_defaults("double_min", 0);}
    {TRACE ((T*)this)->test_defaults("double_max", 0);}
    {TRACE ((T*)this)->test_defaults("str_empty", 1);}
    {TRACE ((T*)this)->test_defaults("str_a", 0);}
    {TRACE ((T*)this)->test_defaults("str_asd", 0);}
    {TRACE ((T*)this)->test_defaults("list_empty", 0);}
    {TRACE ((T*)this)->test_defaults("list_bool", 0);}
    {TRACE ((T*)this)->test_defaults("list_int", 0);}
    {TRACE ((T*)this)->test_defaults("list_uint", 0);}
    {TRACE ((T*)this)->test_defaults("list_double", 0);}
    {TRACE ((T*)this)->test_defaults("list_str", 0);}
    {TRACE ((T*)this)->test_defaults("dict_empty", 0);}
    {TRACE ((T*)this)->test_defaults("dict_bool", 0);}
    {TRACE ((T*)this)->test_defaults("dict_int", 0);}
    {TRACE ((T*)this)->test_defaults("dict_uint", 0);}
    {TRACE ((T*)this)->test_defaults("dict_double", 0);}
    {TRACE ((T*)this)->test_defaults("dict_str", 0);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", 0);}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", 0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", 0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", 1);}
    {TRACE ((T*)this)->test_defaults("nonexistent", -1);}
    {TRACE ((T*)this)->test_defaults("nonexistent", INT_MAX);}
    {TRACE ((T*)this)->test_defaults("nonexistent", INT_MIN);}
    {TRACE ((T*)this)->test_defaults("nonexistent", (long)INT_MAX+1);}

    {TRACE ((T*)this)->test("int_0", 1u, 0u, false);}
    {TRACE ((T*)this)->test("int_1", 0u, 1u, false);}
    {TRACE ((T*)this)->test("int_2", 0u, 2u, false);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", 0u);}
    {TRACE ((T*)this)->test("int_max", 0u, (unsigned int)INT_MAX, false);}
    {TRACE ((T*)this)->test_defaults("int_min", 0u);}

    {TRACE ((T*)this)->test("int_0", (signed char)1, (signed char)0, false);}
    {TRACE ((T*)this)->test("int_1", (signed char)0, (signed char)1, false);}
    {TRACE ((T*)this)->test("int_2", (signed char)0, (signed char)2, false);}
    {TRACE ((T*)this)->test("int_minus_1", (signed char)0, (signed char)-1, false);}
    {TRACE ((T*)this)->test_defaults("int_max", (signed char)0);}
    {TRACE ((T*)this)->test_defaults("int_min", (signed char)0);}

    {TRACE ((T*)this)->test("int_0", (unsigned char)1, (unsigned char)0, false);}
    {TRACE ((T*)this)->test("int_1", (unsigned char)0, (unsigned char)1, false);}
    {TRACE ((T*)this)->test("int_2", (unsigned char)0, (unsigned char)2, false);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", (unsigned char)0);}
    {TRACE ((T*)this)->test_defaults("int_max", (unsigned char)0);}
    {TRACE ((T*)this)->test_defaults("int_min", (unsigned char)0);}

    {TRACE ((T*)this)->test("int_0", (short)1, (short)0, false);}
    {TRACE ((T*)this)->test("int_1", (short)0, (short)1, false);}
    {TRACE ((T*)this)->test("int_2", (short)0, (short)2, false);}
    {TRACE ((T*)this)->test("int_minus_1", (short)0, (short)-1, false);}
    {TRACE ((T*)this)->test_defaults("int_max", (short)0);}
    {TRACE ((T*)this)->test_defaults("int_min", (short)0);}

    {TRACE ((T*)this)->test("int_0", (unsigned short)1, (unsigned short)0, false);}
    {TRACE ((T*)this)->test("int_1", (unsigned short)0, (unsigned short)1, false);}
    {TRACE ((T*)this)->test("int_2", (unsigned short)0, (unsigned short)2, false);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", (unsigned short)0);}
    {TRACE ((T*)this)->test_defaults("int_max", (unsigned short)0);}
    {TRACE ((T*)this)->test_defaults("int_min", (unsigned short)0);}

    {TRACE ((T*)this)->test("int_0", (long)1, (long)0, false);}
    {TRACE ((T*)this)->test("int_1", (long)0, (long)1, false);}
    {TRACE ((T*)this)->test("int_2", (long)0, (long)2, false);}
    {TRACE ((T*)this)->test("int_minus_1", (long)0, (long)-1, false);}
    {TRACE ((T*)this)->test("int_max", (long)0, (long)INT_MAX, false);}
    {TRACE ((T*)this)->test("int_min", (long)0, (long)INT_MIN, false);}

    {TRACE ((T*)this)->test("int_0", (unsigned long)1, (unsigned long)0, false);}
    {TRACE ((T*)this)->test("int_1", (unsigned long)0, (unsigned long)1, false);}
    {TRACE ((T*)this)->test("int_2", (unsigned long)0, (unsigned long)2, false);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", (unsigned long)0);}
    {TRACE ((T*)this)->test("int_max", (unsigned long)0, (unsigned long)INT_MAX, false);}
    {TRACE ((T*)this)->test_defaults("int_min", (unsigned long)0);}

    {TRACE ((T*)this)->test("int_0", (long long)1, (long long)0, false);}
    {TRACE ((T*)this)->test("int_1", (long long)0, (long long)1, false);}
    {TRACE ((T*)this)->test("int_2", (long long)0, (long long)2, false);}
    {TRACE ((T*)this)->test("int_minus_1", (long long)0, (long long)-1, false);}
    {TRACE ((T*)this)->test("int_max", (long long)0, (long long)INT_MAX, false);}
    {TRACE ((T*)this)->test("int_min", (long long)0, (long long)INT_MIN, false);}

    {TRACE ((T*)this)->test("int_0", (unsigned long long)1, (unsigned long long)0, false);}
    {TRACE ((T*)this)->test("int_1", (unsigned long long)0, (unsigned long long)1, false);}
    {TRACE ((T*)this)->test("int_2", (unsigned long long)0, (unsigned long long)2, false);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", (unsigned long long)0);}
    {TRACE ((T*)this)->test("int_max", (unsigned long long)0, (unsigned long long)INT_MAX, false);}
    {TRACE ((T*)this)->test_defaults("int_min", (unsigned long long)0);}
  }

  inline void testFloat()
  {
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test_defaults("bool_True", 0.0f);}
    {TRACE ((T*)this)->test_defaults("bool_False", 1.0f);}
    {TRACE ((T*)this)->test_defaults("bool_true", 0.0f);}
    {TRACE ((T*)this)->test_defaults("bool_false", 1.0f);}
    {TRACE ((T*)this)->test("int_0", 1.0f, 0.0f, false);}
    {TRACE ((T*)this)->test("int_1", 0.0f, 1.0f, false);}
    {TRACE ((T*)this)->test("int_2", 0.0f, 2.0f, false);}
    {TRACE ((T*)this)->test("int_minus_1", 0.0f, -1.0f, false);}
    {TRACE ((T*)this)->test("int_max", 0.0f, (float)INT_MAX, false);}
    {TRACE ((T*)this)->test("int_min", 0.0f, (float)INT_MIN, false);}
    {TRACE ((T*)this)->test("double_0", 1.0f, 0.0f, false);}
    {TRACE ((T*)this)->test("double_1", 0.0f, 1.0f, false);}
    {TRACE ((T*)this)->test("double_3_14", 0.0f, 3.14f, false);}
    {TRACE ((T*)this)->test("double_minus_1", 0.0f, -1.0f, false);}
    {TRACE ((T*)this)->test("double_minus_3_14", 0.0f, -3.14f, false);}
    {TRACE ((T*)this)->test_defaults("double_lowest", 0.0f);}
    {TRACE ((T*)this)->test("double_min", 0.0f, 0.0f, false);}
    {TRACE ((T*)this)->test_defaults("double_max", 0.0f);}
    {TRACE ((T*)this)->test("double_inf", 0.0f, std::numeric_limits<float>::infinity(), false);}
    {TRACE ((T*)this)->test("double_minus_inf", 0.0f, -std::numeric_limits<float>::infinity(), false);}
    {TRACE ((T*)this)->test_defaults("str_empty", 1.0f);}
    {TRACE ((T*)this)->test_defaults("str_a", 0.0f);}
    {TRACE ((T*)this)->test_defaults("str_asd", 0.0f);}
    {TRACE ((T*)this)->test_defaults("list_empty", 0.0f);}
    {TRACE ((T*)this)->test_defaults("list_bool", 0.0f);}
    {TRACE ((T*)this)->test_defaults("list_int", 0.0f);}
    {TRACE ((T*)this)->test_defaults("list_uint", 0.0f);}
    {TRACE ((T*)this)->test_defaults("list_double", 0.0f);}
    {TRACE ((T*)this)->test_defaults("list_str", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_empty", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_bool", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_int", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_uint", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_double", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_str", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", 0.0f);}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", 0.0f);}
    {TRACE ((T*)this)->test_defaults("nonexistent", 0.0f);}
    {TRACE ((T*)this)->test_defaults("nonexistent", 1.0f);}
    {TRACE ((T*)this)->test_defaults("nonexistent", -1.0f);}
    {TRACE ((T*)this)->test_defaults("nonexistent", FLT_MAX);}
    {TRACE ((T*)this)->test_defaults("nonexistent", FLT_MIN);}
  }

  inline void testDouble()
  {
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test_defaults("bool_True", 0.0);}
    {TRACE ((T*)this)->test_defaults("bool_False", 1.0);}
    {TRACE ((T*)this)->test_defaults("bool_true", 0.0);}
    {TRACE ((T*)this)->test_defaults("bool_false", 1.0);}
    {TRACE ((T*)this)->test("int_0", 1.0, 0.0, false);}
    {TRACE ((T*)this)->test("int_1", 0.0, 1.0, false);}
    {TRACE ((T*)this)->test("int_2", 0.0, 2.0, false);}
    {TRACE ((T*)this)->test("int_minus_1", 0.0, -1.0, false);}
    {TRACE ((T*)this)->test("int_max", 0.0, (double)INT_MAX, false);}
    {TRACE ((T*)this)->test("int_min", 0.0, (double)INT_MIN, false);}
    {TRACE ((T*)this)->test("double_0", 1.0, 0.0, false);}
    {TRACE ((T*)this)->test("double_1", 0.0, 1.0, false);}
    {TRACE ((T*)this)->test("double_3_14", 0.0, 3.14, false);}
    {TRACE ((T*)this)->test("double_minus_1", 0.0, -1.0, false);}
    {TRACE ((T*)this)->test("double_minus_3_14", 0.0, -3.14, false);}
    {TRACE ((T*)this)->test("double_lowest", 0.0, -1.79769e+308, false);}
    {TRACE ((T*)this)->test("double_min", 0.0, 2.22507e-308, false);}
    {TRACE ((T*)this)->test("double_max", 0.0, 1.79769e+308, false);}
    {TRACE ((T*)this)->test("double_inf", 0.0, std::numeric_limits<double>::infinity(), false);}
    {TRACE ((T*)this)->test("double_minus_inf", 0.0, -std::numeric_limits<double>::infinity(), false);}
    {TRACE ((T*)this)->test_defaults("str_empty", 1.0);}
    {TRACE ((T*)this)->test_defaults("str_a", 0.0);}
    {TRACE ((T*)this)->test_defaults("str_asd", 0.0);}
    {TRACE ((T*)this)->test_defaults("list_empty", 0.0);}
    {TRACE ((T*)this)->test_defaults("list_bool", 0.0);}
    {TRACE ((T*)this)->test_defaults("list_int", 0.0);}
    {TRACE ((T*)this)->test_defaults("list_uint", 0.0);}
    {TRACE ((T*)this)->test_defaults("list_double", 0.0);}
    {TRACE ((T*)this)->test_defaults("list_str", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_empty", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_bool", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_int", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_uint", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_double", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_str", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", 0.0);}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", 0.0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", 0.0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", 1.0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", -1.0);}
    {TRACE ((T*)this)->test_defaults("nonexistent", DBL_MAX);}
    {TRACE ((T*)this)->test_defaults("nonexistent", DBL_MIN);}

    {TRACE ((T*)this)->test("int_0", (long double)1.0, (long double)0.0, false);}
    {TRACE ((T*)this)->test("int_1", (long double)0.0, (long double)1.0, false);}
    {TRACE ((T*)this)->test("int_2", (long double)0.0, (long double)2.0, false);}
    {TRACE ((T*)this)->test("int_minus_1", (long double)0.0, (long double)-1.0, false);}
    {TRACE ((T*)this)->test("int_max", (long double)0.0, (long double)INT_MAX, false);}
    {TRACE ((T*)this)->test("int_min", (long double)0.0, (long double)INT_MIN, false);}
    {TRACE ((T*)this)->test("double_0", (long double)1.0, (long double)0.0, false);}
    {TRACE ((T*)this)->test("double_1", (long double)0.0, (long double)1.0, false);}
    {TRACE ((T*)this)->test("double_3_14", (long double)0.0, (long double)3.14, false);}
    {TRACE ((T*)this)->test("double_minus_1", (long double)0.0, (long double)-1.0, false);}
    {TRACE ((T*)this)->test("double_minus_3_14", (long double)0.0, (long double)-3.14, false);}
    {TRACE ((T*)this)->test("double_lowest", (long double)0.0, (long double)-1.79769e+308, false);}
    {TRACE ((T*)this)->test("double_min", (long double)0.0, (long double)2.22507e-308, false);}
    {TRACE ((T*)this)->test("double_max", (long double)0.0, (long double)1.79769e+308, false);}
    {TRACE ((T*)this)->test("double_inf", (long double)0.0, std::numeric_limits<long double>::infinity(), false);}
    {TRACE ((T*)this)->test("double_minus_inf", (long double)0.0, -std::numeric_limits<long double>::infinity(), false);}
  }

  inline void testCstring()
  {
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test_defaults("bool_True", "0");}
    {TRACE ((T*)this)->test_defaults("bool_False", "1");}
    {TRACE ((T*)this)->test_defaults("bool_true", "0");}
    {TRACE ((T*)this)->test_defaults("bool_false", "1");}
    {TRACE ((T*)this)->test_defaults("int_0", "1");}
    {TRACE ((T*)this)->test_defaults("int_1", "0");}
    {TRACE ((T*)this)->test_defaults("int_2", "0");}
    {TRACE ((T*)this)->test_defaults("int_minus_1", "0");}
    {TRACE ((T*)this)->test_defaults("int_max", "0");}
    {TRACE ((T*)this)->test_defaults("int_min", "0");}
    {TRACE ((T*)this)->test_defaults("double_0", "1");}
    {TRACE ((T*)this)->test_defaults("double_1", "0");}
    {TRACE ((T*)this)->test_defaults("double_3_14", "0");}
    {TRACE ((T*)this)->test_defaults("double_minus_1", "0");}
    {TRACE ((T*)this)->test_defaults("double_minus_3_14", "0");}
    {TRACE ((T*)this)->test_s("str_empty", "a", "", false);}
    {TRACE ((T*)this)->test_s("str_a", "", "a", false);}
    {TRACE ((T*)this)->test_s("str_asd", "a", "asd", false);}
    {TRACE ((T*)this)->test_defaults("list_empty", "0");}
    {TRACE ((T*)this)->test_defaults("list_bool", "0");}
    {TRACE ((T*)this)->test_defaults("list_int", "0");}
    {TRACE ((T*)this)->test_defaults("list_uint", "0");}
    {TRACE ((T*)this)->test_defaults("list_double", "0");}
    {TRACE ((T*)this)->test_defaults("list_str", "0");}
    {TRACE ((T*)this)->test_defaults("dict_empty", "0");}
    {TRACE ((T*)this)->test_defaults("dict_bool", "0");}
    {TRACE ((T*)this)->test_defaults("dict_int", "0");}
    {TRACE ((T*)this)->test_defaults("dict_uint", "0");}
    {TRACE ((T*)this)->test_defaults("dict_double", "0");}
    {TRACE ((T*)this)->test_defaults("dict_str", "0");}
    {TRACE ((T*)this)->test_defaults("dict_mixed", "0");}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", "0");}
    {TRACE ((T*)this)->test_defaults("nonexistent", "");}
    {TRACE ((T*)this)->test_defaults("nonexistent", "a");}
    {TRACE ((T*)this)->test_defaults("nonexistent", "asd");}
  }

  inline void testString()
  {
    // test(param, def, expected, defUsed)
    {TRACE ((T*)this)->test_defaults("bool_True", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("bool_False", std::string("1"));}
    {TRACE ((T*)this)->test_defaults("bool_true", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("bool_false", std::string("1"));}
    {TRACE ((T*)this)->test_defaults("int_0", std::string("1"));}
    {TRACE ((T*)this)->test_defaults("int_1", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("int_2", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("int_minus_1", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("int_max", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("int_min", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("double_0", std::string("1"));}
    {TRACE ((T*)this)->test_defaults("double_1", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("double_3_14", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("double_minus_1", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("double_minus_3_14", std::string("0"));}
    {TRACE ((T*)this)->test_s("str_empty", std::string("a"), std::string(""), false);}
    {TRACE ((T*)this)->test_s("str_a", std::string(""), std::string("a"), false);}
    {TRACE ((T*)this)->test_s("str_asd", std::string("a"), std::string("asd"), false);}
    {TRACE ((T*)this)->test_defaults("list_empty", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("list_bool", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("list_int", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("list_uint", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("list_double", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("list_str", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_empty", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_bool", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_int", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_uint", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_double", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_str", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_mixed", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", std::string("0"));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::string(""));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::string("a"));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::string("asd"));}
  }

  inline void testList()
  {
    // test(param, def, expected, defUsed)
    const std::vector<std::string> dummyVec({"a"});;
    {TRACE ((T*)this)->test_defaults("bool_True", dummyVec);}
    {TRACE ((T*)this)->test_defaults("bool_False", dummyVec);}
    {TRACE ((T*)this)->test_defaults("bool_true", dummyVec);}
    {TRACE ((T*)this)->test_defaults("bool_false", dummyVec);}
    {TRACE ((T*)this)->test_defaults("int_0", dummyVec);}
    {TRACE ((T*)this)->test_defaults("int_1", dummyVec);}
    {TRACE ((T*)this)->test_defaults("int_2", dummyVec);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", dummyVec);}
    {TRACE ((T*)this)->test_defaults("int_max", dummyVec);}
    {TRACE ((T*)this)->test_defaults("int_min", dummyVec);}
    {TRACE ((T*)this)->test_defaults("double_0", dummyVec);}
    {TRACE ((T*)this)->test_defaults("double_1", dummyVec);}
    {TRACE ((T*)this)->test_defaults("double_3_14", dummyVec);}
    {TRACE ((T*)this)->test_defaults("double_minus_1", dummyVec);}
    {TRACE ((T*)this)->test_defaults("double_minus_3_14", dummyVec);}
    {TRACE ((T*)this)->test_defaults("str_empty", dummyVec);}
    {TRACE ((T*)this)->test_defaults("str_a", dummyVec);}
    {TRACE ((T*)this)->test_defaults("str_asd", dummyVec);}
    {TRACE ((T*)this)->test_s("list_empty", std::vector<std::string>({"a"}), std::vector<std::string>(), false);}
    {TRACE ((T*)this)->test_s("list_bool", std::vector<bool>(), std::vector<bool>({true, false, true}), false);}
    {TRACE ((T*)this)->test_s("list_int", std::vector<int>(), std::vector<int>({0, 1, -1}), false);}
    {TRACE ((T*)this)->test_s("list_uint", std::vector<unsigned int>(), std::vector<unsigned int>({0u, 1u, 2u}), false);}
    {TRACE ((T*)this)->test_s("list_double", std::vector<double>(), std::vector<double>({0.0, 1.0, -1.0}), false);}
    {TRACE ((T*)this)->test_s(
      "list_str", std::vector<std::string>(), std::vector<std::string>({"a", "b", "cde"}), false);}
    {TRACE ((T*)this)->test_defaults("dict_empty", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_bool", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_int", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_uint", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_double", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_str", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", dummyVec);}
    {TRACE ((T*)this)->test_defaults("dict_str_recursive", dummyVec);}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::vector<std::string>());}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::vector<bool>({true, false, true}));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::vector<int>({0, 1, -1}));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::vector<unsigned int>({0u, 1u, 2u}));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::vector<double>({0.0, 1.0, -1.0}));}
    {TRACE ((T*)this)->test_defaults("nonexistent", std::vector<std::string>({"a", "b", "cde"}));}

    {TRACE ((T*)this)->test_s("list_empty", std::list<std::string>({"a"}), std::list<std::string>(), false);}
    {TRACE ((T*)this)->test_s("list_bool", std::list<bool>(), std::list<bool>({true, false, true}), false);}
    {TRACE ((T*)this)->test_s("list_int", std::list<int>(), std::list<int>({0, 1, -1}), false);}
    {TRACE ((T*)this)->test_s("list_uint", std::list<unsigned int>(), std::list<unsigned int>({0u, 1u, 2u}), false);}
    {TRACE ((T*)this)->test_s("list_double", std::list<double>(), std::list<double>({0.0, 1.0, -1.0}), false);}
    {TRACE ((T*)this)->test_s("list_str", std::list<std::string>(), std::list<std::string>({"a", "b", "cde"}), false);}

    {TRACE ((T*)this)->test_s("list_empty", std::set<std::string>({"a"}), std::set<std::string>(), false);}
    {TRACE ((T*)this)->test_s("list_bool", std::set<bool>(), std::set<bool>({true, false, true}), false);}
    {TRACE ((T*)this)->test_s("list_int", std::set<int>(), std::set<int>({0, 1, -1}), false);}
    {TRACE ((T*)this)->test_s("list_uint", std::set<unsigned int>(), std::set<unsigned int>({0u, 1u, 2u}), false);}
    {TRACE ((T*)this)->test_s("list_double", std::set<double>(), std::set<double>({0.0, 1.0, -1.0}), false);}
    {TRACE ((T*)this)->test_s("list_str", std::set<std::string>(), std::set<std::string>({"a", "b", "cde"}), false);}

    {TRACE ((T*)this)->test_s(
      "list_empty", std::unordered_set<std::string>({"a"}), std::unordered_set<std::string>(), false);}
    {TRACE ((T*)this)->test_s(
      "list_bool", std::unordered_set<bool>(), std::unordered_set<bool>({true, false, true}), false);}
    {TRACE ((T*)this)->test_s(
      "list_int", std::unordered_set<int>(), std::unordered_set<int>({0, 1, -1}), false);}
    {TRACE ((T*)this)->test_s(
      "list_uint", std::unordered_set<unsigned int>(), std::unordered_set<unsigned int>({0u, 1u, 2u}), false);}
    {TRACE ((T*)this)->test_s(
      "list_double", std::unordered_set<double>(), std::unordered_set<double>({0.0, 1.0, -1.0}), false);}
    {TRACE ((T*)this)->test_s(
      "list_str", std::unordered_set<std::string>(), std::unordered_set<std::string>({"a", "b", "cde"}), false);}
  }

  inline void testDict()
  {
    // test(param, def, expected, defUsed)
    const std::map<std::string, std::string> dummyMap({{"a", "a"}});;
    {TRACE ((T*)this)->test_defaults("bool_True", dummyMap);}
    {TRACE ((T*)this)->test_defaults("bool_False", dummyMap);}
    {TRACE ((T*)this)->test_defaults("bool_true", dummyMap);}
    {TRACE ((T*)this)->test_defaults("bool_false", dummyMap);}
    {TRACE ((T*)this)->test_defaults("int_0", dummyMap);}
    {TRACE ((T*)this)->test_defaults("int_1", dummyMap);}
    {TRACE ((T*)this)->test_defaults("int_2", dummyMap);}
    {TRACE ((T*)this)->test_defaults("int_minus_1", dummyMap);}
    {TRACE ((T*)this)->test_defaults("int_max", dummyMap);}
    {TRACE ((T*)this)->test_defaults("int_min", dummyMap);}
    {TRACE ((T*)this)->test_defaults("double_0", dummyMap);}
    {TRACE ((T*)this)->test_defaults("double_1", dummyMap);}
    {TRACE ((T*)this)->test_defaults("double_3_14", dummyMap);}
    {TRACE ((T*)this)->test_defaults("double_minus_1", dummyMap);}
    {TRACE ((T*)this)->test_defaults("double_minus_3_14", dummyMap);}
    {TRACE ((T*)this)->test_defaults("str_empty", dummyMap);}
    {TRACE ((T*)this)->test_defaults("str_a", dummyMap);}
    {TRACE ((T*)this)->test_defaults("str_asd", dummyMap);}
    {TRACE ((T*)this)->test_defaults("list_empty", dummyMap);}
    {TRACE ((T*)this)->test_defaults("list_bool", dummyMap);}
    {TRACE ((T*)this)->test_defaults("list_int", dummyMap);}
    {TRACE ((T*)this)->test_defaults("list_uint", dummyMap);}
    {TRACE ((T*)this)->test_defaults("list_double", dummyMap);}
    {TRACE ((T*)this)->test_defaults("list_str", dummyMap);}
    {TRACE ((T*)this)->test_s("dict_empty", dummyMap,
      (std::map<std::string, std::string>()), false);}
    {TRACE ((T*)this)->test_s("dict_bool", (std::map<std::string, bool>()),
      (std::map<std::string, bool>({{"a", true}, {"b", false}, {"c", true}})), false);}
    {TRACE ((T*)this)->test_s("dict_int", (std::map<std::string, int>()),
      (std::map<std::string, int>({{"a", 0}, {"b", 1}, {"c", -1}})), false);}
    {TRACE ((T*)this)->test_s("dict_uint", (std::map<std::string, unsigned int>()),
      (std::map<std::string, unsigned int>({{"a", 0}, {"b", 1}, {"c", 2}})), false);}
    {TRACE ((T*)this)->test_s("dict_double", (std::map<std::string, double>()),
      (std::map<std::string, double>({{"aaa", 0.0}, {"bbb", 1.0}, {"ccc", -1.0}})), false);}
    {TRACE ((T*)this)->test_s("dict_str", (std::map<std::string, std::string>()),
      (std::map<std::string, std::string>({{"aaa", "a"}, {"bbb", "b"}, {"ccc", "c"}})), false);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", dummyMap);}
    {TRACE ((T*)this)->test_s("dict_str_recursive", (std::map<std::string, std::map<std::string, int>>()),
       (std::map<std::string, std::map<std::string, int>>(
         {{"a", {{"b", 1}, {"c", 2}}}, {"d", {{"e", 3}, {"f", 4}, {"g", 5}}}}
       )), false);}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, std::string>()));}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, bool>({{"a", true}, {"b", false}, {"c", true}})));}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, int>({{"a", 0}, {"b", 1}, {"c", -1}})));}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, unsigned int>({{"a", 0u}, {"b", 1u}, {"c", 2u}})));}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, double>({{"a", 0.0}, {"b", 1.1}, {"c", -1.1}})));}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, std::string>({{"a", "a"}, {"b", "b"}, {"cde", "cde"}})));}
    {TRACE ((T*)this)->test_defaults(
      "dict_bool", (std::map<std::string, int>({{"a", 1}})));}
    {TRACE ((T*)this)->test_defaults(
      "dict_bool", (std::map<std::string, double>({{"a", 1.0}})));}
    {TRACE ((T*)this)->test_defaults(
      "dict_bool", (std::map<std::string, std::string>({{"a", "a"}})));}

    // More crazy recursive stuff
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, std::vector<int>>({{"a", {0, 1}}, {"b", {2, 3}}})));}
    {TRACE ((T*)this)->test_defaults(
      "nonexistent", (std::map<std::string, std::map<std::string, std::vector<size_t>>>({
        {"a", {{"aa", {0_sz, 1_sz}}}},
        {"b", {{"bb", {2_sz, 3_sz}}}}
      })));}
    // the dict contains -2 and -3, so reading that member as size_t shouldn't work; but 0 and 1 member should load
    {TRACE ((T*)this)->test_s(
      "dict_crazy", (std::map<std::string, std::map<std::string, std::vector<size_t>>>()),
        (std::map<std::string, std::map<std::string, std::vector<size_t>>>({
          {"a", {{"aa", {0_sz, 1_sz}}}}
        })), false);}
    // reading as int should work
    {TRACE ((T*)this)->test_s(
      "dict_crazy", (std::map<std::string, std::map<std::string, std::vector<int>>>()),
        (std::map<std::string, std::map<std::string, std::vector<int>>>({
          {"a", {{"aa", {0, 1}}}},
          {"b", {{"bb", {-2, -3}}}}
        })), false);}

    {TRACE ((T*)this)->test_s("dict_empty", (std::unordered_map<std::string, std::string>({{"a", "a"}})),
      (std::unordered_map<std::string, std::string>()), false);}
    {TRACE ((T*)this)->test_s("dict_bool", (std::unordered_map<std::string, bool>()),
      (std::unordered_map<std::string, bool>({{"a", true}, {"b", false}, {"c", true}})), false);}
    {TRACE ((T*)this)->test_s("dict_int", (std::unordered_map<std::string, int>()),
      (std::unordered_map<std::string, int>({{"a", 0}, {"b", 1}, {"c", -1}})), false);}
    {TRACE ((T*)this)->test_s("dict_uint", (std::unordered_map<std::string, unsigned int>()),
      (std::unordered_map<std::string, unsigned int>({{"a", 0}, {"b", 1}, {"c", 2}})), false);}
    {TRACE ((T*)this)->test_s("dict_double", (std::unordered_map<std::string, double>()),
      (std::unordered_map<std::string, double>({{"aaa", 0.0}, {"bbb", 1.0}, {"ccc", -1.0}})), false);}
    {TRACE ((T*)this)->test_s("dict_str", (std::unordered_map<std::string, std::string>()),
      (std::unordered_map<std::string, std::string>({{"aaa", "a"}, {"bbb", "b"}, {"ccc", "c"}})), false);}
    {TRACE ((T*)this)->test_defaults("dict_mixed", (std::unordered_map<std::string, std::string>({{"a", "a"}})));}
  }

  inline void testRos()
  {
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::Time(30));}
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::Duration(30));}
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::Rate(30));}
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::WallTime(30));}
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::WallDuration(30));}
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::WallRate(30));}
   {TRACE ((T*)this)->test_defaults("nonexistent", ros::SteadyTime(30));}

   {TRACE ((T*)this)->test("double_3_14", ros::Duration(30), ros::Duration(3.14), false);}
  }

  inline void testGeometryMsgs()
  {
    geometry_msgs::Vector3 v; v.x = 0; v.y = 1; v.z = 2;
    {TRACE ((T*)this)->test_defaults("nonexistent", v);}

    geometry_msgs::Vector3 v2; v2.x = M_PI;
    {TRACE ((T*)this)->test_near("quat3", v, v2, false);}

    geometry_msgs::Point p; p.x = 0; p.y = 1; p.z = 2;
    geometry_msgs::Point p2; p2.x = M_PI;
    {TRACE ((T*)this)->test_near("quat3", p, p2, false);}

    geometry_msgs::Point32 pp; pp.x = 0; pp.y = 1; pp.z = 2;
    geometry_msgs::Point32 pp2; pp2.x = M_PI;
    {TRACE ((T*)this)->test_near("quat3", pp, pp2, false);}

    geometry_msgs::Quaternion q; q.x = 0; q.y = 1; q.z = 2; q.w = 3; // it doesn't need to be a valid quaternion
    {TRACE ((T*)this)->test_defaults("nonexistent", q);}

    geometry_msgs::Quaternion q2; q2.x = 1;
    {TRACE ((T*)this)->test_near("quat3", q, q2, false);}
    {TRACE ((T*)this)->test_near("quat4", q, q2, false);}

    geometry_msgs::Transform t; t.translation = v; t.rotation = q;
    {TRACE ((T*)this)->test_defaults("nonexistent", t);}

    geometry_msgs::Vector3 v3; v3.x = 1; v3.y = 2; v3.z = 3;
    geometry_msgs::Transform t2; t2.translation = v3; t2.rotation = q2;
    {TRACE ((T*)this)->test_near("tf6", t, t2, false);}
    {TRACE ((T*)this)->test_near("tf7", t, t2, false);}
    {TRACE ((T*)this)->test_near("tf16", t, t2, false);}
  }

  inline void testTF2()
  {
    tf2::Vector3 v(0, 1, 2);
    {TRACE ((T*)this)->test_defaults("nonexistent", v);}

    tf2::Vector3 v2(M_PI, 0, 0);
    {TRACE ((T*)this)->test_near("quat3", v, v2, false);}

    tf2::Quaternion q(0, 1, 2, 3); // it doesn't need to be a valid quaternion
    {TRACE ((T*)this)->test_defaults("nonexistent", q);}

    tf2::Quaternion q2(1, 0, 0, 0);
    {TRACE ((T*)this)->test_near("quat3", q, q2, false);}
    {TRACE ((T*)this)->test_near("quat4", q, q2, false);}

    tf2::Transform t; t.setOrigin(v); t.setRotation(q);
    {TRACE ((T*)this)->test_defaults("nonexistent", t);}

    tf2::Vector3 v3(1, 2, 3);
    tf2::Transform t2; t2.setOrigin(v3); t2.setRotation(q2);
    {TRACE ((T*)this)->test_near("tf6", t, t2, false);}
    {TRACE ((T*)this)->test_near("tf7", t, t2, false);}
    {TRACE ((T*)this)->test_near("tf16", t, t2, false);}
  }

  template <typename Scalar>
  inline void testEigenTemplate()
  {
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
    typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
    typedef Eigen::Quaternion<Scalar> Quaternion;
    typedef Eigen::Transform<Scalar, 3, Eigen::Isometry> Isometry3;
    
    Vector3 v(0, 1, 2);
    {TRACE ((T*)this)->test_defaults("nonexistent", v);}
    {TRACE ((T*)this)->test_defaults("quat4", v);}

    Vector3 v2(M_PI, 0, 0);
    {TRACE ((T*)this)->test_near("quat3", v, v2, false);}

    Quaternion q(3, 0, 1, 2); // it doesn't need to be a valid quaternion; w first!
    {TRACE ((T*)this)->test_defaults("nonexistent", q);}
    {TRACE ((T*)this)->test_defaults("tf7", q);}

    Quaternion q2(0, 1, 0, 0);  // w first!
    {TRACE ((T*)this)->test_near("quat3", q, q2, false);}
    {TRACE ((T*)this)->test_near("quat4", q, q2, false);}

    Vector4 v4(1, 0, 0, 0);
    {TRACE ((T*)this)->test_near("quat4", v4, Vector4(1, 0, 0, 0), false);}
    {TRACE ((T*)this)->test_defaults("tf7", v4);}

    Isometry3 t = Isometry3::Identity();
    t.translationExt() = v; t.linearExt() = q.toRotationMatrix();
    {TRACE ((T*)this)->test_defaults("nonexistent", t);}

    Vector3 v3(1, 2, 3);
    Isometry3 t2 = Isometry3::Identity();
    t2.translationExt() = v3; t2.linearExt() = q2.toRotationMatrix();
    {TRACE ((T*)this)->test_near("tf6", t, t2, false);}
    {TRACE ((T*)this)->test_near("tf7", t, t2, false);}
    {TRACE ((T*)this)->test_near("tf16", t, t2, false);}
    {TRACE ((T*)this)->test_defaults("quat3", t);}

    Matrix4 m = Matrix4::Identity();
    Matrix4 m2; m2 <<
      1, 0, 0, 1,
      0, -1, 0, 2,
      0, 0, -1, 3,
      0, 0, 0, 1;
    {TRACE ((T*)this)->test_near("tf16", m, m2, false);}
    {TRACE ((T*)this)->test_defaults("tf7", m);}
    {TRACE ((T*)this)->test_defaults("bool_True", m);}
  }

  inline void testEigen()
  {
    this->template testEigenTemplate<double>();
    this->template testEigenTemplate<float>();
  }

  inline void testNested()
  {
    {TRACE ((T*)this)->test("debug/pcl/inside", true, false, false);}
    {TRACE ((T*)this)->test_defaults("debug/pcl/nonexistent", true);}
    {TRACE ((T*)this)->test("test/negative", 1, -1, false);}
    {TRACE ((T*)this)->test("sensor/min_distance", 0.01, 0.1, false);}
    {TRACE ((T*)this)->test_s("frames/fixed", std::string("fixed"), std::string("odom"), false);}
    {TRACE ((T*)this)->test_s("frames/fixed", "fixed", "odom", false);}
    {TRACE ((T*)this)->test_defaults("frames/fixed", 1);} // wrong value type
    {TRACE ((T*)this)->test_defaults("test/negative", static_cast<uint64_t>(1));}
    {TRACE ((T*)this)->test_defaults("test/negative", static_cast<unsigned int>(1));}
    {TRACE ((T*)this)->test_defaults("non/existent", static_cast<unsigned int>(1));}
    {TRACE ((T*)this)->test("transforms/buffer_length", ros::Duration(30), ros::Duration(60), false);}
    {TRACE ((T*)this)->test_s("ignored_links/bounding_sphere", std::vector<std::string>(),
      std::vector<std::string>({"antenna", "base_link::big_collision_box"}), false);}
    {TRACE ((T*)this)->test_s("ignored_links/bounding_sphere", std::set<std::string>(),
      std::set<std::string>({"antenna", "base_link::big_collision_box"}), false);}
    {TRACE ((T*)this)->test_s("body_model/per_link/scale", (std::map<std::string, double>()), 
      (std::map<std::string, double>({
        {"antenna::contains", 1.2},
        {"antenna::bounding_sphere", 1.2},
        {"antenna::bounding_box", 1.2},
        {"*::big_collision_box::contains", 2.0},
        {"*::big_collision_box::bounding_sphere", 2.0},
        {"*::big_collision_box::bounding_box", 2.0},
        {"*::big_collision_box::shadow", 3.0}
      })), false);}
    {TRACE ((T*)this)->test_s("body_model/per_link/padding", (std::map<std::string, double>()),
      (std::map<std::string, double>({{"laser::shadow", 0.015}, {"base_link", 0.05}})), false);}
    {TRACE ((T*)this)->test_defaults(
      "body_model/per_link/all_wrong", (std::map<std::string, double>({{"a", 1}, {"b", 2}})));}
  }
};