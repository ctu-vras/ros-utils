#pragma once

/**
 * \file
 * \brief Utilities for working with XmlRpcValues.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <limits>
#include <list>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <cras_cpp_common/xmlrpc_value_traits.hpp>

namespace XmlRpc
{
// This prevents GTest to understand all XmlRpcValues as structs and trying to print them as such.
// That results in memory corruption and an exception thrown in GTest EXPECT* methods.
inline void PrintTo(const XmlRpcValue& value, ::std::ostream* os)
{
  if (os)
    *os << value.toXml();
}
}

namespace cras
{

/**
 * \brief Convert XmlRpcValue `x` to value `v`.
 * \param[in] x The XmlRpcValue to convert.
 * \param[out] v The value to convert to.
 * \param[in] skipNonConvertible If true and converting to a container type, skip those values that cannot be held by
 *                               the target container type.
 * \param[in,out] errors If non-null, any conversion error messages will be stored here.
 * \return True if the conversion succeeded. If skipNonConvertible is true, conversion will succeed if at least one
 *         contained value succeeded converting (if converting to a container type). 
 */
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::XmlRpc::XmlRpcValue& v, bool /*skipNonConvertible*/ = false,
  ::std::list<::std::string>* errors = nullptr)
{
  v = x;
  return true;
}

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
inline bool convert(const ::XmlRpc::XmlRpcValue& x, bool& v, bool /*skipNonConvertible*/ = false,
  ::std::list<::std::string>* errors = nullptr)
{
  if (x.getType() == ::XmlRpc::XmlRpcValue::TypeBoolean)
  {
    v = x;
    return true;
  }
  
  if (x.getType() == ::XmlRpc::XmlRpcValue::TypeInt)
  {
    const auto i = static_cast<int>(x);
    if (i == 0 || i == 1)
    {
      v = static_cast<bool>(i);
      return true;
    }
    else if (errors != nullptr)
      errors->push_back(::cras::format("Cannot convert int value %i to boolean.", i));
  }

  if (errors != nullptr)
    errors->push_back(::cras::format("Cannot convert type %s to boolean.", ::cras::to_cstring(x.getType())));
  return false;
}

inline bool convert(const ::XmlRpc::XmlRpcValue& x, int& v, bool /*skipNonConvertible*/ = false,
  ::std::list<::std::string>* errors = nullptr)
{
  if (x.getType() == ::XmlRpc::XmlRpcValue::TypeInt)
  {
    v = x;
    return true;
  }

  if (errors != nullptr)
    errors->push_back(::cras::format("Cannot convert type %s to int.", ::cras::to_cstring(x.getType())));

  return false;
}

#define DEFINE_INTEGRAL_CONVERT(resultType, xmlType, minBound, maxBound) \
  /** \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*) */\
  inline bool convert(const ::XmlRpc::XmlRpcValue& x, resultType& v, bool skipNonConvertible = false, \
    ::std::list<::std::string>* errors = nullptr) \
  { \
    xmlType i; \
    if (!convert(x, i, skipNonConvertible, errors)) \
      return false; \
    if (i < (minBound) || i > (maxBound)) { \
      if (errors != nullptr) \
        errors->push_back(::cras::format("Value %s is out of bounds <%s, %s>.", \
          ::cras::to_string(i).c_str(), ::cras::to_string(minBound).c_str(), ::cras::to_string(maxBound).c_str())); \
      return false; \
    } \
    v = static_cast<resultType>(i); \
    return true; \
  }

DEFINE_INTEGRAL_CONVERT(char, int, CHAR_MIN, CHAR_MAX)
DEFINE_INTEGRAL_CONVERT(signed char, int, CHAR_MIN, CHAR_MAX)
DEFINE_INTEGRAL_CONVERT(short, int, SHRT_MIN, SHRT_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(long, int, LONG_MIN, LONG_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(long long, int, LONG_LONG_MIN, LONG_LONG_MAX)  // NOLINT

DEFINE_INTEGRAL_CONVERT(unsigned char, int, 0, UCHAR_MAX)
DEFINE_INTEGRAL_CONVERT(unsigned short, int, 0, USHRT_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(unsigned int, int, 0, UINT_MAX)
DEFINE_INTEGRAL_CONVERT(unsigned long, int, 0, ULONG_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(unsigned long long, int, 0, ULONG_LONG_MAX)  // NOLINT

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
inline bool convert(const ::XmlRpc::XmlRpcValue& x, double& v, bool /*skipNonConvertible*/ = false,
  ::std::list<::std::string>* errors = nullptr)
{
  if (x.getType() == ::XmlRpc::XmlRpcValue::TypeDouble)
  {
    v = x;
    return true;
  }

  if (x.getType() == ::XmlRpc::XmlRpcValue::TypeInt)
  {
    v = static_cast<double>(static_cast<int>(x));
    return true;
  }

  if (errors != nullptr)
    errors->push_back(::cras::format("Cannot convert type %s to double.", ::cras::to_cstring(x.getType())));

  return false;
}

#define DEFINE_DOUBLE_CONVERT(resultType, xmlType, minBound, maxBound) \
  /** \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*) */\
  inline bool convert(const ::XmlRpc::XmlRpcValue& x, resultType& v, bool skipNonConvertible = false, \
    ::std::list<::std::string>* errors = nullptr) \
  { \
    xmlType i; \
    if (!convert(x, i, skipNonConvertible, errors)) \
      return false; \
    if (::std::isnan(i)) { v = ::std::numeric_limits<resultType>::quiet_NaN(); return true; } \
    if (::std::isinf(i) && i > 0) { v = ::std::numeric_limits<resultType>::infinity(); return true; } \
    if (::std::isinf(i) && i < 0) { v = -::std::numeric_limits<resultType>::infinity(); return true; } \
    if (i < (minBound) || i > (maxBound)) { \
      if (errors != nullptr) \
        errors->push_back(::cras::format("Value %s is out of bounds <%s, %s>.", \
          ::cras::to_string(i).c_str(), ::cras::to_string(minBound).c_str(), ::cras::to_string(maxBound).c_str())); \
      return false; \
    } \
    v = static_cast<resultType>(i); \
    return true; \
  }

DEFINE_DOUBLE_CONVERT(float, double, -FLT_MAX, FLT_MAX)
DEFINE_DOUBLE_CONVERT(long double, double, -LDBL_MAX, LDBL_MAX)

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
inline bool convert(const ::XmlRpc::XmlRpcValue& x, std::string& v, bool /*skipNonConvertible*/ = false,
  ::std::list<::std::string>* errors = nullptr)
{
  if (x.getType() == ::XmlRpc::XmlRpcValue::TypeString)
  {
    v = static_cast<::std::string>(x);
    return true;
  }

  if (errors != nullptr)
    errors->push_back(::cras::format("Cannot convert type %s to string.", ::cras::to_cstring(x.getType())));

  return false;
}

// forward-declare container types so that they can be used by the other container converters (map inside vector etc.)
//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
template<typename T>
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::std::map<::std::string, T>& v,
  bool skipNonConvertible = false, ::std::list<::std::string>* errors = nullptr);

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
template<typename T>
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::std::unordered_map<::std::string, T>& v,
  bool skipNonConvertible = false, ::std::list<::std::string>* errors = nullptr);

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
template<typename T>
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::std::vector<T>& v,
  bool skipNonConvertible = false, ::std::list<::std::string>* errors = nullptr);

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
template<typename T>
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::std::list<T>& v,
  bool skipNonConvertible = false, ::std::list<::std::string>* errors = nullptr);

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
template<typename T>
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::std::set<T>& v,
  bool skipNonConvertible = false, ::std::list<::std::string>* errors = nullptr);

//! \overload cras::convert(const XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&, bool, ::std::list<::std::string>*)
template<typename T>
inline bool convert(const ::XmlRpc::XmlRpcValue& x, ::std::unordered_set<T>& v,
  bool skipNonConvertible = false, ::std::list<::std::string>* errors = nullptr);

#define DEFINE_ARRAY_CONVERT(arrayType, insertFn) \
  template<typename T> \
  inline bool convert(const ::XmlRpc::XmlRpcValue& x, arrayType<T>& v, bool skipNonConvertible, \
    ::std::list<::std::string>* errors) \
  { \
    if (x.getType() != ::XmlRpc::XmlRpcValue::TypeArray) { \
      if (errors != nullptr) \
        errors->push_back(::cras::format("Cannot convert type %s to array.", ::cras::to_cstring(x.getType())));\
      return false; \
    } \
    v.clear(); \
    for (size_t i = 0; i < x.size(); ++i) \
    { \
      T t; \
      if (convert(x[i], t, skipNonConvertible, errors)) \
        v.insertFn(t); \
      else if (!skipNonConvertible) \
        return false; \
    } \
    return v.size() > 0 || x.size() == 0; \
  }

DEFINE_ARRAY_CONVERT(::std::vector, push_back)
DEFINE_ARRAY_CONVERT(::std::list, push_back)
DEFINE_ARRAY_CONVERT(::std::set, insert)
DEFINE_ARRAY_CONVERT(::std::unordered_set, insert)

#define DEFINE_STRUCT_CONVERT(mapType) \
  template<typename T> \
  inline bool convert(const ::XmlRpc::XmlRpcValue& x, mapType<::std::string, T>& v, bool skipNonConvertible, \
    ::std::list<::std::string>* errors) \
  { \
    if (x.getType() != ::XmlRpc::XmlRpcValue::TypeStruct) { \
      if (errors != nullptr) \
        errors->push_back(::cras::format("Cannot convert type %s to struct.", ::cras::to_cstring(x.getType())));\
      return false; \
    } \
    v.clear(); \
    for (auto it = x.begin(); it != x.end(); ++it) \
    { \
      T t; \
      if (convert(it->second, t, skipNonConvertible, errors)) \
        v[it->first] = t; \
      else if (!skipNonConvertible) \
        return false; \
    } \
    return v.size() > 0 || x.size() == 0; \
  }

DEFINE_STRUCT_CONVERT(::std::map)
DEFINE_STRUCT_CONVERT(::std::unordered_map)

}
