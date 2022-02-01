/**
 * \file
 * \brief Various type traits for XmlRpcValue.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <XmlRpcValue.h>

#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

/**
 * \brief Return a string representation of the XmlRpcValue type.
 * \param[in] value The value to get type of.
 * \return The string representation of the XmlRpcValue type.
 */
constexpr const char* to_cstring(const ::XmlRpc::XmlRpcValue::Type &value)
{
  switch (value)
  {
    case ::XmlRpc::XmlRpcValue::TypeBoolean:
      return "bool";
    case ::XmlRpc::XmlRpcValue::TypeInt:
      return "int";
    case ::XmlRpc::XmlRpcValue::TypeDouble:
      return "double";
    case ::XmlRpc::XmlRpcValue::TypeString:
      return "string";
    case ::XmlRpc::XmlRpcValue::TypeDateTime:
      return "datetime";
    case ::XmlRpc::XmlRpcValue::TypeBase64:
      return "binary";
    case ::XmlRpc::XmlRpcValue::TypeArray:
      return "array";
    case ::XmlRpc::XmlRpcValue::TypeStruct:
      return "struct";
    default:
      return "invalid";
  }
}

// cras::to_string overload
template<>
inline ::std::string to_string(const ::XmlRpc::XmlRpcValue::Type &value)
{
  return ::cras::to_cstring(value);
}

/**
 * \brief Type traits for XmlRpcValue.
 * \tparam T A datatype possibly convertible to a XmlRpcValue.
 */
template<typename T, class = void>
struct XmlRpcValueTraits
{
  //! \brief Corresponding XmlRpcValue type that can represent values of T. TypeInvalid for non-representable types.
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInvalid };
  
  //! \brief String representation of xmlRpcType.
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  
  //! \brief Whether T is a "canonical" type for XmlRpcValue, i.e. whether there is a conversion function defined for T
  //! on a XmlRpcValue. This is examined recursively for vectors/maps, and a vector/map is canonical if the innermost
  //! type is canonical.
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<bool>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeBoolean };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { true };
};

template<> struct XmlRpcValueTraits<char>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<signed char>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<short>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<int>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { true };
};

template<> struct XmlRpcValueTraits<long>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<long long>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<unsigned char>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<unsigned short>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<unsigned int>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<unsigned long>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<unsigned long long>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<float>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeDouble };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<double>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeDouble };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { true };
};

template<> struct XmlRpcValueTraits<long double>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeDouble };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<::std::string>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeString };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { true };
};

template<> struct XmlRpcValueTraits<char*>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeString };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<const char*>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeString };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<> struct XmlRpcValueTraits<::tm>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeDateTime };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { true };
};

template<> struct XmlRpcValueTraits<typename ::XmlRpc::XmlRpcValue::BinaryData>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeBase64 };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { true };
};

template<typename T> struct XmlRpcValueTraits<::std::vector<T>,
  typename ::std::enable_if<::cras::XmlRpcValueTraits<T>::xmlRpcType != ::XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeArray };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { ::cras::XmlRpcValueTraits<T>::isCanonical };
};

template<typename T> struct XmlRpcValueTraits<::std::list<T>,
  typename ::std::enable_if<::cras::XmlRpcValueTraits<T>::xmlRpcType != ::XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeArray };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<typename T> struct XmlRpcValueTraits<::std::set<T>,
  typename ::std::enable_if<::cras::XmlRpcValueTraits<T>::xmlRpcType != ::XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeArray };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

template<typename T> struct XmlRpcValueTraits<::std::unordered_set<T>,
  typename ::std::enable_if<::cras::XmlRpcValueTraits<T>::xmlRpcType != ::XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeArray };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};
  
template<typename T> struct XmlRpcValueTraits<::std::map<::std::string, T>,
  typename ::std::enable_if<::cras::XmlRpcValueTraits<T>::xmlRpcType != ::XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeStruct };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { ::cras::XmlRpcValueTraits<T>::isCanonical };
};
  
template<typename T> struct XmlRpcValueTraits<::std::unordered_map<::std::string, T>,
  typename ::std::enable_if<::cras::XmlRpcValueTraits<T>::xmlRpcType != ::XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const ::XmlRpc::XmlRpcValue::Type xmlRpcType { ::XmlRpc::XmlRpcValue::TypeStruct };
  constexpr static const char* stringType { ::cras::to_cstring(xmlRpcType) };
  constexpr static const bool isCanonical { false };
};

}
