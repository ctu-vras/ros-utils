#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Utilities for working with XmlRpcValues.
 * \author Martin Pecka
 */

#include <array>
#include <cfloat>
#include <limits>
#include <list>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <cras_cpp_common/param_utils/param_traits.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <rclcpp/parameter_value.hpp>

namespace cras {

/**
 * \brief Convert XmlRpcValue `x` to value `v`.
 * \param[in] x The XmlRpcValue to convert.
 * \param[out] v The value to convert to.
 * \return True if the conversion succeeded. If skip_non_convertible is true, conversion will succeed if at least one
 *         contained value succeeded converting (if converting to a container type).
 */
inline bool convert(
  const ::rclcpp::ParameterValue& x, ::rclcpp::ParameterValue& v, bool /*skip_non_convertible*/ = false,
  ::std::list<::std::string>* /*errors*/ = nullptr) {
  v = x;
  return true;
}

inline bool convert(
  const ::rclcpp::ParameterValue& x, bool& v, bool /*skip_non_convertible*/ = false,
  ::std::list<::std::string>* errors = nullptr) {
  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_BOOL) {
    v = x.get<bool>();
    return true;
  }

  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_INTEGER) {
    const auto i = x.get<int64_t>();
    if (i == 0 || i == 1) {
      v = static_cast<bool>(i);
      return true;
    } else if (errors != nullptr) {
      errors->push_back(::cras::format("Cannot convert int value {} to boolean.", i));
    }
  }

  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_STRING) {
    const auto i = ::cras::toLower(x.get<::std::string>());
    if (i == "true" || i == "1") {
      v = true;
      return true;
    } else if (i == "false" || i == "0") {
      v = false;
      return true;
    } else if (errors != nullptr) {
      errors->push_back(::cras::format("Cannot convert string value {} to boolean.", i));
    }
  }

  if (errors != nullptr) {
    errors->push_back(::cras::format("Cannot convert type {} to boolean.", ::rclcpp::to_string(x.get_type())));
  }
  return false;
}

inline bool convert(
  const ::rclcpp::ParameterValue& x, int64_t& v, bool /*skip_non_convertible*/ = false,
  ::std::list<::std::string>* errors = nullptr) {
  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_INTEGER) {
    v = x.get<int64_t>();
    return true;
  }

  if (errors != nullptr) {
    errors->push_back(::cras::format("Cannot convert type {} to int.", ::rclcpp::to_string(x.get_type())));
  }

  return false;
}

template<typename T>
bool convert(
  const ::std::string& x, ::std::enable_if_t<std::is_convertible_v<::std::string, T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* /*errors*/ = nullptr) {
  v = static_cast<T>(x);
  return true;
}

template<typename T>
bool convert(
  const bool& x, ::std::enable_if_t<std::is_same_v<bool, T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* /*errors*/ = nullptr) {
  v = x;
  return true;
}

template<typename T>
bool convert(
  const int64_t& x, ::std::enable_if_t<std::is_same_v<bool, T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* errors = nullptr) {
  if (x == 0 || x == 1) {
    v = static_cast<T>(x);
    return true;
  } else if (errors != nullptr) {
    errors->push_back(::cras::format("Cannot convert int value {} to boolean.", x));
  }
  return false;
}

template<typename T>
bool convert(
  const ::std::string& x, ::std::enable_if_t<std::is_same_v<bool, T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* errors = nullptr) {
  if (x == "true" || x == "1") {
    v = true;
    return true;
  } else if (x == "false" || x == "0") {
    v = false;
    return true;
  } else if (errors != nullptr) {
    errors->push_back(::cras::format("Cannot convert string value {} to boolean.", x));
  }
  return false;
}

template<typename T>
bool convert(
  const int64_t& x, ::std::enable_if_t<std::is_integral_v<T> && !std::is_same_v<T, bool>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* errors = nullptr) {
  if constexpr (sizeof(T) > sizeof(int64_t)) {
    v = static_cast<T>(x);
  } else if constexpr (std::is_same_v<int64_t, T>) {
    v = x;
  } else if constexpr (std::is_same_v<uint64_t, T>) {
    if (x < 0) {
      if (errors != nullptr) {
        errors->push_back(::cras::format("Value {} is out of bounds <0, {}>.",
          ::cras::to_string(x), ::cras::to_string(std::numeric_limits<T>::max())));
      }
      return false;
    }
    v = static_cast<T>(x);
  } else {
    // we have at most 32-bit types
    constexpr auto min_bound = static_cast<int64_t>(::std::numeric_limits<T>::lowest());
    constexpr auto max_bound = static_cast<int64_t>(::std::numeric_limits<T>::max());
    if (x < min_bound || x > max_bound) {
      if (errors != nullptr) {
        errors->push_back(::cras::format("Value {} is out of bounds <{}, {}>.",
          ::cras::to_string(x), ::cras::to_string(min_bound), ::cras::to_string(max_bound)));
      }
      return false;
    }
    v = static_cast<int64_t>(x);
  }
  return true;
}

template<typename T>
bool convert(
  const double& x, ::std::enable_if_t<std::is_floating_point_v<T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* errors = nullptr) {
  if constexpr (sizeof(T) > sizeof(double)) {
    v = static_cast<T>(x);
  } else if constexpr (std::is_same_v<double, T>) {
    v = x;
  } else {
    // we have at most 32-bit types
    if (::std::isnan(x)) { v = ::std::numeric_limits<T>::quiet_NaN(); return true; }
    if (::std::isinf(x) && x > 0.0) { v = ::std::numeric_limits<T>::infinity(); return true; }
    if (::std::isinf(x) && x < 0.0) { v = -::std::numeric_limits<T>::infinity(); return true; }

    constexpr auto min_bound = static_cast<double>(::std::numeric_limits<T>::lowest());
    constexpr auto max_bound = static_cast<double>(::std::numeric_limits<T>::max());
    if (x < min_bound || x > max_bound) {
      if (errors != nullptr) {
        errors->push_back(::cras::format("Value {} is out of bounds <{}, {}>.",
          ::cras::to_string(x), ::cras::to_string(min_bound), ::cras::to_string(max_bound)));
      }
      return false;
    }
    v = static_cast<double>(x);
  }
  return true;
}

template<typename T>
bool convert(
  const int64_t& x, ::std::enable_if_t<std::is_floating_point_v<T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* errors = nullptr) {
  return convert(static_cast<double>(x), v, false, errors);
}

template<typename T>
bool convert(
  const uint8_t& x, ::std::enable_if_t<std::is_same_v<uint8_t, T>, T>& v,
  bool /*skip_non_convertible*/ = false, ::std::list<::std::string>* /*errors*/ = nullptr) {
  v = x;
  return true;
}

#define DEFINE_INTEGRAL_CONVERT(resultType, paramType, minBound, maxBound) \
  inline bool convert( \
    const ::rclcpp::ParameterValue& x, resultType& v, bool skip_non_convertible = false, \
    ::std::list<::std::string>* errors = nullptr) { \
    paramType i; \
    if (!convert(x, i, skip_non_convertible, errors)) { \
      return false; \
    } \
    if (i < (minBound) || i > (maxBound)) { \
      if (errors != nullptr) { \
        errors->push_back(::cras::format("Value {} is out of bounds <{}, {}>.", \
          ::cras::to_string(i), ::cras::to_string(minBound), ::cras::to_string(maxBound))); \
      } \
      return false; \
    } \
    v = static_cast<resultType>(i); \
    return true; \
  }

DEFINE_INTEGRAL_CONVERT(char, int64_t, std::numeric_limits<char>::min(), std::numeric_limits<char>::max())

DEFINE_INTEGRAL_CONVERT(signed char, int64_t, SCHAR_MIN, SCHAR_MAX)

DEFINE_INTEGRAL_CONVERT(short, int64_t, SHRT_MIN, SHRT_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(int, int64_t, INT_MIN, INT_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(long long, int64_t, LONG_LONG_MIN, LONG_LONG_MAX)  // NOLINT

DEFINE_INTEGRAL_CONVERT(unsigned char, int64_t, 0, UCHAR_MAX)

DEFINE_INTEGRAL_CONVERT(unsigned short, int64_t, 0, USHRT_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(unsigned int, int64_t, 0, UINT_MAX)

DEFINE_INTEGRAL_CONVERT(unsigned long, int64_t, 0, ULONG_MAX)  // NOLINT
DEFINE_INTEGRAL_CONVERT(unsigned long long, int64_t, 0, ULONG_LONG_MAX)  // NOLINT

inline bool convert(
  const ::rclcpp::ParameterValue& x, double& v, bool /*skip_non_convertible*/ = false,
  ::std::list<::std::string>* errors = nullptr) {
  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_DOUBLE) {
    v = x.get<double>();
    return true;
  }

  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_INTEGER) {
    v = static_cast<double>(x.get<int64_t>());
    return true;
  }

  if (errors != nullptr) {
    errors->push_back(::cras::format("Cannot convert type {} to double.", ::rclcpp::to_string(x.get_type())));
  }

  return false;
}

#define DEFINE_DOUBLE_CONVERT(resultType, paramType, minBound, maxBound) \
  inline bool convert( \
    const ::rclcpp::ParameterValue& x, resultType& v, bool skip_non_convertible = false, \
    ::std::list<::std::string>* errors = nullptr) { \
    paramType i; \
    if (!convert(x, i, skip_non_convertible, errors)) { \
      return false; \
    } \
    if (::std::isnan(i)) { v = ::std::numeric_limits<resultType>::quiet_NaN(); return true; } \
    if (::std::isinf(i) && i > 0) { v = ::std::numeric_limits<resultType>::infinity(); return true; } \
    if (::std::isinf(i) && i < 0) { v = -::std::numeric_limits<resultType>::infinity(); return true; } \
    if (i < (minBound) || i > (maxBound)) { \
      if (errors != nullptr) { \
        errors->push_back(::cras::format("Value {} is out of bounds <{}, {}>.", \
          ::cras::to_string(i), ::cras::to_string(minBound), ::cras::to_string(maxBound))); \
      } \
      return false; \
    } \
    v = static_cast<resultType>(i); \
    return true; \
  }

DEFINE_DOUBLE_CONVERT(float, double, -FLT_MAX, FLT_MAX)

DEFINE_DOUBLE_CONVERT(long double, double, -LDBL_MAX, LDBL_MAX)

inline bool convert(
  const ::rclcpp::ParameterValue& x, std::string& v, bool /*skip_non_convertible*/ = false,
  ::std::list<::std::string>* errors = nullptr) {
  if (x.get_type() == ::rclcpp::ParameterType::PARAMETER_STRING) {
    v = x.get<std::string>();
    return true;
  }

  if (errors != nullptr) {
    errors->push_back(::cras::format("Cannot convert type {} to string.", ::rclcpp::to_string(x.get_type())));
  }

  return false;
}

// forward-declare container types so that they can be used by the other container converters (set inside vector etc.)
template<typename T>
bool convert(
  const ::rclcpp::ParameterValue& x, ::std::vector<T>& v,
  bool skip_non_convertible = false, ::std::list<::std::string>* errors = nullptr);

template<typename T>
bool convert(
  const ::rclcpp::ParameterValue& x, ::std::list<T>& v,
  bool skip_non_convertible = false, ::std::list<::std::string>* errors = nullptr);

template<typename T>
bool convert(
  const ::rclcpp::ParameterValue& x, ::std::set<T>& v,
  bool skip_non_convertible = false, ::std::list<::std::string>* errors = nullptr);

template<typename T>
bool convert(
  const ::rclcpp::ParameterValue& x, ::std::unordered_set<T>& v,
  bool skip_non_convertible = false, ::std::list<::std::string>* errors = nullptr);

template<typename T, size_t N>
bool convert(
  const ::rclcpp::ParameterValue& x, ::std::array<T, N>& v,
  bool skip_non_convertible = false, ::std::list<::std::string>* errors = nullptr);

#define DEFINE_ARRAY_CONVERT(arrayType, insertFn) \
  template<typename T> \
  inline bool convert(const ::rclcpp::ParameterValue& x, arrayType<T>& v, bool skip_non_convertible, \
    ::std::list<::std::string>* errors) \
  { \
    const auto array_param_type = ::cras::ParameterValueTraits<arrayType<T>>::param_type; \
    if (x.get_type() != array_param_type) { \
      if (errors != nullptr) { \
        errors->push_back(::cras::format( \
          "Cannot convert type {} to {}.", ::rclcpp::to_string(x.get_type()), ::rclcpp::to_string(array_param_type))); \
      } \
      return false; \
    } \
    v.clear(); \
    const auto& array = x.get<::cras::ParameterValueTraits<arrayType<T>>::param_type>(); \
    for (size_t i = 0; i < array.size(); ++i) { \
      T t; \
      if (convert(array[i], t, skip_non_convertible, errors)) \
      { \
        v.insertFn(t); \
      } else if (!skip_non_convertible) { \
        return false; \
      } \
    } \
    return v.size() > 0 || array.size() == 0; \
  }

DEFINE_ARRAY_CONVERT(::std::vector, push_back)

DEFINE_ARRAY_CONVERT(::std::list, push_back)

DEFINE_ARRAY_CONVERT(::std::set, insert)

DEFINE_ARRAY_CONVERT(::std::unordered_set, insert)

template<typename T, size_t N>
bool convert(
  const ::rclcpp::ParameterValue& x, ::std::array<T, N>& v, bool skip_non_convertible,
  ::std::list<::std::string>* errors) {
  const auto array_param_type = ::cras::ParameterValueTraits<::std::array<T, N>>::param_type;
  if (x.get_type() != array_param_type) {
    if (errors != nullptr) {
      errors->push_back(::cras::format(
        "Cannot convert type {} to {}.", ::cras::to_string(x.get_type()), ::cras::to_string(array_param_type)));
    }
    return false;
  }
  const auto& array = x.get<::cras::ParameterValueTraits<::std::array<T, N>>::param_type>();
  if (array.size() != N) {
    if (errors != nullptr) {
      errors->push_back(::cras::format(
        "The array is expected to have {} items, but {} was given.", N, array.size()));
    }
    return false;
  }
  for (size_t i = 0; i < array.size(); ++i) {
    T t;
    if (convert(array[i], t, skip_non_convertible, errors)) {
      v[i] = t;
    } else {  // we cannot skip non-convertible values because we do not know what to use instead of them
      return false;
    }
  }
  return true;
}

}  // namespace cras
