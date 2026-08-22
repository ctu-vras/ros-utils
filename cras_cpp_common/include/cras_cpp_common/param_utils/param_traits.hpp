#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Various type traits for ParameterValue.
 * \author Martin Pecka
 */

#include <array>
#include <list>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <cras_cpp_common/string_utils.hpp>
#include <rclcpp/parameter_value.hpp>

namespace cras {

constexpr ::rclcpp::ParameterType arrayFromBaseType(const ::rclcpp::ParameterType base_type) {
  switch (base_type) {
    case ::rclcpp::ParameterType::PARAMETER_BOOL:
      return ::rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
    case ::rclcpp::ParameterType::PARAMETER_INTEGER:
      return ::rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    case ::rclcpp::ParameterType::PARAMETER_DOUBLE:
      return ::rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
    case ::rclcpp::ParameterType::PARAMETER_STRING:
      return ::rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    default:
      return base_type;
  }
}

/**
 * \brief Type traits for ParameterValue.
 * \tparam T A datatype possibly convertible to a ParameterValue.
 */
template<typename T, class = void>
struct ParameterValueTraits {
  //! \brief Corresponding ParameterValue type that can represent values of T. NOT_SET for non-representable types.
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_NOT_SET};

  //! \brief String representation of param_type.
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};

  //! \brief Whether T is a "canonical" type for ParameterValue, i.e. whether there is a conversion function defined for
  //! T on a ParameterValue. This is examined recursively for vectors, and a vector is canonical if the innermost
  //! type is canonical or the vector itself is.
  constexpr static bool is_canonical {false};

  //! \brief Whether T is an array type.
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<bool> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_BOOL};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<char> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<signed char> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<short>  // NOLINT
{
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<int> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {std::is_same_v<int64_t, int>};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<long>  // NOLINT
{
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {std::is_same_v<int64_t, long>};  // NOLINT
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<long long>  // NOLINT
{
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {std::is_same_v<int64_t, long long>};  // NOLINT
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<unsigned char> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<unsigned short>  // NOLINT
{
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<unsigned int> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<unsigned long>  // NOLINT
{
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<unsigned long long>  // NOLINT
{
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<float> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_DOUBLE};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<double> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_DOUBLE};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<long double> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_DOUBLE};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<::std::string> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_STRING};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<char*> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_STRING};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<const char*> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_STRING};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {false};
};

template<>
struct ParameterValueTraits<::std::vector<bool>> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_BOOL_ARRAY};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {true};
};

template<>
struct ParameterValueTraits<::std::vector<int64_t>> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {true};
};

template<>
struct ParameterValueTraits<::std::vector<double>> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {true};
};

template<>
struct ParameterValueTraits<::std::vector<::std::string>> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_STRING_ARRAY};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {true};
};

template<>
struct ParameterValueTraits<::std::vector<uint8_t>> {
  constexpr static ::rclcpp::ParameterType param_type {::rclcpp::ParameterType::PARAMETER_BYTE_ARRAY};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {true};
  constexpr static bool is_array {true};
};

template<typename T>
struct ParameterValueTraits<
  ::std::vector<T>,
  ::std::enable_if_t<!::cras::ParameterValueTraits<T>::is_array && !::cras::ParameterValueTraits<T>::is_canonical>
> {
  constexpr static ::rclcpp::ParameterType param_type {
    ::cras::arrayFromBaseType(::cras::ParameterValueTraits<T>::param_type)
  };
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {true};
};

template<typename T>
struct ParameterValueTraits<
  ::std::list<T>,
  ::std::enable_if_t<!::cras::ParameterValueTraits<T>::is_array && !::cras::ParameterValueTraits<T>::is_canonical>
> {
  constexpr static ::rclcpp::ParameterType param_type {
    ::cras::arrayFromBaseType(::cras::ParameterValueTraits<T>::param_type)
  };
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {true};
};

template<typename T>
struct ParameterValueTraits<
  ::std::set<T>,
  ::std::enable_if_t<!::cras::ParameterValueTraits<T>::is_array && !::cras::ParameterValueTraits<T>::is_canonical>
> {
  constexpr static ::rclcpp::ParameterType param_type {
    ::cras::arrayFromBaseType(::cras::ParameterValueTraits<T>::param_type)
  };
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {true};
};

template<typename T>
struct ParameterValueTraits<
  ::std::unordered_set<T>,
  ::std::enable_if_t<!::cras::ParameterValueTraits<T>::is_array && !::cras::ParameterValueTraits<T>::is_canonical>
> {
  constexpr static ::rclcpp::ParameterType param_type {
    ::cras::arrayFromBaseType(::cras::ParameterValueTraits<T>::param_type)
  };
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {true};
};

template<typename T, size_t N>
struct ParameterValueTraits<
  ::std::array<T, N>,
  ::std::enable_if_t<
    ::cras::ParameterValueTraits<::std::vector<T>>::param_type != ::rclcpp::ParameterType::PARAMETER_NOT_SET>
> {
  constexpr static ::rclcpp::ParameterType param_type {::cras::ParameterValueTraits<::std::vector<T>>::param_type};
  constexpr static ::std::string_view string_type {::cras::to_string(param_type)};
  constexpr static bool is_canonical {false};
  constexpr static bool is_array {true};
};

}  // namespace cras
