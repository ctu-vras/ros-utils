#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Useful C++ string traits.
 * \author Martin Pecka
 */

#include <cstddef>
#include <string>
#include <type_traits>

namespace cras
{

/**
 * \brief Type trait for dynamic-sized and constant-sized C strings.
 * \tparam T The type to test.
 */
template<typename T>
struct is_c_string : public std::false_type {};

template<>
struct is_c_string<char*> : public std::true_type {};

template<>
struct is_c_string<char* const> : public std::true_type {};

template<>
struct is_c_string<const char*> : public std::true_type {};

template<>
struct is_c_string<const char* const> : public std::true_type {};

template<int I>
struct is_c_string<char[I]> : public std::true_type {};

template<int I>
struct is_c_string<const char[I]> : public std::true_type {};

/**
 * \brief Char trait for a C-string or std::string.
 * \tparam T The type to test.
 */
template<typename T, typename = void>
struct is_string : public std::false_type {};

template<typename T>
struct is_string<T, ::std::enable_if_t<::cras::is_c_string<typename std::decay<T>::type>::value>> :
  public std::true_type {};

template<typename T>
struct is_string<T, ::std::enable_if_t<::std::is_same<typename std::decay<T>::type, ::std::string>::value>> :
  public std::true_type {};

}
