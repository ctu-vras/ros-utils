#pragma once

/**
 * \file
 * \brief A C++11 shim for std::any. Uses std::any when used in C++17 mode.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <type_traits>

#include <cras_cpp_common/external/any-lite/any.hpp>

namespace cras
{

using ::nonstd::any;
using ::nonstd::any_cast;
using ::nonstd::make_any;
using ::nonstd::swap;
using ::nonstd::bad_any_cast;

/**
 * \brief Type trait determining whether type T is cras::any or not.
 * \tparam T The type to test.
 */
template<typename T>
struct is_any : public ::std::false_type {};

/**
 * \brief Type trait determining whether type T is cras::any or not.
 */
template<>
struct is_any<::cras::any> : public ::std::true_type {};

}
