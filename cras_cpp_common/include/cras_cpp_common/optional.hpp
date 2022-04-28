#pragma once

/**
 * \file
 * \brief A C++11 shim for std::optional. Uses std::optional when used in C++17 mode.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#if __has_include(<optional>) && __cplusplus >= 201703L
#include <optional>
namespace cras
{
  using ::std::optional;
  using ::std::bad_optional_access;
  using ::std::nullopt;
}
#else
#include <tl/optional.hpp>
namespace cras
{
  using ::tl::optional;
  using ::tl::bad_optional_access;
  using ::tl::nullopt;
}
#endif

#include <type_traits>

namespace cras
{
/**
 * \brief Type trait determining whether type T is cras::optional or not.
 * \tparam T The type to test.
 */
template<typename T>
struct is_optional : public ::std::false_type {};

/**
 * \brief Type trait determining whether type T is cras::optional or not.
 * \tparam T The type to test.
 */
template<typename T>
struct is_optional<::cras::optional<T>> : public ::std::true_type {};
}