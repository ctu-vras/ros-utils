#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Type traits for std::optional.
 * \author Martin Pecka
 */

#include <optional>
#include <type_traits>

namespace cras {

template<typename T, typename Enable = void>
struct is_optional : ::std::false_type {};

template<typename T>
struct is_optional<::std::optional<T>> : ::std::true_type {};

}  // namespace cras
