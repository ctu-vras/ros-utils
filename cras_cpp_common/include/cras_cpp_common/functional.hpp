// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief C++ utilities for working with functions.
 * \author Martin Pecka
 */

#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace cras
{

// Shim for C++20 std::bind_front.

#if defined(__cpp_lib_bind_front) && __cpp_lib_bind_front >= 201907L
using std::bind_front;
#else
template<class F, class...Args>
auto bind_front(F&& f, Args&&... args)
{
  return [f = ::std::forward<F>(f), boundArgs = ::std::make_tuple(::std::forward<Args>(args)...)](auto&&... unboundArgs)
  {
    return ::cras::apply(
      [&](auto&&... args) -> decltype(auto) {
        return ::cras::invoke(f, decltype(args)(args)..., decltype(unboundArgs)(unboundArgs)...); },
      boundArgs);
  };
}
#endif

}
