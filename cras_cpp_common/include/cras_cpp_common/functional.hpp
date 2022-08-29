/**
 * \file
 * \brief C++ utilities for working with functions.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <functional>
#include <type_traits>

namespace cras
{

// Shims for C++17-only std::invoke and std::apply.

#if defined(__cpp_lib_invoke) && __cpp_lib_invoke >= 201411L && defined(__cpp_lib_apply) && __cpp_lib_apply >= 201603L
using ::std::invoke;
using ::std::invoke_result;
using ::std::invoke_result_t;
using ::std::is_invocable;
using ::std::is_invocable_r;
using ::std::apply;
#else
namespace impl
{
#include <cras_cpp_common/external/invoke.hpp/invoke.hpp>
}
using ::cras::impl::invoke_hpp::invoke;
using ::cras::impl::invoke_hpp::invoke_result;
using ::cras::impl::invoke_hpp::invoke_result_t;
using ::cras::impl::invoke_hpp::is_invocable;
using ::cras::impl::invoke_hpp::is_invocable_r;
using ::cras::impl::invoke_hpp::apply;
#endif

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
