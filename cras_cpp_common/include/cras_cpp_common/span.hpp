#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A C++11 shim for std::span. Uses std::span when used in C++20 mode.
 * \author Martin Pecka
 */

#if __has_include(<span>) && __cpp_lib_span >= 202002L

#include <span>

namespace cras
{
  using ::std::span;
  using ::std::make_span;
  using ::std::get;
  using ::std::as_bytes;
  using ::std::as_writable_bytes;
  using ::std::dynamic_extent;
  using ::std::byte;
}

#else

#include <cras_cpp_common/external/tcb/span.hpp>

namespace cras
{
  using ::tcb::span;
  using ::tcb::make_span;
  using ::tcb::get;
  using ::tcb::as_bytes;
  using ::tcb::as_writable_bytes;
  using ::tcb::dynamic_extent;
  using ::tcb::byte;
}

#endif
