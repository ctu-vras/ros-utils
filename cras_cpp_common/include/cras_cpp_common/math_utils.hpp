#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Math utilities.
 * \author Martin Pecka
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <type_traits>

/**
 * \brief Return the sign of the given value (-1, 0 or +1).
 * \tparam T Type of the number.
 * \param val The value to get sign of.
 * \return Sign of the value: -1 for negative, 0 for 0, +1 for positive numbers.
 */
template<typename T> inline int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

namespace cras {

/**
 * \brief Cast the given value to the given type, saturating if necessary.
 * \tparam To The type to cast to.
 * \tparam From The type to cast from.
 * \param[in] from The value to cast.
 * \return The casted value.
 * \note NaNs are converted to 0 when To type is integral.
 * \note Overflowing integral values are converted to infinity when To type is floating point.
 */
template<
    typename To, typename From,
    typename = ::std::enable_if_t<::std::is_arithmetic_v<From> && ::std::is_arithmetic_v<To>>>
constexpr To saturating_cast(From from) noexcept {
  if constexpr (::std::is_same_v<From, To>) {
    return from;
  } else if constexpr (::std::is_integral_v<From> && ::std::is_integral_v<To>) {
#if defined(__cpp_lib_saturation_arithmetic) && __cpp_lib_saturation_arithmetic >= 202603L
    return ::std::saturating_cast<To>(from);
#elif defined(__cpp_lib_saturation_arithmetic) && __cpp_lib_saturation_arithmetic >= 202311L
    return ::std::saturate_cast<To>(from);
#else
    if constexpr (::std::numeric_limits<From>::digits <= ::std::numeric_limits<To>::digits) {
      if (::std::is_unsigned_v<To> && ::std::is_signed_v<From> && from < From(0)) {
        return To(0);
      } else {
        return To(from);
      }
    } else {
      constexpr auto to_max = static_cast<From>(std::numeric_limits<To>::max());
      if constexpr (::std::is_unsigned_v<From>) {
        return To(std::clamp(from, static_cast<From>(0), to_max));
      } else {
        constexpr auto to_min = static_cast<From>(std::numeric_limits<To>::lowest());
        return To(std::clamp(from, to_min, to_max));
      }
    }
#endif
  } else if constexpr (::std::is_integral_v<From> && !::std::is_integral_v<To>) {
    return To(from);
  } else if constexpr (!::std::is_integral_v<From> && ::std::is_integral_v<To>) {
    // adapted from https://isocpp.org/files/papers/P4355R0.html (BSL-1.0 license?)
    if (::std::isnan(from)) {
      return To(0);
    }
    if constexpr (::std::numeric_limits<To>::digits >= ::std::numeric_limits<From>::max_exponent) {
      if constexpr (::std::isinf(from)) {
        return from < From(0) ? ::std::numeric_limits<To>::lowest() : ::std::numeric_limits<To>::max();
      } else {
        return To(from);
      }
    } else if constexpr (::std::is_signed_v<To>) {
      constexpr From max = -From(::std::numeric_limits<To>::lowest());
      if (from >= max) {
        return ::std::numeric_limits<To>::max();
      }

      constexpr From min = -max;
      if (min - 1 < min) {
        // Numbers within 1 distance of lowest() get truncated to lowest(),
        // so lowest() is not the bound; the next lower integer is the bound.
        if (from <= min - 1) {
          return ::std::numeric_limits<To>::lowest();
        } else {
          return To(from);
        }
      } else if (from < min) {
        // In this case, there exists no integer directly below lowest()
        // due to limited floating-point precision.
        // This makes lowest() a genuine exclusive bound.
        return ::std::numeric_limits<To>::lowest();
      } else {
        return To(from);
      }
    } else {
      constexpr From max = From(::std::numeric_limits<To>::max());
      if (from >= max) {
        return ::std::numeric_limits<To>::max();
      }
      // Since the conversion truncates, negative values greater than -1 result in 0.
      // Cast UB only happens if the truncated value is not representable in the result.
      if (from <= From(-1)) {
        return ::std::numeric_limits<To>::lowest();
      } else {
        return To(from);
      }
    }
  } else {
    return static_cast<To>(from);
  }
}

}  // namespace cras
