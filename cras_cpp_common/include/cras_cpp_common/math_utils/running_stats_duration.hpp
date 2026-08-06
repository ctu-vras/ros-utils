#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Template specializations of RunningStats for rclcpp::Duration.
 * \author Martin Pecka
 */

#include <cmath>

#include <cras_cpp_common/math_utils/running_stats.hpp>

#include <rclcpp/duration.hpp>

namespace cras {

template<>
::rclcpp::Duration RunningStats<::rclcpp::Duration>::multiply(
  const ::rclcpp::Duration& val1, const ::rclcpp::Duration& val2) {
  const auto s1 = static_cast<int64_t>(val1.nanoseconds() / 1'000'000'000LL);
  const auto s2 = static_cast<int64_t>(val2.nanoseconds() / 1'000'000'000LL);
  const auto ns1 = static_cast<int64_t>(val1.nanoseconds() % 1'000'000'000LL);
  const auto ns2 = static_cast<int64_t>(val2.nanoseconds() % 1'000'000'000LL);
  return ::rclcpp::Duration::from_nanoseconds(
    s1 * s2 * 1'000'000'000LL +
    s1 * ns2 +
    s2 * ns1 +
    (ns1 * ns2) / 1'000'000'000LL);
}

template<>
::rclcpp::Duration RunningStats<::rclcpp::Duration>::sqrt(const ::rclcpp::Duration& val) {
  return ::rclcpp::Duration::from_seconds(::std::sqrt(val.seconds()));
}

template<>
::rclcpp::Duration RunningStats<::rclcpp::Duration>::zero() {
  return {0, 0};
}

template<>
::rclcpp::Duration RunningStats<::rclcpp::Duration>::minValue() {
  return ::rclcpp::Duration::from_nanoseconds(-::rclcpp::Duration::max().nanoseconds());
}

template<>
::rclcpp::Duration RunningStats<::rclcpp::Duration>::maxValue() {
  return ::rclcpp::Duration::max();
}

}  // namespace cras
