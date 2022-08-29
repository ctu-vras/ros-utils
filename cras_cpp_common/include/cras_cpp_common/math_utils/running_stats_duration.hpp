#pragma once

/**
 * \file
 * \brief Template specializations of RunningStats for ros::Duration and ros::WallDuration.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cmath>

#include <cras_cpp_common/math_utils/running_stats.hpp>

#include <ros/duration.h>

namespace cras
{

template<>
::ros::Duration RunningStats<::ros::Duration>::multiply(const ::ros::Duration& val1, const ::ros::Duration& val2)
{
  const auto s1 = static_cast<int64_t>(val1.sec);
  const auto s2 = static_cast<int64_t>(val2.sec);
  const auto ns1 = static_cast<int64_t>(val1.nsec);
  const auto ns2 = static_cast<int64_t>(val2.nsec);
  return ::ros::Duration().fromNSec(s1 * s2 * 1000000000LL + s1 * ns2 + s2 * ns1 + (ns1 * ns2) / 1000000000LL);
}

template<>
::ros::Duration RunningStats<::ros::Duration>::sqrt(const ::ros::Duration& val)
{
  return ::ros::Duration(::sqrt(val.toSec()));
}

template<>
::ros::Duration RunningStats<::ros::Duration>::zero()
{
  return {0, 0};
}

template<>
::ros::WallDuration RunningStats<::ros::WallDuration>::multiply(
  const ::ros::WallDuration& val1, const ::ros::WallDuration& val2)
{
  const auto s1 = static_cast<int64_t>(val1.sec);
  const auto s2 = static_cast<int64_t>(val2.sec);
  const auto ns1 = static_cast<int64_t>(val1.nsec);
  const auto ns2 = static_cast<int64_t>(val2.nsec);
  return ::ros::WallDuration().fromNSec(s1 * s2 * 1000000000LL + s1 * ns2 + s2 * ns1 + (ns1 * ns2) / 1000000000LL);
}

template<>
::ros::WallDuration RunningStats<::ros::WallDuration>::sqrt(const ::ros::WallDuration& val)
{
  return ::ros::WallDuration(::sqrt(val.toSec()));
}

template<>
::ros::WallDuration RunningStats<::ros::WallDuration>::zero()
{
  return {0, 0};
}

}
