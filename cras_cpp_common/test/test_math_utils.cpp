// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for math_utils.hpp.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <limits>

#include <cras_cpp_common/math_utils.hpp>
#include <cras_cpp_common/math_utils/running_stats.hpp>
#include <cras_cpp_common/math_utils/running_stats_duration.hpp>

#include <rclcpp/duration.hpp>

using namespace cras;

TEST(MathUtils, sgn) {  // NOLINT
  EXPECT_EQ(1, sgn(1.0));
  EXPECT_EQ(-1, sgn(-1.0));
  EXPECT_EQ(0, sgn(0.0));
  EXPECT_EQ(0, sgn(-0.0));
  EXPECT_EQ(1, sgn(0.1));
  EXPECT_EQ(-1, sgn(-0.1));
  EXPECT_EQ(1, sgn(1e-20));
  EXPECT_EQ(-1, sgn(-1e-20));
  EXPECT_EQ(1, sgn(1e20));
  EXPECT_EQ(-1, sgn(-1e20));
  EXPECT_EQ(1, sgn(std::numeric_limits<double>::infinity()));
  EXPECT_EQ(-1, sgn(-std::numeric_limits<double>::infinity()));
  EXPECT_EQ(0, sgn(-std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(typeid(int), typeid(sgn(0.0)));

  EXPECT_EQ(1, sgn(1.0f));
  EXPECT_EQ(-1, sgn(-1.0f));
  EXPECT_EQ(0, sgn(0.0f));
  EXPECT_EQ(typeid(int), typeid(sgn(0.0f)));

  EXPECT_EQ(1, sgn(1));
  EXPECT_EQ(-1, sgn(-1));
  EXPECT_EQ(0, sgn(0));
  EXPECT_EQ(typeid(int), typeid(sgn(0)));
}

#define EXPECT_DURATION_NEAR(d1, d2, eps) EXPECT_NEAR((d1).seconds(), (d2).seconds(), (eps))

template<typename T>
class TestRunningStats : public cras::RunningStats<T> {
public:
  using cras::RunningStats<T>::multiply;
  using cras::RunningStats<T>::multiplyScalar;
  using cras::RunningStats<T>::zero;
  using cras::RunningStats<T>::sqrt;
  using cras::RunningStats<T>::minValue;
  using cras::RunningStats<T>::maxValue;
};

template<typename T>
constexpr T _min() noexcept {
  return std::numeric_limits<T>::lowest();
}

template<typename T>
constexpr T _max() noexcept {
  return std::numeric_limits<T>::max();
}

TEST(SaturatingCast, SameZero) {
  // NOLINT
  // Check that it is constexpr
  static_assert(false == cras::saturating_cast<bool>(false));
  static_assert(static_cast<char>(0) == cras::saturating_cast<char>(static_cast<char>(0)));
  static_assert(static_cast<unsigned char>(0) == cras::saturating_cast<unsigned char>(static_cast<unsigned char>(0)));
  static_assert(static_cast<signed char>(0) == cras::saturating_cast<signed char>(static_cast<signed char>(0)));
  static_assert(static_cast<int8_t>(0) == cras::saturating_cast<int8_t>(static_cast<int8_t>(0)));
  static_assert(static_cast<uint8_t>(0) == cras::saturating_cast<uint8_t>(static_cast<uint8_t>(0)));
  static_assert(static_cast<short>(0) == cras::saturating_cast<short>(static_cast<short>(0)));
  static_assert(
    static_cast<unsigned short>(0) == cras::saturating_cast<unsigned short>(static_cast<unsigned short>(0)));
  static_assert(static_cast<int16_t>(0) == cras::saturating_cast<int16_t>(static_cast<int16_t>(0)));
  static_assert(static_cast<uint16_t>(0) == cras::saturating_cast<uint16_t>(static_cast<uint16_t>(0)));
  static_assert(static_cast<int>(0) == cras::saturating_cast<int>(static_cast<int>(0)));
  static_assert(static_cast<unsigned int>(0) == cras::saturating_cast<unsigned int>(static_cast<unsigned int>(0)));
  static_assert(static_cast<int32_t>(0) == cras::saturating_cast<int32_t>(static_cast<int32_t>(0)));
  static_assert(static_cast<uint32_t>(0) == cras::saturating_cast<uint32_t>(static_cast<uint32_t>(0)));
  static_assert(static_cast<long>(0) == cras::saturating_cast<long>(static_cast<long>(0)));
  static_assert(static_cast<unsigned long>(0) == cras::saturating_cast<unsigned long>(static_cast<unsigned long>(0)));
  static_assert(static_cast<int64_t>(0) == cras::saturating_cast<int64_t>(static_cast<int64_t>(0)));
  static_assert(static_cast<uint64_t>(0) == cras::saturating_cast<uint64_t>(static_cast<uint64_t>(0)));
  static_assert(static_cast<long long>(0) == cras::saturating_cast<long long>(static_cast<long long>(0)));
  static_assert(
    static_cast<unsigned long long>(0) == cras::saturating_cast<unsigned long long>(
      static_cast<unsigned long long>(0)));
  static_assert(static_cast<float>(0) == cras::saturating_cast<float>(static_cast<float>(0)));
  static_assert(static_cast<double>(0) == cras::saturating_cast<double>(static_cast<double>(0)));
  static_assert(static_cast<long double>(0) == cras::saturating_cast<long double>(static_cast<long double>(0)));

  EXPECT_EQ(false, cras::saturating_cast<bool>(false));
  EXPECT_EQ(static_cast<char>(0), cras::saturating_cast<char>(static_cast<char>(0)));
  EXPECT_EQ(static_cast<unsigned char>(0), cras::saturating_cast<unsigned char>(static_cast<unsigned char>(0)));
  EXPECT_EQ(static_cast<signed char>(0), cras::saturating_cast<signed char>(static_cast<signed char>(0)));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(static_cast<int8_t>(0)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<uint8_t>(0)));
  EXPECT_EQ(static_cast<short>(0), cras::saturating_cast<short>(static_cast<short>(0)));
  EXPECT_EQ(static_cast<unsigned short>(0), cras::saturating_cast<unsigned short>(static_cast<unsigned short>(0)));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(static_cast<int16_t>(0)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<uint16_t>(0)));
  EXPECT_EQ(static_cast<int>(0), cras::saturating_cast<int>(static_cast<int>(0)));
  EXPECT_EQ(static_cast<unsigned int>(0), cras::saturating_cast<unsigned int>(static_cast<unsigned int>(0)));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(static_cast<int32_t>(0)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<uint32_t>(0)));
  EXPECT_EQ(static_cast<long>(0), cras::saturating_cast<long>(static_cast<long>(0)));
  EXPECT_EQ(static_cast<unsigned long>(0), cras::saturating_cast<unsigned long>(static_cast<unsigned long>(0)));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(static_cast<int64_t>(0)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<uint64_t>(0)));
  EXPECT_EQ(static_cast<long long>(0), cras::saturating_cast<long long>(static_cast<long long>(0)));
  EXPECT_EQ(
    static_cast<unsigned long long>(0), cras::saturating_cast<unsigned long long>(static_cast<unsigned long long>(0)));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(static_cast<long double>(0)));
}

TEST(SaturatingCast, SameOne) {
  // NOLINT
  EXPECT_EQ(true, cras::saturating_cast<bool>(true));
  EXPECT_EQ(static_cast<char>(1), cras::saturating_cast<char>(static_cast<char>(1)));
  EXPECT_EQ(static_cast<unsigned char>(1), cras::saturating_cast<unsigned char>(static_cast<unsigned char>(1)));
  EXPECT_EQ(static_cast<signed char>(1), cras::saturating_cast<signed char>(static_cast<signed char>(1)));
  EXPECT_EQ(static_cast<int8_t>(1), cras::saturating_cast<int8_t>(static_cast<int8_t>(1)));
  EXPECT_EQ(static_cast<uint8_t>(1), cras::saturating_cast<uint8_t>(static_cast<uint8_t>(1)));
  EXPECT_EQ(static_cast<short>(1), cras::saturating_cast<short>(static_cast<short>(1)));
  EXPECT_EQ(static_cast<unsigned short>(1), cras::saturating_cast<unsigned short>(static_cast<unsigned short>(1)));
  EXPECT_EQ(static_cast<int16_t>(1), cras::saturating_cast<int16_t>(static_cast<int16_t>(1)));
  EXPECT_EQ(static_cast<uint16_t>(1), cras::saturating_cast<uint16_t>(static_cast<uint16_t>(1)));
  EXPECT_EQ(static_cast<int>(1), cras::saturating_cast<int>(static_cast<int>(1)));
  EXPECT_EQ(static_cast<unsigned int>(1), cras::saturating_cast<unsigned int>(static_cast<unsigned int>(1)));
  EXPECT_EQ(static_cast<int32_t>(1), cras::saturating_cast<int32_t>(static_cast<int32_t>(1)));
  EXPECT_EQ(static_cast<uint32_t>(1), cras::saturating_cast<uint32_t>(static_cast<uint32_t>(1)));
  EXPECT_EQ(static_cast<long>(1), cras::saturating_cast<long>(static_cast<long>(1)));
  EXPECT_EQ(static_cast<unsigned long>(1), cras::saturating_cast<unsigned long>(static_cast<unsigned long>(1)));
  EXPECT_EQ(static_cast<int64_t>(1), cras::saturating_cast<int64_t>(static_cast<int64_t>(1)));
  EXPECT_EQ(static_cast<uint64_t>(1), cras::saturating_cast<uint64_t>(static_cast<uint64_t>(1)));
  EXPECT_EQ(static_cast<long long>(1), cras::saturating_cast<long long>(static_cast<long long>(1)));
  EXPECT_EQ(
    static_cast<unsigned long long>(1), cras::saturating_cast<unsigned long long>(static_cast<unsigned long long>(1)));
  EXPECT_EQ(static_cast<float>(1), cras::saturating_cast<float>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<double>(1), cras::saturating_cast<double>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<long double>(1), cras::saturating_cast<long double>(static_cast<long double>(1)));
}

TEST(SaturatingCast, SameMinusOne) {
  // NOLINT
  EXPECT_EQ(static_cast<signed char>(-1), cras::saturating_cast<signed char>(static_cast<signed char>(-1)));
  EXPECT_EQ(static_cast<int8_t>(-1), cras::saturating_cast<int8_t>(static_cast<int8_t>(-1)));
  EXPECT_EQ(static_cast<short>(-1), cras::saturating_cast<short>(static_cast<short>(-1)));
  EXPECT_EQ(static_cast<int16_t>(-1), cras::saturating_cast<int16_t>(static_cast<int16_t>(-1)));
  EXPECT_EQ(static_cast<int>(-1), cras::saturating_cast<int>(static_cast<int>(-1)));
  EXPECT_EQ(static_cast<int32_t>(-1), cras::saturating_cast<int32_t>(static_cast<int32_t>(-1)));
  EXPECT_EQ(static_cast<long>(-1), cras::saturating_cast<long>(static_cast<long>(-1)));
  EXPECT_EQ(static_cast<int64_t>(-1), cras::saturating_cast<int64_t>(static_cast<int64_t>(-1)));
  EXPECT_EQ(static_cast<long long>(-1), cras::saturating_cast<long long>(static_cast<long long>(-1)));
  EXPECT_EQ(static_cast<float>(-1), cras::saturating_cast<float>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<double>(-1), cras::saturating_cast<double>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<long double>(-1), cras::saturating_cast<long double>(static_cast<long double>(-1)));

  static_assert(static_cast<uint8_t>(0) == cras::saturating_cast<uint8_t>(static_cast<int8_t>(-1)));

  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<int8_t>(-1)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<int16_t>(-1)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<int32_t>(-1)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<int64_t>(-1)));

  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<int16_t>(-1)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<int32_t>(-1)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<int64_t>(-1)));

  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<int32_t>(-1)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<int64_t>(-1)));
}

TEST(SaturatingCast, FloatZero) {
  // NOLINT
  EXPECT_EQ(static_cast<bool>(0), cras::saturating_cast<bool>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(static_cast<float>(0)));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(static_cast<float>(0)));

  EXPECT_EQ(static_cast<bool>(0), cras::saturating_cast<bool>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(static_cast<double>(0)));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(static_cast<double>(0)));

  EXPECT_EQ(static_cast<bool>(0), cras::saturating_cast<bool>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(static_cast<long double>(0)));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(static_cast<long double>(0)));
}

TEST(SaturatingCast, FloatOne) {
  // NOLINT
  EXPECT_EQ(static_cast<bool>(1), cras::saturating_cast<bool>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<int8_t>(1), cras::saturating_cast<int8_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<uint8_t>(1), cras::saturating_cast<uint8_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<int16_t>(1), cras::saturating_cast<int16_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<uint16_t>(1), cras::saturating_cast<uint16_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<int32_t>(1), cras::saturating_cast<int32_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<uint32_t>(1), cras::saturating_cast<uint32_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<int64_t>(1), cras::saturating_cast<int64_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<uint64_t>(1), cras::saturating_cast<uint64_t>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<double>(1), cras::saturating_cast<double>(static_cast<float>(1)));
  EXPECT_EQ(static_cast<long double>(1), cras::saturating_cast<long double>(static_cast<float>(1)));

  EXPECT_EQ(static_cast<bool>(1), cras::saturating_cast<bool>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<int8_t>(1), cras::saturating_cast<int8_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<uint8_t>(1), cras::saturating_cast<uint8_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<int16_t>(1), cras::saturating_cast<int16_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<uint16_t>(1), cras::saturating_cast<uint16_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<int32_t>(1), cras::saturating_cast<int32_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<uint32_t>(1), cras::saturating_cast<uint32_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<int64_t>(1), cras::saturating_cast<int64_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<uint64_t>(1), cras::saturating_cast<uint64_t>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<float>(1), cras::saturating_cast<float>(static_cast<double>(1)));
  EXPECT_EQ(static_cast<long double>(1), cras::saturating_cast<long double>(static_cast<double>(1)));

  EXPECT_EQ(static_cast<bool>(1), cras::saturating_cast<bool>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<int8_t>(1), cras::saturating_cast<int8_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<uint8_t>(1), cras::saturating_cast<uint8_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<int16_t>(1), cras::saturating_cast<int16_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<uint16_t>(1), cras::saturating_cast<uint16_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<int32_t>(1), cras::saturating_cast<int32_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<uint32_t>(1), cras::saturating_cast<uint32_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<int64_t>(1), cras::saturating_cast<int64_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<uint64_t>(1), cras::saturating_cast<uint64_t>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<float>(1), cras::saturating_cast<float>(static_cast<long double>(1)));
  EXPECT_EQ(static_cast<double>(1), cras::saturating_cast<double>(static_cast<long double>(1)));
}

TEST(SaturatingCast, FloatMinusOne) {
  // NOLINT
  EXPECT_EQ(static_cast<bool>(0), cras::saturating_cast<bool>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<int8_t>(-1), cras::saturating_cast<int8_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<int16_t>(-1), cras::saturating_cast<int16_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<int32_t>(-1), cras::saturating_cast<int32_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<int64_t>(-1), cras::saturating_cast<int64_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<double>(-1), cras::saturating_cast<double>(static_cast<float>(-1)));
  EXPECT_EQ(static_cast<long double>(-1), cras::saturating_cast<long double>(static_cast<float>(-1)));

  EXPECT_EQ(static_cast<bool>(0), cras::saturating_cast<bool>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<int8_t>(-1), cras::saturating_cast<int8_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<int16_t>(-1), cras::saturating_cast<int16_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<int32_t>(-1), cras::saturating_cast<int32_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<int64_t>(-1), cras::saturating_cast<int64_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<float>(-1), cras::saturating_cast<float>(static_cast<double>(-1)));
  EXPECT_EQ(static_cast<long double>(-1), cras::saturating_cast<long double>(static_cast<double>(-1)));

  EXPECT_EQ(static_cast<bool>(0), cras::saturating_cast<bool>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<int8_t>(-1), cras::saturating_cast<int8_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<int16_t>(-1), cras::saturating_cast<int16_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<int32_t>(-1), cras::saturating_cast<int32_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<int64_t>(-1), cras::saturating_cast<int64_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<float>(-1), cras::saturating_cast<float>(static_cast<long double>(-1)));
  EXPECT_EQ(static_cast<double>(-1), cras::saturating_cast<double>(static_cast<long double>(-1)));
}

TEST(SaturatingCast, Max) {  // NOLINT
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<bool>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<int8_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<uint8_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<int16_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<uint16_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<int32_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<uint32_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<int64_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<uint64_t>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<float>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<double>()));
  EXPECT_EQ(_max<bool>(), cras::saturating_cast<bool>(_max<long double>()));

  EXPECT_EQ(static_cast<int8_t>(1), cras::saturating_cast<int8_t>(_max<bool>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<int8_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<uint8_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<int16_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<uint16_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<int32_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<int64_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<float>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<double>()));
  EXPECT_EQ(_max<int8_t>(), cras::saturating_cast<int8_t>(_max<long double>()));

  static_assert(_max<int8_t>() == cras::saturating_cast<int8_t>(_max<float>()));

  EXPECT_EQ(static_cast<uint8_t>(1), cras::saturating_cast<uint8_t>(_max<bool>()));
  EXPECT_EQ(static_cast<uint8_t>(0x7fLL), cras::saturating_cast<uint8_t>(_max<int8_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<uint8_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<int16_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<uint16_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<int32_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<int64_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<float>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<double>()));
  EXPECT_EQ(_max<uint8_t>(), cras::saturating_cast<uint8_t>(_max<long double>()));

  EXPECT_EQ(static_cast<int16_t>(1), cras::saturating_cast<int16_t>(_max<bool>()));
  EXPECT_EQ(static_cast<int16_t>(0x7fLL), cras::saturating_cast<int16_t>(_max<int8_t>()));
  EXPECT_EQ(static_cast<int16_t>(0xffLL), cras::saturating_cast<int16_t>(_max<uint8_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<int16_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<uint16_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<int32_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<int64_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<float>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<double>()));
  EXPECT_EQ(_max<int16_t>(), cras::saturating_cast<int16_t>(_max<long double>()));

  EXPECT_EQ(static_cast<uint16_t>(1), cras::saturating_cast<uint16_t>(_max<bool>()));
  EXPECT_EQ(static_cast<uint16_t>(0x7fLL), cras::saturating_cast<uint16_t>(_max<int8_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0xffLL), cras::saturating_cast<uint16_t>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0x7fffLL), cras::saturating_cast<uint16_t>(_max<int16_t>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<uint16_t>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<int32_t>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<int64_t>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<float>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<double>()));
  EXPECT_EQ(_max<uint16_t>(), cras::saturating_cast<uint16_t>(_max<long double>()));

  EXPECT_EQ(static_cast<int32_t>(1), cras::saturating_cast<int32_t>(_max<bool>()));
  EXPECT_EQ(static_cast<int32_t>(0x7fLL), cras::saturating_cast<int32_t>(_max<int8_t>()));
  EXPECT_EQ(static_cast<int32_t>(0xffLL), cras::saturating_cast<int32_t>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<int32_t>(0x7fffLL), cras::saturating_cast<int32_t>(_max<int16_t>()));
  EXPECT_EQ(static_cast<int32_t>(0xffffLL), cras::saturating_cast<int32_t>(_max<uint16_t>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<int32_t>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<int64_t>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<float>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<double>()));
  EXPECT_EQ(_max<int32_t>(), cras::saturating_cast<int32_t>(_max<long double>()));

  EXPECT_EQ(static_cast<uint32_t>(1), cras::saturating_cast<uint32_t>(_max<bool>()));
  EXPECT_EQ(static_cast<uint32_t>(0x7fLL), cras::saturating_cast<uint32_t>(_max<int8_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0xffLL), cras::saturating_cast<uint32_t>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0x7fffLL), cras::saturating_cast<uint32_t>(_max<int16_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0xffffLL), cras::saturating_cast<uint32_t>(_max<uint16_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0x7fffffffLL), cras::saturating_cast<uint32_t>(_max<int32_t>()));
  EXPECT_EQ(_max<uint32_t>(), cras::saturating_cast<uint32_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<uint32_t>(), cras::saturating_cast<uint32_t>(_max<int64_t>()));
  EXPECT_EQ(_max<uint32_t>(), cras::saturating_cast<uint32_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<uint32_t>(), cras::saturating_cast<uint32_t>(_max<float>()));
  EXPECT_EQ(_max<uint32_t>(), cras::saturating_cast<uint32_t>(_max<double>()));
  EXPECT_EQ(_max<uint32_t>(), cras::saturating_cast<uint32_t>(_max<long double>()));

  EXPECT_EQ(static_cast<int64_t>(1), cras::saturating_cast<int64_t>(_max<bool>()));
  EXPECT_EQ(static_cast<int64_t>(0x7fLL), cras::saturating_cast<int64_t>(_max<int8_t>()));
  EXPECT_EQ(static_cast<int64_t>(0xffLL), cras::saturating_cast<int64_t>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<int64_t>(0x7fffLL), cras::saturating_cast<int64_t>(_max<int16_t>()));
  EXPECT_EQ(static_cast<int64_t>(0xffffLL), cras::saturating_cast<int64_t>(_max<uint16_t>()));
  EXPECT_EQ(static_cast<int64_t>(0x7fffffffLL), cras::saturating_cast<int64_t>(_max<int32_t>()));
  EXPECT_EQ(static_cast<int64_t>(0xffffffffLL), cras::saturating_cast<int64_t>(_max<uint32_t>()));
  EXPECT_EQ(_max<int64_t>(), cras::saturating_cast<int64_t>(_max<int64_t>()));
  EXPECT_EQ(_max<int64_t>(), cras::saturating_cast<int64_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<int64_t>(), cras::saturating_cast<int64_t>(_max<float>()));
  EXPECT_EQ(_max<int64_t>(), cras::saturating_cast<int64_t>(_max<double>()));
  EXPECT_EQ(_max<int64_t>(), cras::saturating_cast<int64_t>(_max<long double>()));

  EXPECT_EQ(static_cast<uint64_t>(1), cras::saturating_cast<uint64_t>(_max<bool>()));
  EXPECT_EQ(static_cast<uint64_t>(0x7fLL), cras::saturating_cast<uint64_t>(_max<int8_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0xffLL), cras::saturating_cast<uint64_t>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0x7fffLL), cras::saturating_cast<uint64_t>(_max<int16_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0xffffLL), cras::saturating_cast<uint64_t>(_max<uint16_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0x7fffffffLL), cras::saturating_cast<uint64_t>(_max<int32_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0xffffffffLL), cras::saturating_cast<uint64_t>(_max<uint32_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0x7fffffffffffffffULL), cras::saturating_cast<uint64_t>(_max<int64_t>()));
  EXPECT_EQ(_max<uint64_t>(), cras::saturating_cast<uint64_t>(_max<uint64_t>()));
  EXPECT_EQ(_max<uint64_t>(), cras::saturating_cast<uint64_t>(_max<float>()));
  EXPECT_EQ(_max<uint64_t>(), cras::saturating_cast<uint64_t>(_max<double>()));
  EXPECT_EQ(_max<uint64_t>(), cras::saturating_cast<uint64_t>(_max<long double>()));

  EXPECT_EQ(static_cast<float>(1), cras::saturating_cast<float>(_max<bool>()));
  EXPECT_EQ(static_cast<float>(0x7fLL), cras::saturating_cast<float>(_max<int8_t>()));
  EXPECT_EQ(static_cast<float>(0xffLL), cras::saturating_cast<float>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<float>(0x7fffLL), cras::saturating_cast<float>(_max<int16_t>()));
  EXPECT_EQ(static_cast<float>(0xffffLL), cras::saturating_cast<float>(_max<uint16_t>()));
  EXPECT_EQ(static_cast<float>(0x7fffffffLL), cras::saturating_cast<float>(_max<int32_t>()));
  EXPECT_EQ(static_cast<float>(0xffffffffLL), cras::saturating_cast<float>(_max<uint32_t>()));
  EXPECT_EQ(static_cast<float>(0x7fffffffffffffffULL), cras::saturating_cast<float>(_max<int64_t>()));
  EXPECT_EQ(static_cast<float>(0xffffffffffffffffULL), cras::saturating_cast<float>(_max<uint64_t>()));
  EXPECT_EQ(_max<float>(), cras::saturating_cast<float>(_max<float>()));
  EXPECT_EQ(std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(_max<double>()));
  EXPECT_EQ(std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(_max<long double>()));

  EXPECT_EQ(static_cast<double>(1), cras::saturating_cast<double>(_max<bool>()));
  EXPECT_EQ(static_cast<double>(0x7fLL), cras::saturating_cast<double>(_max<int8_t>()));
  EXPECT_EQ(static_cast<double>(0xffLL), cras::saturating_cast<double>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<double>(0x7fffLL), cras::saturating_cast<double>(_max<int16_t>()));
  EXPECT_EQ(static_cast<double>(0xffffLL), cras::saturating_cast<double>(_max<uint16_t>()));
  EXPECT_EQ(static_cast<double>(0x7fffffffLL), cras::saturating_cast<double>(_max<int32_t>()));
  EXPECT_EQ(static_cast<double>(0xffffffffLL), cras::saturating_cast<double>(_max<uint32_t>()));
  EXPECT_EQ(static_cast<double>(0x7fffffffffffffffULL), cras::saturating_cast<double>(_max<int64_t>()));
  EXPECT_EQ(static_cast<double>(0xffffffffffffffffULL), cras::saturating_cast<double>(_max<uint64_t>()));
  EXPECT_EQ(static_cast<double>(_max<float>()), cras::saturating_cast<double>(_max<float>()));
  EXPECT_EQ(_max<double>(), cras::saturating_cast<double>(_max<double>()));
  EXPECT_EQ(std::numeric_limits<double>::infinity(), cras::saturating_cast<double>(_max<long double>()));

  EXPECT_EQ(static_cast<long double>(1), cras::saturating_cast<long double>(_max<bool>()));
  EXPECT_EQ(static_cast<long double>(0x7fLL), cras::saturating_cast<long double>(_max<int8_t>()));
  EXPECT_EQ(static_cast<long double>(0xffLL), cras::saturating_cast<long double>(_max<uint8_t>()));
  EXPECT_EQ(static_cast<long double>(0x7fffLL), cras::saturating_cast<long double>(_max<int16_t>()));
  EXPECT_EQ(static_cast<long double>(0xffffLL), cras::saturating_cast<long double>(_max<uint16_t>()));
  EXPECT_EQ(static_cast<long double>(0x7fffffffLL), cras::saturating_cast<long double>(_max<int32_t>()));
  EXPECT_EQ(static_cast<long double>(0xffffffffLL), cras::saturating_cast<long double>(_max<uint32_t>()));
  EXPECT_EQ(static_cast<long double>(0x7fffffffffffffffULL), cras::saturating_cast<long double>(_max<int64_t>()));
  EXPECT_EQ(static_cast<long double>(0xffffffffffffffffULL), cras::saturating_cast<long double>(_max<uint64_t>()));
  EXPECT_EQ(static_cast<long double>(_max<float>()), cras::saturating_cast<long double>(_max<float>()));
  EXPECT_EQ(static_cast<long double>(_max<double>()), cras::saturating_cast<long double>(_max<double>()));
  EXPECT_EQ(_max<long double>(), cras::saturating_cast<long double>(_max<long double>()));
}

TEST(SaturatingCast, Min) {  // NOLINT
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<bool>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<int8_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<uint8_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<int16_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<uint16_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<int32_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<uint32_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<int64_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<uint64_t>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<float>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<double>()));
  EXPECT_EQ(_min<bool>(), cras::saturating_cast<bool>(_min<long double>()));

  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(_min<bool>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(_min<uint8_t>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(_min<uint16_t>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(_min<uint32_t>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<int8_t>(0), cras::saturating_cast<int8_t>(_min<uint64_t>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<float>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<double>()));
  EXPECT_EQ(_min<int8_t>(), cras::saturating_cast<int8_t>(_min<long double>()));

  static_assert(_min<int8_t>() == cras::saturating_cast<int8_t>(_min<float>()));

  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<bool>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<uint32_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<uint64_t>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<float>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<double>()));
  EXPECT_EQ(static_cast<uint8_t>(0), cras::saturating_cast<uint8_t>(_min<long double>()));

  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(_min<bool>()));
  EXPECT_EQ(static_cast<int16_t>(-0x80LL), cras::saturating_cast<int16_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(_min<uint8_t>()));
  EXPECT_EQ(_min<int16_t>(), cras::saturating_cast<int16_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(_min<uint16_t>()));
  EXPECT_EQ(_min<int16_t>(), cras::saturating_cast<int16_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(_min<uint32_t>()));
  EXPECT_EQ(_min<int16_t>(), cras::saturating_cast<int16_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<int16_t>(0), cras::saturating_cast<int16_t>(_min<uint64_t>()));
  EXPECT_EQ(_min<int16_t>(), cras::saturating_cast<int16_t>(_min<float>()));
  EXPECT_EQ(_min<int16_t>(), cras::saturating_cast<int16_t>(_min<double>()));
  EXPECT_EQ(_min<int16_t>(), cras::saturating_cast<int16_t>(_min<long double>()));

  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<bool>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<uint32_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<uint64_t>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<float>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<double>()));
  EXPECT_EQ(static_cast<uint16_t>(0), cras::saturating_cast<uint16_t>(_min<long double>()));

  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(_min<bool>()));
  EXPECT_EQ(static_cast<int32_t>(-0x80LL), cras::saturating_cast<int32_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<int32_t>(-0x8000LL), cras::saturating_cast<int32_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(_min<uint16_t>()));
  EXPECT_EQ(_min<int32_t>(), cras::saturating_cast<int32_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(_min<uint32_t>()));
  EXPECT_EQ(_min<int32_t>(), cras::saturating_cast<int32_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<int32_t>(0), cras::saturating_cast<int32_t>(_min<uint64_t>()));
  EXPECT_EQ(_min<int32_t>(), cras::saturating_cast<int32_t>(_min<float>()));
  EXPECT_EQ(_min<int32_t>(), cras::saturating_cast<int32_t>(_min<double>()));
  EXPECT_EQ(_min<int32_t>(), cras::saturating_cast<int32_t>(_min<long double>()));

  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<bool>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<uint32_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<uint64_t>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<float>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<double>()));
  EXPECT_EQ(static_cast<uint32_t>(0), cras::saturating_cast<uint32_t>(_min<long double>()));

  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(_min<bool>()));
  EXPECT_EQ(static_cast<int64_t>(-0x80LL), cras::saturating_cast<int64_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<int64_t>(-0x8000LL), cras::saturating_cast<int64_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<int64_t>(-0x80000000LL), cras::saturating_cast<int64_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(_min<uint32_t>()));
  EXPECT_EQ(_min<int64_t>(), cras::saturating_cast<int64_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<int64_t>(0), cras::saturating_cast<int64_t>(_min<uint64_t>()));
  EXPECT_EQ(_min<int64_t>(), cras::saturating_cast<int64_t>(_min<float>()));
  EXPECT_EQ(_min<int64_t>(), cras::saturating_cast<int64_t>(_min<double>()));
  EXPECT_EQ(_min<int64_t>(), cras::saturating_cast<int64_t>(_min<long double>()));

  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<bool>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<int8_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<int16_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<int32_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<uint32_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<int64_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<uint64_t>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<float>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<double>()));
  EXPECT_EQ(static_cast<uint64_t>(0), cras::saturating_cast<uint64_t>(_min<long double>()));

  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(_min<bool>()));
  EXPECT_EQ(static_cast<float>(-0x80LL), cras::saturating_cast<float>(_min<int8_t>()));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<float>(-0x8000LL), cras::saturating_cast<float>(_min<int16_t>()));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<float>(-0x80000000LL), cras::saturating_cast<float>(_min<int32_t>()));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(_min<uint32_t>()));
  EXPECT_EQ(-static_cast<float>(0x8000000000000000LL), cras::saturating_cast<float>(_min<int64_t>()));
  EXPECT_EQ(static_cast<float>(0), cras::saturating_cast<float>(_min<uint64_t>()));
  EXPECT_EQ(_min<float>(), cras::saturating_cast<float>(_min<float>()));
  EXPECT_EQ(-std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(_min<double>()));
  EXPECT_EQ(-std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(_min<long double>()));

  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(_min<bool>()));
  EXPECT_EQ(static_cast<double>(-0x80LL), cras::saturating_cast<double>(_min<int8_t>()));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<double>(-0x8000LL), cras::saturating_cast<double>(_min<int16_t>()));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<double>(-0x80000000LL), cras::saturating_cast<double>(_min<int32_t>()));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(_min<uint32_t>()));
  EXPECT_EQ(-static_cast<double>(0x8000000000000000LL), cras::saturating_cast<double>(_min<int64_t>()));
  EXPECT_EQ(static_cast<double>(0), cras::saturating_cast<double>(_min<uint64_t>()));
  EXPECT_EQ(static_cast<double>(_min<float>()), cras::saturating_cast<double>(_min<float>()));
  EXPECT_EQ(_min<double>(), cras::saturating_cast<double>(_min<double>()));
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), cras::saturating_cast<double>(_min<long double>()));

  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(_min<bool>()));
  EXPECT_EQ(static_cast<long double>(-0x80LL), cras::saturating_cast<long double>(_min<int8_t>()));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(_min<uint8_t>()));
  EXPECT_EQ(static_cast<long double>(-0x8000LL), cras::saturating_cast<long double>(_min<int16_t>()));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(_min<uint16_t>()));
  EXPECT_EQ(static_cast<long double>(-0x80000000LL), cras::saturating_cast<long double>(_min<int32_t>()));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(_min<uint32_t>()));
  EXPECT_EQ(-static_cast<long double>(0x8000000000000000LL), cras::saturating_cast<long double>(_min<int64_t>()));
  EXPECT_EQ(static_cast<long double>(0), cras::saturating_cast<long double>(_min<uint64_t>()));
  EXPECT_EQ(static_cast<long double>(_min<float>()), cras::saturating_cast<long double>(_min<float>()));
  EXPECT_EQ(static_cast<long double>(_min<double>()), cras::saturating_cast<long double>(_min<double>()));
  EXPECT_EQ(_min<long double>(), cras::saturating_cast<long double>(_min<long double>()));
}

TEST(SaturatingCast, Random) {
  // NOLINT
  EXPECT_EQ(127, cras::saturating_cast<int8_t>(200));
  static_assert(127 == cras::saturating_cast<int8_t>(200));
  EXPECT_EQ(-128, cras::saturating_cast<int8_t>(-200));
  static_assert(-128 == cras::saturating_cast<int8_t>(-200));

  EXPECT_EQ(32767, cras::saturating_cast<int16_t>(100'000));
  static_assert(32767 == cras::saturating_cast<int16_t>(100'000));
  EXPECT_EQ(-32768, cras::saturating_cast<int16_t>(-100'000));
  static_assert(-32768 == cras::saturating_cast<int16_t>(-100'000));

  EXPECT_EQ(std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(1e+40));
  static_assert(std::numeric_limits<float>::infinity() == cras::saturating_cast<float>(1e+40));
  EXPECT_EQ(-std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(-1e+40));
  static_assert(-std::numeric_limits<float>::infinity() == cras::saturating_cast<float>(-1e+40));

  EXPECT_EQ(0, cras::saturating_cast<int32_t>(std::numeric_limits<float>::quiet_NaN()));
  static_assert(0 == cras::saturating_cast<int32_t>(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_EQ(-0x80000000, cras::saturating_cast<int32_t>(-std::numeric_limits<float>::infinity()));
  static_assert(-0x80000000 == cras::saturating_cast<int32_t>(-std::numeric_limits<float>::infinity()));
  EXPECT_EQ(0x7fffffff, cras::saturating_cast<int32_t>(std::numeric_limits<float>::infinity()));
  static_assert(0x7fffffff == cras::saturating_cast<int32_t>(std::numeric_limits<float>::infinity()));
}

#ifdef __SIZEOF_INT128__
TEST(SaturatingCast, int128) {  // NOLINT
  constexpr auto UINT128_MAX = static_cast<__uint128_t>(static_cast<__int128_t>(-1L));
  constexpr __int128_t INT128_MAX = UINT128_MAX >> 1;
  constexpr __int128_t INT128_MIN = -INT128_MAX - 1;

  EXPECT_TRUE(std::isfinite(cras::saturating_cast<float>(INT128_MAX)));
  static_assert(std::isfinite(cras::saturating_cast<float>(INT128_MAX)));
  EXPECT_LT(1.7e+38, cras::saturating_cast<float>(INT128_MAX));
  static_assert(1.7e+38 < cras::saturating_cast<float>(INT128_MAX));
  EXPECT_TRUE(std::isfinite(cras::saturating_cast<float>(INT128_MIN)));
  static_assert(std::isfinite(cras::saturating_cast<float>(INT128_MIN)));
  EXPECT_GT(-1.7e+38, cras::saturating_cast<float>(INT128_MIN));
  static_assert(-1.7e+38 > cras::saturating_cast<float>(INT128_MIN));
  EXPECT_EQ(std::numeric_limits<float>::infinity(), cras::saturating_cast<float>(UINT128_MAX));
  static_assert(std::numeric_limits<float>::infinity() == cras::saturating_cast<float>(UINT128_MAX));
}
#endif

TEST(MathUtils, RunningStatsDouble) {  // NOLINT
  TestRunningStats<double> stats;

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_EQ(0.0, stats.getMean());
  EXPECT_EQ(0.0, stats.getVariance());
  EXPECT_EQ(0.0, stats.getSampleVariance());
  EXPECT_EQ(0.0, stats.getStandardDeviation());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  stats.addSample(2.0);

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_EQ(2.0, stats.getMean());
  EXPECT_EQ(0.0, stats.getVariance());
  EXPECT_EQ(0.0, stats.getSampleVariance());
  EXPECT_EQ(0.0, stats.getStandardDeviation());
  EXPECT_EQ(2.0, stats.getMin());
  EXPECT_EQ(2.0, stats.getMax());

  stats.addSample(2.0);

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_EQ(2.0, stats.getMean());
  EXPECT_EQ(0.0, stats.getVariance());
  EXPECT_EQ(0.0, stats.getSampleVariance());
  EXPECT_EQ(0.0, stats.getStandardDeviation());
  EXPECT_EQ(2.0, stats.getMin());
  EXPECT_EQ(2.0, stats.getMax());

  stats += 5.0;

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_NEAR(3.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(2.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(3.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(3.0), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(2.0, stats.getMin());
  EXPECT_EQ(5.0, stats.getMax());

  stats.addSample(7.0);

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_NEAR(4.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(4.5, stats.getVariance(), 1e-6);
  EXPECT_NEAR(6.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(6.0), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(2.0, stats.getMin());
  EXPECT_EQ(7.0, stats.getMax());

  stats.removeSample(7.0);

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_NEAR(3.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(2.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(3.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(3.0), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  // we can also remove out-of-order samples

  stats.removeSample(2.0);

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_NEAR(3.5, stats.getMean(), 1e-6);
  EXPECT_NEAR(2.25, stats.getVariance(), 1e-6);
  EXPECT_NEAR(4.5, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(4.5), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  stats -= 2.0;

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_NEAR(5.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  stats.removeSample(5.0);

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_NEAR(0.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  // Try once more on the empty sequence; nothing should happen
  stats.removeSample(5.0);

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_NEAR(0.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  stats.addSample(2.0);
  stats.addSample(2.0);
  stats.addSample(5.0);
  stats.addSample(7.0);

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_NEAR(4.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(4.5, stats.getVariance(), 1e-6);
  EXPECT_NEAR(6.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(6.0), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(2.0, stats.getMin());
  EXPECT_EQ(7.0, stats.getMax());

  stats.reset();

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_NEAR(0.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), stats.getMin());
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), stats.getMax());

  for (size_t i = 0; i < 100; ++i)
    stats.addSample(i + i / 100.0);

  EXPECT_EQ(100u, stats.getCount());
  EXPECT_NEAR(49.995, stats.getMean(), 1e-6);
  EXPECT_NEAR(849.998325, stats.getVariance(), 1e-6);
  EXPECT_NEAR(858.584167, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(sqrt(858.584167), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(0.0, stats.getMin());
  EXPECT_EQ(99 + 99 / 100.0, stats.getMax());

  RunningStats<double> stats1;
  RunningStats<double> stats2;
  stats1.addSample(2.0);
  stats1.addSample(5.0);
  stats2.addSample(2.0);
  stats2.addSample(7.0);

  const auto sumStats = stats1 + stats2;
  EXPECT_EQ(4u, sumStats.getCount());
  EXPECT_NEAR(4.0, sumStats.getMean(), 1e-6);
  EXPECT_NEAR(4.5, sumStats.getVariance(), 1e-6);
  EXPECT_NEAR(6.0, sumStats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(6.0), sumStats.getStandardDeviation(), 1e-6);

  const auto minusStats1 = sumStats - stats2;
  EXPECT_EQ(stats1.getCount(), minusStats1.getCount());
  EXPECT_NEAR(stats1.getMean(), minusStats1.getMean(), 1e-6);
  EXPECT_NEAR(stats1.getVariance(), minusStats1.getVariance(), 1e-6);
  EXPECT_NEAR(stats1.getSampleVariance(), minusStats1.getSampleVariance(), 1e-6);
  EXPECT_NEAR(stats1.getStandardDeviation(), minusStats1.getStandardDeviation(), 1e-6);

  const auto minusStats2 = sumStats - stats1;
  EXPECT_EQ(stats2.getCount(), minusStats2.getCount());
  EXPECT_NEAR(stats2.getMean(), minusStats2.getMean(), 1e-6);
  EXPECT_NEAR(stats2.getVariance(), minusStats2.getVariance(), 1e-6);
  EXPECT_NEAR(stats2.getSampleVariance(), minusStats2.getSampleVariance(), 1e-6);
  EXPECT_NEAR(stats2.getStandardDeviation(), minusStats2.getStandardDeviation(), 1e-6);

  const auto zeroStats = sumStats - sumStats;
  EXPECT_EQ(0u, zeroStats.getCount());
  EXPECT_NEAR(0.0, zeroStats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, zeroStats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, zeroStats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, zeroStats.getStandardDeviation(), 1e-6);

  const auto emptyStats = stats1 - sumStats;
  EXPECT_EQ(0u, emptyStats.getCount());
  EXPECT_NEAR(0.0, emptyStats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, emptyStats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, emptyStats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, emptyStats.getStandardDeviation(), 1e-6);
}

TEST(MathUtils, RunningStatsDuration) {  // NOLINT
  using D = rclcpp::Duration;
  const auto Dd = [](const double secs) {
    return D::from_seconds(secs);
  };
  const auto ZERO = D(0, 0);
  const auto MIN = D::from_nanoseconds(-D::max().nanoseconds());
  const auto MAX = D::max();
  TestRunningStats<D> stats;

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_EQ(ZERO, stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  stats.addSample(Dd(2.0));

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_EQ(D(2, 0), stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());
  EXPECT_EQ(D(2, 0), stats.getMin());
  EXPECT_EQ(D(2, 0), stats.getMax());

  stats.addSample(D(2, 0));

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_EQ(D(2, 0), stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());
  EXPECT_EQ(D(2, 0), stats.getMin());
  EXPECT_EQ(D(2, 0), stats.getMax());

  stats += D(5, 0);

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2, 0), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(3, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(sqrt(3.0)), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(D(2, 0), stats.getMin());
  EXPECT_EQ(D(5, 0), stats.getMax());

  stats.addSample(D(7, 0));

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_DURATION_NEAR(D(4, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(4.5), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(sqrt(6.0)), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(D(2, 0), stats.getMin());
  EXPECT_EQ(D(7, 0), stats.getMax());

  stats.removeSample(D(7, 0));

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2, 0), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(3, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(sqrt(3.0)), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  // we can also remove out-of-order samples

  stats.removeSample(D(2, 0));

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_DURATION_NEAR(Dd(3.5), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(2.25), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(4.5), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(sqrt(4.5)), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  stats -= D(2, 0);

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_DURATION_NEAR(D(5, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  stats.removeSample(D(5, 0));

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  // Try once more on the empty sequence; nothing should happen
  stats.removeSample(D(5, 0));

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  stats.addSample(D(2, 0));
  stats.addSample(D(2, 0));
  stats.addSample(D(5, 0));
  stats.addSample(D(7, 0));

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_DURATION_NEAR(Dd(4.0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(4.5), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(6.0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(sqrt(6.0)), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(D(2, 0), stats.getMin());
  EXPECT_EQ(D(7, 0), stats.getMax());

  stats.reset();

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, stats.getMin());
  EXPECT_EQ(MIN, stats.getMax());

  for (size_t i = 0; i < 100; ++i)
    stats.addSample(D(i, i * 10000000));

  EXPECT_EQ(100u, stats.getCount());
  EXPECT_DURATION_NEAR(Dd(49.995), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(849.998325), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(858.584167), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(sqrt(858.584167)), stats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(D(0, 0), stats.getMin());
  EXPECT_EQ(D(99, 99 * 10000000), stats.getMax());

  EXPECT_DURATION_NEAR(Dd(1.2345 * 2.3456), stats.multiply(Dd(1.2345), Dd(2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(Dd(-1.2345 * 2.3456), stats.multiply(Dd(-1.2345), Dd(2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(Dd(1.2345 * -2.3456), stats.multiply(Dd(1.2345), Dd(-2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(Dd(-1.2345 * -2.3456), stats.multiply(Dd(-1.2345), Dd(-2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(Dd(-1e5 * -2e4), stats.multiply(Dd(-1e5), Dd(-2e4)), 1e-6);
  const auto almostMax = MAX - D(1, 0);
  const auto sqrtAlmostMax = Dd(::sqrt(almostMax.seconds()));
  EXPECT_DURATION_NEAR(almostMax, stats.multiply(sqrtAlmostMax, sqrtAlmostMax), 1e-3);

  RunningStats<D> stats1;
  RunningStats<D> stats2;
  stats1.addSample(D(2, 0));
  stats1.addSample(D(5, 0));
  stats2.addSample(D(2, 0));
  stats2.addSample(D(7, 0));

  const auto sumStats = stats1 + stats2;
  EXPECT_EQ(4u, sumStats.getCount());
  EXPECT_DURATION_NEAR(Dd(4.0), sumStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(4.5), sumStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(6.0), sumStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(Dd(::sqrt(6.0)), sumStats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(D(2, 0), sumStats.getMin());
  EXPECT_EQ(D(7, 0), sumStats.getMax());

  const auto minusStats1 = sumStats - stats2;
  EXPECT_EQ(stats1.getCount(), minusStats1.getCount());
  EXPECT_DURATION_NEAR(stats1.getMean(), minusStats1.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getVariance(), minusStats1.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getSampleVariance(), minusStats1.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getStandardDeviation(), minusStats1.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, minusStats1.getMin());
  EXPECT_EQ(MIN, minusStats1.getMax());

  const auto minusStats2 = sumStats - stats1;
  EXPECT_EQ(stats2.getCount(), minusStats2.getCount());
  EXPECT_DURATION_NEAR(stats2.getMean(), minusStats2.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getVariance(), minusStats2.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getSampleVariance(), minusStats2.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getStandardDeviation(), minusStats2.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, minusStats2.getMin());
  EXPECT_EQ(MIN, minusStats2.getMax());

  const auto zeroStats = sumStats - sumStats;
  EXPECT_EQ(0u, zeroStats.getCount());
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, zeroStats.getMin());
  EXPECT_EQ(MIN, zeroStats.getMax());

  const auto emptyStats = stats1 - sumStats;
  EXPECT_EQ(0u, emptyStats.getCount());
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getStandardDeviation(), 1e-6);
  EXPECT_EQ(MAX, emptyStats.getMin());
  EXPECT_EQ(MIN, emptyStats.getMax());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
