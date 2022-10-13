/**
 * \file
 * \brief Unit test for math_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <limits>

#include <cras_cpp_common/math_utils.hpp>
#include <cras_cpp_common/math_utils/running_stats.hpp>
#include <cras_cpp_common/math_utils/running_stats_duration.hpp>

#include <ros/duration.h>

using namespace cras;

TEST(MathUtils, sgn)  // NOLINT
{
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

#define EXPECT_DURATION_NEAR(d1, d2, eps) EXPECT_NEAR((d1).toSec(), (d2).toSec(), (eps))

template<typename T>
class TestRunningStats : public cras::RunningStats<T>
{
public:
  using cras::RunningStats<T>::multiply;
  using cras::RunningStats<T>::multiplyScalar;
  using cras::RunningStats<T>::zero;
  using cras::RunningStats<T>::sqrt;
};

TEST(MathUtils, RunningStatsDouble)  // NOLINT
{
  TestRunningStats<double> stats;

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_EQ(0.0, stats.getMean());
  EXPECT_EQ(0.0, stats.getVariance());
  EXPECT_EQ(0.0, stats.getSampleVariance());
  EXPECT_EQ(0.0, stats.getStandardDeviation());

  stats.addSample(2.0);

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_EQ(2.0, stats.getMean());
  EXPECT_EQ(0.0, stats.getVariance());
  EXPECT_EQ(0.0, stats.getSampleVariance());
  EXPECT_EQ(0.0, stats.getStandardDeviation());

  stats.addSample(2.0);

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_EQ(2.0, stats.getMean());
  EXPECT_EQ(0.0, stats.getVariance());
  EXPECT_EQ(0.0, stats.getSampleVariance());
  EXPECT_EQ(0.0, stats.getStandardDeviation());

  stats += 5.0;

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_NEAR(3.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(2.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(3.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(3.0), stats.getStandardDeviation(), 1e-6);

  stats.addSample(7.0);

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_NEAR(4.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(4.5, stats.getVariance(), 1e-6);
  EXPECT_NEAR(6.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(6.0), stats.getStandardDeviation(), 1e-6);

  stats.removeSample(7.0);

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_NEAR(3.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(2.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(3.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(3.0), stats.getStandardDeviation(), 1e-6);

  // we can also remove out-of-order samples

  stats.removeSample(2.0);

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_NEAR(3.5, stats.getMean(), 1e-6);
  EXPECT_NEAR(2.25, stats.getVariance(), 1e-6);
  EXPECT_NEAR(4.5, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(4.5), stats.getStandardDeviation(), 1e-6);

  stats -= 2.0;

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_NEAR(5.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);

  stats.removeSample(5.0);

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_NEAR(0.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);

  // Try once more on the empty sequence; nothing should happen
  stats.removeSample(5.0);

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_NEAR(0.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);

  stats.addSample(2.0);
  stats.addSample(2.0);
  stats.addSample(5.0);
  stats.addSample(7.0);

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_NEAR(4.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(4.5, stats.getVariance(), 1e-6);
  EXPECT_NEAR(6.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(::sqrt(6.0), stats.getStandardDeviation(), 1e-6);

  stats.reset();

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_NEAR(0.0, stats.getMean(), 1e-6);
  EXPECT_NEAR(0.0, stats.getVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(0.0, stats.getStandardDeviation(), 1e-6);

  for (size_t i = 0; i < 100; ++i)
    stats.addSample(i + i / 100.0);

  EXPECT_EQ(100u, stats.getCount());
  EXPECT_NEAR(49.995, stats.getMean(), 1e-6);
  EXPECT_NEAR(849.998325, stats.getVariance(), 1e-6);
  EXPECT_NEAR(858.584167, stats.getSampleVariance(), 1e-6);
  EXPECT_NEAR(sqrt(858.584167), stats.getStandardDeviation(), 1e-6);

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

TEST(MathUtils, RunningStatsDuration)  // NOLINT
{
  using D = ros::Duration;
  const auto ZERO = D(0, 0);
  TestRunningStats<D> stats;

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_EQ(ZERO, stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());

  stats.addSample(D(2.0));

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_EQ(D(2, 0), stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());

  stats.addSample(D(2, 0));

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_EQ(D(2, 0), stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());

  stats += D(5, 0);

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2, 0), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(3, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(3.0)), stats.getStandardDeviation(), 1e-6);

  stats.addSample(D(7, 0));

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_DURATION_NEAR(D(4, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(6.0)), stats.getStandardDeviation(), 1e-6);

  stats.removeSample(D(7, 0));

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2, 0), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(3, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(3.0)), stats.getStandardDeviation(), 1e-6);

  // we can also remove out-of-order samples

  stats.removeSample(D(2, 0));

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3.5), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2.25), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(4.5)), stats.getStandardDeviation(), 1e-6);

  stats -= D(2, 0);

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_DURATION_NEAR(D(5, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  stats.removeSample(D(5, 0));

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  // Try once more on the empty sequence; nothing should happen
  stats.removeSample(D(5, 0));

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  stats.addSample(D(2, 0));
  stats.addSample(D(2, 0));
  stats.addSample(D(5, 0));
  stats.addSample(D(7, 0));

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_DURATION_NEAR(D(4.0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6.0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(6.0)), stats.getStandardDeviation(), 1e-6);

  stats.reset();

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  for (size_t i = 0; i < 100; ++i)
    stats.addSample(D(i, i * 10000000));

  EXPECT_EQ(100u, stats.getCount());
  EXPECT_DURATION_NEAR(D(49.995), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(849.998325), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(858.584167), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(858.584167)), stats.getStandardDeviation(), 1e-6);

  EXPECT_DURATION_NEAR(D(1.2345 * 2.3456), stats.multiply(D(1.2345), D(2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(-1.2345 * 2.3456), stats.multiply(D(-1.2345), D(2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(1.2345 * -2.3456), stats.multiply(D(1.2345), D(-2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(-1.2345 * -2.3456), stats.multiply(D(-1.2345), D(-2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(-1e5 * -2e4), stats.multiply(D(-1e5), D(-2e4)), 1e-6);
  const auto almostMax = ros::DURATION_MAX - D(1, 0);
  const auto sqrtAlmostMax = D(::sqrt(almostMax.toSec()));
  EXPECT_DURATION_NEAR(almostMax, stats.multiply(sqrtAlmostMax, sqrtAlmostMax), 1e-3);

  RunningStats<D> stats1;
  RunningStats<D> stats2;
  stats1.addSample(D(2, 0));
  stats1.addSample(D(5, 0));
  stats2.addSample(D(2, 0));
  stats2.addSample(D(7, 0));

  const auto sumStats = stats1 + stats2;
  EXPECT_EQ(4u, sumStats.getCount());
  EXPECT_DURATION_NEAR(D(4.0), sumStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), sumStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6.0), sumStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(::sqrt(6.0)), sumStats.getStandardDeviation(), 1e-6);

  const auto minusStats1 = sumStats - stats2;
  EXPECT_EQ(stats1.getCount(), minusStats1.getCount());
  EXPECT_DURATION_NEAR(stats1.getMean(), minusStats1.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getVariance(), minusStats1.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getSampleVariance(), minusStats1.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getStandardDeviation(), minusStats1.getStandardDeviation(), 1e-6);

  const auto minusStats2 = sumStats - stats1;
  EXPECT_EQ(stats2.getCount(), minusStats2.getCount());
  EXPECT_DURATION_NEAR(stats2.getMean(), minusStats2.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getVariance(), minusStats2.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getSampleVariance(), minusStats2.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getStandardDeviation(), minusStats2.getStandardDeviation(), 1e-6);

  const auto zeroStats = sumStats - sumStats;
  EXPECT_EQ(0u, zeroStats.getCount());
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getStandardDeviation(), 1e-6);

  const auto emptyStats = stats1 - sumStats;
  EXPECT_EQ(0u, emptyStats.getCount());
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getStandardDeviation(), 1e-6);
}

TEST(MathUtils, RunningStatsWallDuration)  // NOLINT
{
  using D = ros::WallDuration;
  const auto ZERO = D(0, 0);
  TestRunningStats<D> stats;

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_EQ(ZERO, stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());

  stats.addSample(D(2.0));

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_EQ(D(2, 0), stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());

  stats.addSample(D(2, 0));

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_EQ(D(2, 0), stats.getMean());
  EXPECT_EQ(ZERO, stats.getVariance());
  EXPECT_EQ(ZERO, stats.getSampleVariance());
  EXPECT_EQ(ZERO, stats.getStandardDeviation());

  stats += D(5, 0);

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2, 0), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(3, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(3.0)), stats.getStandardDeviation(), 1e-6);

  stats.addSample(D(7, 0));

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_DURATION_NEAR(D(4, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(6.0)), stats.getStandardDeviation(), 1e-6);

  stats.removeSample(D(7, 0));

  EXPECT_EQ(3u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2, 0), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(3, 0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(3.0)), stats.getStandardDeviation(), 1e-6);

  // we can also remove out-of-order samples

  stats.removeSample(D(2, 0));

  EXPECT_EQ(2u, stats.getCount());
  EXPECT_DURATION_NEAR(D(3.5), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(2.25), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(4.5)), stats.getStandardDeviation(), 1e-6);

  stats -= D(2, 0);

  EXPECT_EQ(1u, stats.getCount());
  EXPECT_DURATION_NEAR(D(5, 0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  stats.removeSample(D(5, 0));

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  // Try once more on the empty sequence; nothing should happen
  stats.removeSample(D(5, 0));

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  stats.addSample(D(2, 0));
  stats.addSample(D(2, 0));
  stats.addSample(D(5, 0));
  stats.addSample(D(7, 0));

  EXPECT_EQ(4u, stats.getCount());
  EXPECT_DURATION_NEAR(D(4.0), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6.0), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(6.0)), stats.getStandardDeviation(), 1e-6);

  stats.reset();

  EXPECT_EQ(0u, stats.getCount());
  EXPECT_DURATION_NEAR(ZERO, stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, stats.getStandardDeviation(), 1e-6);

  for (size_t i = 0; i < 100; ++i)
    stats.addSample(D(i, i * 10000000));

  EXPECT_EQ(100u, stats.getCount());
  EXPECT_DURATION_NEAR(D(49.995), stats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(849.998325), stats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(858.584167), stats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(sqrt(858.584167)), stats.getStandardDeviation(), 1e-6);

  EXPECT_DURATION_NEAR(D(1.2345 * 2.3456), stats.multiply(D(1.2345), D(2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(-1.2345 * 2.3456), stats.multiply(D(-1.2345), D(2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(1.2345 * -2.3456), stats.multiply(D(1.2345), D(-2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(-1.2345 * -2.3456), stats.multiply(D(-1.2345), D(-2.3456)), 1e-6);
  EXPECT_DURATION_NEAR(D(-1e5 * -2e4), stats.multiply(D(-1e5), D(-2e4)), 1e-6);
  const auto almostMax = ros::WallDuration(ros::DURATION_MAX.sec, ros::DURATION_MAX.nsec) - D(1, 0);
  const auto sqrtAlmostMax = D(::sqrt(almostMax.toSec()));
  EXPECT_DURATION_NEAR(almostMax, stats.multiply(sqrtAlmostMax, sqrtAlmostMax), 1e-3);

  RunningStats<D> stats1;
  RunningStats<D> stats2;
  stats1.addSample(D(2, 0));
  stats1.addSample(D(5, 0));
  stats2.addSample(D(2, 0));
  stats2.addSample(D(7, 0));

  const auto sumStats = stats1 + stats2;
  EXPECT_EQ(4u, sumStats.getCount());
  EXPECT_DURATION_NEAR(D(4.0), sumStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(D(4.5), sumStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(6.0), sumStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(D(::sqrt(6.0)), sumStats.getStandardDeviation(), 1e-6);

  const auto minusStats1 = sumStats - stats2;
  EXPECT_EQ(stats1.getCount(), minusStats1.getCount());
  EXPECT_DURATION_NEAR(stats1.getMean(), minusStats1.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getVariance(), minusStats1.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getSampleVariance(), minusStats1.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats1.getStandardDeviation(), minusStats1.getStandardDeviation(), 1e-6);

  const auto minusStats2 = sumStats - stats1;
  EXPECT_EQ(stats2.getCount(), minusStats2.getCount());
  EXPECT_DURATION_NEAR(stats2.getMean(), minusStats2.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getVariance(), minusStats2.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getSampleVariance(), minusStats2.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(stats2.getStandardDeviation(), minusStats2.getStandardDeviation(), 1e-6);

  const auto zeroStats = sumStats - sumStats;
  EXPECT_EQ(0u, zeroStats.getCount());
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, zeroStats.getStandardDeviation(), 1e-6);

  const auto emptyStats = stats1 - sumStats;
  EXPECT_EQ(0u, emptyStats.getCount());
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getMean(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getSampleVariance(), 1e-6);
  EXPECT_DURATION_NEAR(ZERO, emptyStats.getStandardDeviation(), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
