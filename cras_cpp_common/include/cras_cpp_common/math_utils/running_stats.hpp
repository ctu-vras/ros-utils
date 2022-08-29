#pragma once

/**
 * \file
 * \brief Computation of running average and variance using Welford's algorithm.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cmath>
#include <cstddef>

namespace cras
{

/**
 * \brief Computation of running average and variance using Welford's algorithm.
 * \tparam T Type of the values. It has to support operator+ and operator- with T.
 *
 * This class can keep track of mean and variance of a stream of data using an efficient online algorithm.
 * The required memory is constant, and time to insert a new value and update the stats is also constant.
 *
 * You can also add and subtract instances of this class which behaves as if you merged / diffed the streams that
 * created the stats.
 */
template<typename T>
class RunningStats
{
public:
  /**
   * \brief Reset the statistics to represent an empty sequence.
   */
  void reset();

  /**
   * \brief Add a new sample to the statistics.
   * \param[in] x The sample to add.
   * \note This function runs in O(1) time.
   */
  void addSample(T x);

  /**
   * \brief Remove a sample from the statistics.
   * \param[in] x The sample to remove.
   * \note This function runs in O(1) time.
   * \note It is the responsibility of the caller to make sure the sample being removed has actually been added before.
   *       If that is not true, the computed statistics will stop being valid.
   * \note Removing from an empty instance does nothing.
   */
  void removeSample(T x);

  /**
   * \brief Get the number of stored samples.
   * \return The number of samples.
   */
  size_t getCount() const;

  /**
   * \brief Get the mean of stored samples.
   * \return The mean.
   * \note This function runs in O(1) time.
   */
  T getMean() const;

  /**
   * \brief Get the variance of stored samples.
   * \return The variance.
   * \note This function runs in O(1) time.
   */
  T getVariance() const;

  /**
   * \brief Get the sample variance of stored samples (like variance, but divided by count-1 instead of count,
   *        should be unbiased estimator).
   * \return The sample variance.
   * \note This function runs in O(1) time.
   */
  T getSampleVariance() const;

  /**
   * \brief Get standard deviation of stored samples.
   * \return The standard deviation.
   * \note The standard deviation computation is based on the sample variance.
   * \note This function runs in O(1) time.
   */
  T getStandardDeviation() const;

  /**
   * \brief Combine the two sequences represented by this and other and represent their joint stats.
   * \param[in] other The other stats to merge.
   * \return This.
   * \note This function runs in O(1) time.
   */
  RunningStats<T>& operator+=(const RunningStats<T>& other);

  /**
   * \brief Combine the two sequences represented by this and other and represent their joint stats.
   * \param[in] other The other stats to merge.
   * \return The combined stats.
   * \note This function runs in O(1) time.
   */
  RunningStats<T> operator+(const RunningStats<T>& other) const;

  /**
   * \brief Add the sample to the stats.
   * \param[in] sample The sample to add.
   * \return This.
   * \note This function runs in O(1) time.
   */
  RunningStats<T>& operator+=(const T& sample);

  /**
   * \brief Return a copy of this with the added sample.
   * \param[in] sample The sample to add.
   * \return The new stats.
   * \note This function runs in O(1) time.
   */
  RunningStats<T> operator+(const T& sample) const;

  /**
   * \brief Subtract the sequence represented by other from the sequence represented by this and update the stats.
   * \param[in] other The stats to subtract.
   * \return This.
   * \note If other is longer than this, empty stats will be returned.
   * \note This function runs in O(1) time.
   */
  RunningStats<T>& operator-=(const RunningStats<T>& other);

  /**
   * \brief Subtract the sequence represented by other from the sequence represented by this and return the
   *        updated stats.
   * \param[in] other The stats to subtract.
   * \return The stats with other removed.
   * \note If other is longer than this, empty stats will be returned.
   * \note This function runs in O(1) time.
   */
  RunningStats<T> operator-(const RunningStats<T>& other) const;

  /**
   * \brief Remove a sample from the statistics.
   * \param[in] sample The sample to remove.
   * \return This.
   * \note This function runs in O(1) time.
   * \note It is the responsibility of the caller to make sure the sample being removed has actually been added before.
   *       If that is not true, the computed statistics will stop being valid.
   * \note Removing from an empty instance does nothing.
   */
  RunningStats<T>& operator-=(const T& sample);

  /**
   * \brief Remove a sample from the statistics.
   * \param[in] sample The sample to remove.
   * \return The stats with sample removed.
   * \note This function runs in O(1) time.
   * \note It is the responsibility of the caller to make sure the sample being removed has actually been added before.
   *       If that is not true, the computed statistics will stop being valid.
   * \note Removing from an empty instance does nothing.
   */
  RunningStats<T> operator-(const T& sample) const;

protected:
  /**
   * \brief val * scalar
   * \param[in] val The value to multiply.
   * \param[in] scalar The scalar to multiply with.
   * \return val * scalar
   */
  static T multiplyScalar(const T& val, double scalar);

  /**
   * \brief val1 * val2
   * \param[in] val1 Multiplicand.
   * \param[in] val2 Multiplicand.
   * \return val1 * val2
   */
  static T multiply(const T& val1, const T& val2);

  /**
   * \brief Return the square root of the given value (whatever meaning that might have).
   * \param[in] val The value to take root of.
   * \return The square root.
   */
  static T sqrt(const T& val);

  /**
   * \brief Return a T value representing zero.
   * \return 0.
   */
  static T zero();

  //! \brief Number of represented samples.
  size_t count {0u};

  //! \brief Mean of represented samples.
  T mean {RunningStats<T>::zero()};

  //! \brief Sk term of the computation such that var(X0...Xk) = this->var/this->count.
  T var {RunningStats<T>::zero()};
};

}

#include "impl/running_stats.hpp"
