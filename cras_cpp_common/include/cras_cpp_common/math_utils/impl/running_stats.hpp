#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Computation of running average and variance using Welford's algorithm (implementation details, do not include
 *        directly).
 * \author Martin Pecka
 *
 * Inspiration taken from https://www.johndcook.com/blog/skewness_kurtosis/ .
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <cras_cpp_common/math_utils/running_stats.hpp>

namespace cras {

template<typename T>
void RunningStats<T>::reset() {
  count_ = 0u;
  min_.reset();
  max_.reset();
}

template<typename T>
void RunningStats<T>::addSample(T x) {
  count_++;

  if (count_ == 1) {
    mean_ = x;
    var_ = zero();
  } else {
    const auto newMean = mean_ + multiplyScalar(x - mean_, 1.0 / count_);
    var_ += multiply(x - mean_, x - newMean);
    mean_ = newMean;
  }
  if (!min_.has_value() || x < *min_) {
    min_ = x;
  }
  if (!max_.has_value() || x > *max_) {
    max_ = x;
  }
}

template<typename T>
void RunningStats<T>::removeSample(T x) {
  if (count_ == 0) {
    return;
  }

  if (count_ == 1) {
    count_--;
    mean_ = zero();
    var_ = zero();
  } else {
    count_--;
    const auto prevMean = mean_ - multiplyScalar(x - mean_, 1.0 / count_);
    var_ -= multiply(x - mean_, x - prevMean);
    mean_ = prevMean;
  }
  min_.reset();
  max_.reset();
}

template<typename T>
size_t RunningStats<T>::getCount() const {
  return count_;
}

template<typename T>
T RunningStats<T>::getMean() const {
  return (count_ > 0) ? mean_ : zero();
}

template<typename T>
T RunningStats<T>::getVariance() const {
  return (count_ > 0) ? multiplyScalar(var_, 1.0 / count_) : zero();
}

template<typename T>
T RunningStats<T>::getSampleVariance() const {
  return (count_ > 1) ? multiplyScalar(var_, 1.0 / (count_ - 1)) : zero();
}

template<typename T>
T RunningStats<T>::getStandardDeviation() const {
  return sqrt(getSampleVariance());
}

template<typename T>
T RunningStats<T>::getMin() const {
  return min_.value_or(maxValue());
}

template<typename T>
T RunningStats<T>::getMax() const {
  return max_.value_or(minValue());
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator+=(const RunningStats<T>& other) {
  const auto stats = *this + other;
  *this = stats;
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator+(const RunningStats<T>& other) const {
  RunningStats<T> stats;
  stats.count_ = count_ + other.count_;

  const auto sum = multiplyScalar(mean_, count_) + other.multiplyScalar(other.mean_, other.count_);
  stats.mean_ = stats.multiplyScalar(sum, 1.0 / stats.count_);

  const auto meanDelta = other.mean_ - mean_;
  stats.var_ =
    var_ + other.var_ +
    stats.multiplyScalar(stats.multiply(meanDelta, meanDelta), count_ * other.count_ / stats.count_);

  stats.min_ = min_;
  if (min_.has_value()) {
    if (other.min_.has_value()) {
      stats.min_ = ::std::min(*min_, *other.min_);
    }
  } else {
    stats.min_ = other.min_;
  }

  stats.max_ = max_;
  if (max_.has_value()) {
    if (other.max_.has_value()) {
      stats.max_ = ::std::max(*max_, *other.max_);
    }
  } else {
    stats.max_ = other.max_;
  }

  return stats;
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator+=(const T& sample) {
  addSample(sample);
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator+(const T& sample) const {
  RunningStats<T> stats = *this;
  stats += sample;
  return stats;
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator-=(const RunningStats<T>& other) {
  const auto stats = *this - other;
  *this = stats;
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator-(const RunningStats<T>& other) const {
  RunningStats<T> stats;

  if (other.count_ > count_) {
    return stats;
  }

  stats.count_ = count_ - other.count_;

  if (stats.count_ == 0u) {
    return stats;
  }

  const auto sum = multiplyScalar(mean_, count_) - other.multiplyScalar(other.mean_, other.count_);
  stats.mean_ = stats.multiplyScalar(sum, 1.0 / stats.count_);

  const auto mean_delta = other.mean_ - stats.mean_;
  stats.var_ =
    var_ - other.var_ -
    stats.multiplyScalar(stats.multiply(mean_delta, mean_delta), stats.count_ * other.count_ / count_);

  stats.min_.reset();
  stats.max_.reset();

  return stats;
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator-=(const T& sample) {
  removeSample(sample);
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator-(const T& sample) const {
  RunningStats<T> stats = *this;
  stats -= sample;
  return stats;
}

template<typename T>
T RunningStats<T>::multiplyScalar(const T& val, double scalar) {
  return static_cast<T>(val * scalar);
}

template<typename T>
T RunningStats<T>::multiply(const T& val1, const T& val2) {
  return static_cast<T>(val1 * val2);
}

template<typename T>
T RunningStats<T>::sqrt(const T& val) {
  return static_cast<T>(::sqrt(val));
}

template<typename T>
T RunningStats<T>::zero() {
  return static_cast<T>(0);
}

template<typename T>
T RunningStats<T>::minValue() {
  if constexpr (std::numeric_limits<T>::has_infinity) {
    return -std::numeric_limits<T>::infinity();
  }
  return std::numeric_limits<T>::lowest();
}

template<typename T>
T RunningStats<T>::maxValue() {
  if constexpr (std::numeric_limits<T>::has_infinity) {
    return std::numeric_limits<T>::infinity();
  }
  return std::numeric_limits<T>::max();
}

}  // namespace cras
