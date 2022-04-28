#pragma once

/**
 * \file
 * \brief Computation of running average and variance using Welford's algorithm (implementation details, do not include
 *        directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 * 
 * Inspiration taken from https://www.johndcook.com/blog/skewness_kurtosis/ .
 */

#include <cmath>
#include <cstddef>

#include <cras_cpp_common/math_utils/running_stats.hpp>

namespace cras
{

template<typename T>
void RunningStats<T>::reset()
{
  this->count = 0u;
}

template<typename T>
void RunningStats<T>::addSample(T x)
{
  this->count++;

  if (this->count == 1)
  {
    this->mean = x;
    this->var = this->zero();
  }
  else
  {
    const auto newMean = this->mean + this->multiplyScalar(x - this->mean, 1.0 / this->count);
    this->var += this->multiply(x - this->mean, x - newMean);
    this->mean = newMean;
  }
}

template<typename T>
void RunningStats<T>::removeSample(T x)
{
  if (this->count == 0)
    return;

  if (this->count == 1)
  {
    this->count--;
    this->mean = this->zero();
    this->var = this->zero();
  }
  else
  {
    this->count--;
    const auto prevMean = this->mean - this->multiplyScalar(x - this->mean, 1.0 / this->count);
    this->var -= this->multiply(x - this->mean, x - prevMean);
    this->mean = prevMean;
  }
}

template<typename T>
size_t RunningStats<T>::getCount() const
{
  return this->count;
}

template<typename T>
T RunningStats<T>::getMean() const
{
  return (this->count > 0) ? this->mean : this->zero();
}

template<typename T>
T RunningStats<T>::getVariance() const
{
  return (this->count > 0) ? this->multiplyScalar(this->var, 1.0 / this->count) : this->zero();
}

template<typename T>
T RunningStats<T>::getSampleVariance() const
{
  return (this->count > 1) ? this->multiplyScalar(this->var, 1.0 / (this->count - 1)) : this->zero();
}

template<typename T>
T RunningStats<T>::getStandardDeviation() const
{
  return this->sqrt(this->getSampleVariance());
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator+=(const RunningStats<T>& other)
{
  const auto stats = *this + other;
  *this = stats;
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator+(const RunningStats<T>& other) const
{
  RunningStats<T> stats;
  stats.count = this->count + other.count;
  
  const auto sum = this->multiplyScalar(this->mean, this->count) + other.multiplyScalar(other.mean, other.count);
  stats.mean = stats.multiplyScalar(sum, 1.0 / stats.count);
  
  const auto meanDelta = other.mean - this->mean;
  stats.var = this->var + other.var +
    stats.multiplyScalar(stats.multiply(meanDelta, meanDelta), this->count * other.count / stats.count);
  
  return stats;
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator+=(const T& sample)
{
  this->addSample(sample);
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator+(const T& sample) const
{
  RunningStats<T> stats = *this;
  stats += sample;
  return stats;
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator-=(const RunningStats<T>& other)
{
  const auto stats = *this - other;
  *this = stats;
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator-(const RunningStats<T>& other) const
{
  RunningStats<T> stats;
  
  if (other.count > this->count)
    return stats;
  
  stats.count = this->count - other.count;
  
  if (stats.count == 0u)
    return stats;
  
  const auto sum = this->multiplyScalar(this->mean, this->count) - other.multiplyScalar(other.mean, other.count);
  stats.mean = stats.multiplyScalar(sum, 1.0 / stats.count);
  
  const auto meanDelta = other.mean - stats.mean;
  stats.var = this->var - other.var -
    stats.multiplyScalar(stats.multiply(meanDelta, meanDelta), stats.count * other.count / this->count);
  
  return stats;
}

template<typename T>
RunningStats<T>& RunningStats<T>::operator-=(const T& sample)
{
  this->removeSample(sample);
  return *this;
}

template<typename T>
RunningStats<T> RunningStats<T>::operator-(const T& sample) const
{
  RunningStats<T> stats = *this;
  stats -= sample;
  return stats;
}

template<typename T>
T RunningStats<T>::multiplyScalar(const T& val, double scalar)
{
  return val * scalar;
}

template<typename T>
T RunningStats<T>::multiply(const T& val1, const T& val2)
{
  return val1 * val2;
}

template<typename T>
T RunningStats<T>::sqrt(const T& val)
{
  return ::sqrt(val);
}

template<typename T>
T RunningStats<T>::zero()
{
  return 0;
}

}