/**
 * \file
 * \brief Definitions of parameters for a TopicStatus diagnostic task.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cras_cpp_common/diag_utils/topic_status_param.hpp>

#include <diagnostic_updater/update_functions.h>

using namespace cras;

FrequencyStatusParam::FrequencyStatusParam(
  double* min_freq, double* max_freq, const double tolerance, const int window_size) :
    diagnostic_updater::FrequencyStatusParam(min_freq, max_freq, tolerance, window_size)
{
}

FrequencyStatusParam::FrequencyStatusParam(
  const double min_freq, const double max_freq, const double tolerance, const int window_size) :
  cras::FrequencyStatusParam(&this->minFreq, &this->maxFreq, tolerance, window_size)
{
  this->minFreq = min_freq;
  this->maxFreq = max_freq;
  this->usesInternalPointers = true;
}

FrequencyStatusParam::FrequencyStatusParam(const cras::FrequencyStatusParam& other) :
  cras::FrequencyStatusParam(other.min_freq_, other.max_freq_, other.tolerance_, other.window_size_)
{
  this->copyAdditionalData(other);
}

FrequencyStatusParam::FrequencyStatusParam(const diagnostic_updater::FrequencyStatusParam& other) :
  cras::FrequencyStatusParam(other.min_freq_, other.max_freq_, other.tolerance_, other.window_size_)
{
}

cras::FrequencyStatusParam& FrequencyStatusParam::operator=(const cras::FrequencyStatusParam& other)
{
  diagnostic_updater::FrequencyStatusParam::operator=(other);
  this->copyAdditionalData(other);
  return *this;
}

cras::FrequencyStatusParam& FrequencyStatusParam::operator=(const diagnostic_updater::FrequencyStatusParam& other)
{
  diagnostic_updater::FrequencyStatusParam::operator=(other);
  return *this;
}

void FrequencyStatusParam::copyAdditionalData(const cras::FrequencyStatusParam& other)
{
  this->minFreq = other.minFreq;
  this->maxFreq = other.maxFreq;
  this->usesInternalPointers = other.usesInternalPointers;
  if (this->usesInternalPointers)
  {
    this->min_freq_ = &this->minFreq;
    this->max_freq_ = &this->maxFreq;
  }
}

double FrequencyStatusParam::getExpectedRate() const
{
  if (*this->min_freq_ <= 0)
    return *this->max_freq_;
  else if (!std::isfinite(*this->max_freq_))
    return *this->min_freq_;
  return (*this->max_freq_ + *this->min_freq_) / 2.0;
}

FrequencyStatusParam::FrequencyStatusParam(const SimpleTopicStatusParamNoHeader& params) :
  cras::FrequencyStatusParam(params.minRate, params.maxRate, params.rateTolerance, params.rateWindowSize)
{
}

TopicStatusParamWithHeader::TopicStatusParamWithHeader(
  double* min_freq, double* max_freq, const double tolerance, const int window_size,
  const double min_acceptable, const double max_acceptable) :
    cras::FrequencyStatusParam(min_freq, max_freq, tolerance, window_size),
    diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)
{
}

TopicStatusParamWithHeader::TopicStatusParamWithHeader(
  const double min_freq, const double max_freq, const double tolerance, const int window_size,
  const double min_acceptable, const double max_acceptable) :
    cras::FrequencyStatusParam(min_freq, max_freq, tolerance, window_size),
    diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)
{
}

TopicStatusParamWithHeader::TopicStatusParamWithHeader(const diagnostic_updater::FrequencyStatusParam& freqParam) :
  TopicStatusParamWithHeader(freqParam.min_freq_, freqParam.max_freq_, freqParam.tolerance_, freqParam.window_size_)
{
}

TopicStatusParamWithHeader::TopicStatusParamWithHeader(const diagnostic_updater::TimeStampStatusParam& stampParam) :
  TopicStatusParamWithHeader(0.0, std::numeric_limits<double>::infinity(), 0.1, 5,
    stampParam.min_acceptable_, stampParam.max_acceptable_)
{
}

TopicStatusParamWithHeader::TopicStatusParamWithHeader(const diagnostic_updater::FrequencyStatusParam& freqParam,
  const diagnostic_updater::TimeStampStatusParam& stampParam) :
    TopicStatusParamWithHeader(freqParam.min_freq_, freqParam.max_freq_, freqParam.tolerance_, freqParam.window_size_,
      stampParam.min_acceptable_, stampParam.max_acceptable_)
{
}

TopicStatusParamWithHeader::TopicStatusParamWithHeader(const SimpleTopicStatusParamWithHeader& params) :
  TopicStatusParamWithHeader(params.minRate, params.maxRate, params.rateTolerance, params.rateWindowSize,
    params.minDelay, params.maxDelay)
{
}
