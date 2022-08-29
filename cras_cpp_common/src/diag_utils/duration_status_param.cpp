/**
 * \file
 * \brief Definitions of parameters for a DurationStatus diagnostic task.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/duration.h>

#include <cras_cpp_common/diag_utils/duration_status_param.h>

namespace cras
{

DurationStatusParam::DurationStatusParam(const ros::Duration& minDuration, const ros::Duration& maxDuration,
  const double tolerance, const size_t windowSize, const bool noEventsIsOk) :
    minDuration(minDuration), maxDuration(maxDuration), tolerance(tolerance), windowSize(windowSize),
    noEventsIsOk(noEventsIsOk)
{
}

DurationStatusParam::DurationStatusParam(const DurationStatusParam& other) :
  cras::DurationStatusParam(other.minDuration, other.maxDuration, other.tolerance, other.windowSize, other.noEventsIsOk)
{
}

DurationStatusParam& DurationStatusParam::operator=(const DurationStatusParam& other)
{
  this->minDuration = other.minDuration;
  this->maxDuration = other.maxDuration;
  this->tolerance = other.tolerance;
  this->windowSize = other.windowSize;
  this->noEventsIsOk = other.noEventsIsOk;
  return *this;
}

DurationStatusParam::DurationStatusParam(const SimpleDurationStatusParam& params) :
  cras::DurationStatusParam(params.minDuration, params.maxDuration, params.tolerance, params.windowSize,
    params.noEventsIsOk)
{
}

}