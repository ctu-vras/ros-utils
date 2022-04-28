/**
 * \file
 * \brief Diagnostic task for duration of some task.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <algorithm>
#include <mutex>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/diag_utils/duration_status.h>
#include <cras_cpp_common/diag_utils/duration_status_param.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>

using namespace cras;

DurationStatus::DurationStatus(const std::string& name, const DurationStatusParam& params) :
  diagnostic_updater::DiagnosticTask(name), params(params), minDurations(params.windowSize, ros::DURATION_MAX),
  maxDurations(params.windowSize, {0, 0}), historyTimes(params.windowSize, ros::Time::now()),
  historyStats(params.windowSize)
{
  this->historyTimes[0] = ros::Time::now();
}

DurationStatus::DurationStatus(const std::string& name, const ros::Duration& minDuration,
  const ros::Duration& maxDuration, const double tolerance, const size_t windowSize, const bool noEventsIsOk) :
  DurationStatus(name, DurationStatusParam(minDuration, maxDuration, tolerance, windowSize, noEventsIsOk))
{
}

DurationStatus::DurationStatus(const std::string& name, const SimpleDurationStatusParam& params) :
  DurationStatus(name, DurationStatusParam(params))
{
}

DurationStatus::DurationStatus(const std::string& name, const BoundParamHelperPtr& params,
  const DurationStatusParam& defaultParams) : diagnostic_updater::DiagnosticTask(name)
{
  this->params = defaultParams;
  this->params.minDuration = params->getParam("min_duration", this->params.minDuration, "s");
  this->params.maxDuration = params->getParam("max_duration", this->params.maxDuration, "s");
  this->params.tolerance = params->getParam("tolerance", this->params.tolerance);
  this->params.windowSize = params->getParam("window_size", this->params.windowSize, "updates");
  this->params.noEventsIsOk = params->getParam("no_events_is_ok", this->params.noEventsIsOk);
  this->minDurations.resize(this->params.windowSize, ros::DURATION_MAX);
  this->maxDurations.resize(this->params.windowSize, {0, 0});
  this->historyTimes.resize(this->params.windowSize, ros::Time::now());
  this->historyStats.resize(this->params.windowSize);
}

DurationStatus::DurationStatus(const std::string& name, const BoundParamHelperPtr& params,
  const SimpleDurationStatusParam& defaultParams) : DurationStatus(name, params, DurationStatusParam(defaultParams))
{
}

DurationStatus::~DurationStatus() = default;

void DurationStatus::start(const ros::Time& time)
{
  std::lock_guard<std::mutex> l(this->lock);
  if (this->lastStartTime.has_value())
    ROS_WARN_THROTTLE(1.0, "DurationStatus::start() called before a matching stop().");
  this->lastStartTime = time;
}

void DurationStatus::start(const ros::WallTime& time)
{
  this->wallTimeMode = true;
  this->start(ros::Time(time.sec, time.nsec));
}

void DurationStatus::stop(const ros::Time& time)
{
  std::lock_guard<std::mutex> l(this->lock);
  if (!this->lastStartTime.has_value())
  {
    ROS_WARN_THROTTLE(1.0, "DurationStatus::stop() called before a matching start(). Ignoring.");
    return;
  }
  const auto duration = time - this->lastStartTime.value();
  this->lastStartTime.reset();
  this->minDurations[this->historyIndex] = (std::min)(duration, this->minDurations[this->historyIndex]);
  this->maxDurations[this->historyIndex] = (std::max)(duration, this->maxDurations[this->historyIndex]);
  this->historyStats[this->historyIndex].addSample(duration);
  this->stats.addSample(duration);
  this->count++;
}

void DurationStatus::stop(const ros::WallTime& time)
{
  this->stop(ros::Time(time.sec, time.nsec));
}

void DurationStatus::run(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::lock_guard<std::mutex> l(this->lock);
  const auto now = ros::Time::now();
  const auto numEvents = this->stats.getCount();
  const auto windowDuration = (now - this->historyTimes[this->historyIndex]).toSec();

  ros::Duration minDuration {ros::DURATION_MAX};
  ros::Duration maxDuration {0, 0};

  for (const auto& duration : this->minDurations)
    minDuration = (std::min)(duration, minDuration);
  for (const auto& duration : this->maxDurations)
    maxDuration = (std::max)(duration, maxDuration);

  const auto meanDuration = this->stats.getMean();
  const auto stdDuration = this->stats.getStandardDeviation();

  this->historyTimes[this->historyIndex] = now;
  this->stats -= this->historyStats[this->historyIndex];
  this->historyStats[this->historyIndex].reset();
  this->minDurations[this->historyIndex] = ros::DURATION_MAX;
  this->maxDurations[this->historyIndex] = {0, 0};
  this->historyIndex = (this->historyIndex + 1) % this->params.windowSize;

  const auto minDurationWithTolerance = this->params.minDuration * (1 - this->params.tolerance);
  const auto maxDurationWithTolerance = this->params.maxDuration == ros::DURATION_MAX ?
    this->params.maxDuration : this->params.maxDuration * (1 + this->params.tolerance);

  using diagnostic_msgs::DiagnosticStatus;

  if (numEvents == 0)
  {
    stat.summary(this->params.noEventsIsOk ? DiagnosticStatus::OK : DiagnosticStatus::ERROR, "No events recorded.");
  }
  else
  {
    if (windowDuration != 0 && minDuration < minDurationWithTolerance)
    {
      stat.summary(DiagnosticStatus::WARN, "Duration too short.");
    }
    else if (windowDuration != 0 && maxDuration > maxDurationWithTolerance)
    {
      stat.summary(DiagnosticStatus::WARN, "Duration too long.");
    }
    else if (windowDuration != 0)
    {
      stat.summary(DiagnosticStatus::OK, "Duration within limits.");
    }
  }

  stat.addf("Events in window", "%d", numEvents);
  stat.addf("Events since startup", "%d", this->count);
  stat.addf("Duration of window (s)", "%f", windowDuration);
  if (windowDuration != 0)
  {
    stat.add("Minimum observed duration (s)", minDuration);
    stat.add("Maximum observed duration (s)", maxDuration);
    stat.add("Mean observed duration (s)", meanDuration);
    stat.add("Observed duration standard deviation (s)", stdDuration);
  }

  if (this->params.minDuration == this->params.maxDuration)
    stat.add("Target duration (s)", this->params.minDuration);

  stat.add("Minimum acceptable duration (s)", minDurationWithTolerance);

  if (maxDurationWithTolerance == ros::DURATION_MAX)
    stat.add("Maximum acceptable duration (s)", "No limit");
  else
    stat.add("Maximum acceptable duration (s)", maxDurationWithTolerance);

  stat.add("Time mode", this->wallTimeMode ? "Wall time" : "Sim time");
}

const ros::Duration& DurationStatus::getMinDuration() const
{
  return this->params.minDuration;
}

const ros::Duration& DurationStatus::getMaxDuration() const
{
  return this->params.maxDuration;
}

double DurationStatus::getTolerance() const
{
  return this->params.tolerance;
}

size_t DurationStatus::getWindowSize() const
{
  return this->params.windowSize;
}
