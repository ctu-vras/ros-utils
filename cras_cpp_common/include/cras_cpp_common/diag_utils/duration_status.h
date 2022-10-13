#pragma once

/**
 * \file
 * \brief Diagnostic task for duration of some task.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <cras_cpp_common/diag_utils/duration_status_param.h>
#include <cras_cpp_common/math_utils/running_stats_duration.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>

namespace cras
{

/**
 * \brief Diagnostic task for topic frequency and timestamp delay (combining FrequencyStatus and TimeStampStatus tasks).
 * \tparam Message Type of the message. If it contains a header field, the task will automatically check both frequency
 * and timestamp delay. Header-less messages will only have their frequency checked.
 */
class DurationStatus : public ::diagnostic_updater::DiagnosticTask
{
public:
  /**
   * \brief Create the diagnostic task for a header-less message (checking frequency only).
   * \tparam M SFINAE only. Do not set explicitly.
   * \param[in] name Name of the diagnostic task.
   * \param[in] params Parameters of the task.
   */
  DurationStatus(const ::std::string& name, const ::cras::DurationStatusParam& params);

  /**
   * \brief Create the diagnostic task for a header-less message (checking frequency only).
   * \tparam M SFINAE only. Do not set explicitly.
   * \param[in] name Name of the diagnostic task.
   * \param[in] minRate Minimum allowed frequency.
   * \param[in] maxRate Maximum allowed frequency.
   * \param[in] rateTolerance Tolerance of the rate.
   * \param[in] rateWindowSize Number of updates during which the frequency is computed.
   */
  explicit DurationStatus(const ::std::string& name,
    const ::ros::Duration& minDuration = {0, 0}, const ::ros::Duration& maxDuration = ::ros::DURATION_MAX,
    double tolerance = 0.1, size_t windowSize = 5u, bool noEventsIsOk = true);

  /**
   * \brief Create the diagnostic task checking frequency of messages and timestamp delay (if the message has header).
   * \param[in] name Name of the diagnostic task.
   * \param[in] params Parameters of the task.
   */
  DurationStatus(const ::std::string& name, const ::cras::SimpleDurationStatusParam& params);

  DurationStatus(const ::std::string& name, const ::cras::BoundParamHelperPtr& params,
                 const ::cras::DurationStatusParam& defaultParams);

  DurationStatus(const ::std::string& name, const ::cras::BoundParamHelperPtr& params,
                 const ::cras::SimpleDurationStatusParam& defaultParams);

  ~DurationStatus() override;

  /**
   * \brief Start a single duration measurement.
   * \param[in] time Time of start.
   */
  void start(const ::ros::Time& time = ::ros::Time::now());

  /**
   * \brief Start a single duration measurement.
   * \param[in] time Time of start.
   */
  void start(const ::ros::WallTime& time);

  /**
   * \brief Stop a single duration measurement.
   * \param[in] time Time of end.
   */
  void stop(const ::ros::Time& time = ::ros::Time::now());

  /**
   * \brief Stop a single duration measurement.
   * \param[in] time Time of end.
   */
  void stop(const ::ros::WallTime& time);

  void run(::diagnostic_updater::DiagnosticStatusWrapper& stat) override;

  /**
   * \brief Minimum allowed duration.
   * \return The duration.
   */
  const ::ros::Duration& getMinDuration() const;

  /**
   * \brief Maximum allowed duration.
   * \return The duration.
   */
  const ::ros::Duration& getMaxDuration() const;

  /**
   * \brief Tolerance of duration.
   * \return The tolerance (0.0 means exact match of the duration limits).
   */
  double getTolerance() const;

  /**
   * \brief Number of updates during which the duration is computed.
   * \return The window size.
   */
  size_t getWindowSize() const;

protected:
  //! \brief The parameters via which this task has been configured.
  ::cras::DurationStatusParam params;

  bool wallTimeMode {false};
  size_t count {0u};
  ::cras::RunningStats<::ros::Duration> stats;
  ::cras::optional<::ros::Time> lastStartTime;
  ::std::vector<::ros::Duration> minDurations;
  ::std::vector<::ros::Duration> maxDurations;
  ::std::vector<::ros::Time> historyTimes;
  ::std::vector<::cras::RunningStats<::ros::Duration>> historyStats;
  size_t historyIndex {0u};
  ::std::mutex lock;
};

using DurationStatusPtr = ::std::shared_ptr<::cras::DurationStatus>;

}
