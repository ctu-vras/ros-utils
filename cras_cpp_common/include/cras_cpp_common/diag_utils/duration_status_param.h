#pragma once

/**
 * \file
 * \brief Definitions of parameters for a DurationStatus diagnostic task.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/duration.h>

namespace cras
{

struct SimpleDurationStatusParam;

/**
 * \brief Parameters of DurationStatus diagnostic task.
 */
struct DurationStatusParam
{
  /**
   * \brief Construct the parameter object using the passed values.
   * \param[in] minDuration Minimum allowed duration.
   * \param[in] maxDuration Maximum allowed duration.
   * \param[in] tolerance Tolerance of the duration.
   * \param[in] windowSize Number of updates during which the duration stats are considered.
   * \param[in] noEventsIsOk Whether it is okay when there are no events between two update calls.
   */
  explicit DurationStatusParam(
    const ::ros::Duration& minDuration = {0, 0}, const ::ros::Duration& maxDuration = ::ros::DURATION_MAX,
    double tolerance = 0.1, size_t windowSize = 5, bool noEventsIsOk = true);

  /**
   * \brief Copy constructor.
   * \param[in] other The object to copy from.
   */
  DurationStatusParam(const ::cras::DurationStatusParam& other);  // NOLINT

  /**
   * \brief Initialize this object from `SimpleDuraitonStatusParam`.
   * \param[in] params The parameters to initialize with.
   */
  DurationStatusParam(const ::cras::SimpleDurationStatusParam& params);  // NOLINT

  /**
   * \brief Copy assignment operator.
   * \param[in] other The object to copy from.
   */
  ::cras::DurationStatusParam& operator=(const ::cras::DurationStatusParam& other);

  //! \brief Minimum duration considered as valid.
  ::ros::Duration minDuration {0, 0};

  //! \brief Maximum duration considered as valid.
  ::ros::Duration maxDuration {::ros::DURATION_MAX};

  //! \brief Tolerance of the duration.
  double tolerance {0.1};

  //! \brief Number of updates during which the duration is computed.
  size_t windowSize {5u};

  //! \brief Whether it is okay when there are no events between two update calls.
  bool noEventsIsOk {true};
};

/**
 * \brief Helper struct for easy brace-initialization of DurationStatusParam objects. On supported compilers, you can
 * also use designated braced initialization, i.e. `param = {.maxDuration = {10,0}}`. Supported is e.g. GCC 8+ in any
 * mode or any compiler in C++20 mode.
 */
struct SimpleDurationStatusParam
{
  //! \brief Minimum duration considered as valid.
  decltype(::cras::DurationStatusParam::minDuration) minDuration {0, 0};

  //! \brief Maximum duration considered as valid.
  decltype(::cras::DurationStatusParam::minDuration) maxDuration {::ros::DURATION_MAX};

  //! \brief Tolerance of the duration.
  decltype(::cras::DurationStatusParam::tolerance) tolerance {0.1};

  //! \brief Number of updates during which the duration is computed.
  decltype(::cras::DurationStatusParam::windowSize) windowSize {5u};

  //! \brief Whether it is okay when there are no events between two update calls.
  decltype(::cras::DurationStatusParam::noEventsIsOk) noEventsIsOk {true};
};


}
