#pragma once

/**
 * \file
 * \brief This file is for backwards compatibility only.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/rate.h>
#include <ros/time.h>

#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>

namespace cras
{

/**
 * \brief A structure that allows adding topic diagnostics to an updater.
 */
class [[deprecated("Use DiagnosedPublisher or DiagnosedSubscriber instead")]] TopicDiagnostic :
  public ::cras::DiagnosedPubSub<::diagnostic_msgs::DiagnosticArray>
{
public:
  TopicDiagnostic(const ::std::string&, ::diagnostic_updater::Updater& updater,
    const ::cras::BoundParamHelperPtr& params);

  TopicDiagnostic(const ::std::string&, ::diagnostic_updater::Updater& updater,
    const ::cras::BoundParamHelperPtr& params, const ::ros::Rate& defaultRate);

  TopicDiagnostic(const ::std::string&, ::diagnostic_updater::Updater& updater,
    const ::cras::BoundParamHelperPtr& params, const ::ros::Rate& defaultRate,
    const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate);

  /**
   * \brief Record a message published at the given time.
   * \param[in] stamp Timestamp of the message.
   */
  void tick(const ::ros::Time& stamp);

  /**
   * \brief Get the name of the diagnostic task.
   * \return Name of the diagnostic task.
   */
  ::std::string getName() const;
};



}
