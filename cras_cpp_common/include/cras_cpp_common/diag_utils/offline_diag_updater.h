// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Offline diagnostics updater.
 * \author Martin Pecka
 */

#pragma once

#include <string>

#include <cras_cpp_common/optional.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace cras
{

class OfflineDiagUpdater : public ::diagnostic_updater::DiagnosticTaskVector
{
public:
  explicit OfflineDiagUpdater(const ::std::string& hwid);
  virtual ~OfflineDiagUpdater();

  virtual ::cras::optional<::diagnostic_msgs::DiagnosticArray> update(const ::ros::Time& now);

protected:
  ::ros::Time next_time_;
  ::ros::Duration period_ {1, 0};
  ::std::string hwid_;
};

}
