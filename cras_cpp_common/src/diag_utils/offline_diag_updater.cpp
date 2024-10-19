// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Offline diagnostics updater.
 * \author Martin Pecka
 */

#include <string>
#include <vector>

#include <cras_cpp_common/diag_utils/offline_diag_updater.h>
#include <cras_cpp_common/optional.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <ros/time.h>

namespace cras
{

OfflineDiagUpdater::OfflineDiagUpdater(const std::string& hwid): hwid_(hwid)
{
}

OfflineDiagUpdater::~OfflineDiagUpdater() = default;

cras::optional<diagnostic_msgs::DiagnosticArray> OfflineDiagUpdater::update(const ros::Time& now)
{
  if (this->next_time_.isZero())
  {
    this->next_time_ = now + this->period_;
    return cras::nullopt;
  }

  if (now < this->next_time_)
    return cras::nullopt;

  this->next_time_ = now + this->period_;

  std::vector<diagnostic_msgs::DiagnosticStatus> status_vec;
  for (const auto& task : this->getTasks())
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = task.getName();
    status.level = 2;
    status.message = "No message was set";
    status.hardware_id = this->hwid_;

    task.run(status);

    status_vec.push_back(status);
  }

  diagnostic_msgs::DiagnosticArray msg;
  msg.status = status_vec;
  msg.header.stamp = now;
  return msg;
}

}
