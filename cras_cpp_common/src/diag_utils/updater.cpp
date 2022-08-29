/**
 * \file
 * \brief Diagnostic updater that automatically sets its Hardware ID to hostname of the machine.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <unistd.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/node_handle.h>

#include <cras_cpp_common/diag_utils/updater.h>

namespace cras
{

DiagnosticUpdater::DiagnosticUpdater(
  const ros::NodeHandle& h, const ros::NodeHandle& ph, const std::string& node_name) :
  diagnostic_updater::Updater(h, ph, node_name)
{
  std::string id {node_name};
  char hostname[1024];
  if (gethostname(hostname, 1023) == 0)
  {
    hostname[1023] = '\0';
    id = hostname;
  }
  this->setHardwareID(id);
}

}
