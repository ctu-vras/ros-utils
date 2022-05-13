#pragma once

/**
 * \file
 * \brief Diagnostic updater that automatically sets its Hardware ID to hostname of the machine.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>

namespace cras
{

/** 
 * \brief Diagnostic updater that automatically sets its Hardware ID to hostname of the machine.
 */
class DiagnosticUpdater : public ::diagnostic_updater::Updater
{
public:
  /**
   * \brief Create the updater.
   * \param[in] h Node handle to use for publishing to "/diagnostics" topic.
   * \param[in] ph Node handle from which "~diagnostic_period" parameter is read.
   * \param[in] node_name This name is prepended to all status messages.
   */
  explicit DiagnosticUpdater(
    const ::ros::NodeHandle& h = ::ros::NodeHandle(),
    const ::ros::NodeHandle& ph = ::ros::NodeHandle("~"),
    const ::std::string& node_name = ::ros::this_node::getName());
};

}
