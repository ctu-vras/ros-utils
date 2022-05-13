#pragma once

/**
 * \file
 * \brief Utils for getting node handle parameters, topic diagnostics etc.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <ros/datatypes.h>
#include <ros/node_handle.h>

#include <cras_cpp_common/node_utils/node_handle_with_diagnostics.h>

namespace cras
{

class NodeHandle : public ::cras::NodeHandleWithDiagnostics
{
public:
  explicit NodeHandle(const ::std::string& ns = "", const ::ros::M_string& remappings = {}) :
    ::ros::NodeHandle(ns, remappings), ::cras::NodeHandleWithDiagnostics(ns, remappings)
  {
  }

  NodeHandle(const ::ros::NodeHandle& parent, const ::std::string& ns) :
    ::ros::NodeHandle(parent, ns), ::cras::NodeHandleWithDiagnostics(parent, ns)
  {
  }

  NodeHandle(const ::ros::NodeHandle& parent, const ::std::string& ns, const ::ros::M_string& remappings) :
    ::ros::NodeHandle(parent, ns, remappings), ::cras::NodeHandleWithDiagnostics(parent, ns, remappings)
  {
  }
};

}
