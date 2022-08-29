/**
 * \file
 * \brief Utils for getting node parameters.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <ros/datatypes.h>
#include <ros/node_handle.h>

#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/node_utils/param_helper.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>

namespace cras
{

NodeParamHelper::NodeParamHelper(const std::string& ns, const ros::M_string& remappings) :
  ros::NodeHandle(ns, remappings),
  cras::BoundParamHelper(
    std::make_shared<cras::NodeLogHelper>(),
    std::make_shared<cras::NodeHandleGetParamAdapter>(ros::NodeHandle(ns, remappings)))
{
}

NodeParamHelper::NodeParamHelper(const ros::NodeHandle& parent, const std::string& ns) :
  ros::NodeHandle(parent, ns),
  cras::BoundParamHelper(
    std::make_shared<cras::NodeLogHelper>(),
    std::make_shared<cras::NodeHandleGetParamAdapter>(ros::NodeHandle(parent, ns)))
{
}

NodeParamHelper::NodeParamHelper(const ros::NodeHandle& parent, const std::string& ns,
  const ros::M_string& remappings) :
    ros::NodeHandle(parent, ns, remappings),
    cras::BoundParamHelper(
      std::make_shared<cras::NodeLogHelper>(),
      std::make_shared<cras::NodeHandleGetParamAdapter>(ros::NodeHandle(parent, ns, remappings)))
{
}

}