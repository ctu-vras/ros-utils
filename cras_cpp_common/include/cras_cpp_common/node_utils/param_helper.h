#pragma once

/**
 * \file
 * \brief Utils for getting node parameters.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <ros/datatypes.h>
#include <ros/node_handle.h>

#include <cras_cpp_common/param_utils/bound_param_helper.hpp>

namespace cras
{

/**
 * \brief This mixin allows calling the getParam() helpers.
 */
class NodeParamHelper : public virtual ::ros::NodeHandle, public ::cras::BoundParamHelper
{
public:
  explicit NodeParamHelper(const ::std::string& ns = "", const ::ros::M_string& remappings = {});
  
  NodeParamHelper(const ::ros::NodeHandle& parent, const ::std::string& ns);
  
  NodeParamHelper(const ::ros::NodeHandle& parent, const ::std::string& ns, const ::ros::M_string& remappings);

  using ::cras::BoundParamHelper::getParam;
  using ::cras::BoundParamHelper::hasParam;
  using ::cras::BoundParamHelper::getNamespace;
};

}
