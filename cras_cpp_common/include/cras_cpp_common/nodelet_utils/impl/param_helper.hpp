/**
 * \file
 * \brief Utils for getting nodelet parameters (private implementation details, do not include this directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include "../param_helper.hpp"

#include <functional>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <cras_cpp_common/log_utils/nodelet.h>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>

namespace cras
{

template <typename NodeletType>
NodeletParamHelper<NodeletType>::NodeletParamHelper() : ParamHelper(::std::make_shared<NodeletLogHelper>(
  ::std::bind(&NodeletParamHelper<NodeletType>::getName, this)))
{
}

template <typename NodeletType>
NodeletParamHelper<NodeletType>::~NodeletParamHelper() = default;

template <typename NodeletType>
::cras::BoundParamHelperPtr NodeletParamHelper<NodeletType>::params(
  const ::ros::NodeHandle& node, const ::std::string& ns) const
{
  const auto param = ::std::make_shared<::cras::NodeHandleGetParamAdapter>(node);
  auto result = ::std::make_shared<::cras::BoundParamHelper>(this->log, param);
  if (!ns.empty())
    result = result->paramsInNamespace(ns);
  return result;
}

template <typename NodeletType>
::cras::BoundParamHelperPtr NodeletParamHelper<NodeletType>::paramsForNodeHandle(const ::ros::NodeHandle& node) const
{
  return this->params(node);
}

template <typename NodeletType>
::cras::BoundParamHelperPtr NodeletParamHelper<NodeletType>::privateParams(const ::std::string& ns) const
{
  return this->params(NodeletType::getPrivateNodeHandle(), ns);
}

template <typename NodeletType>
::cras::BoundParamHelperPtr NodeletParamHelper<NodeletType>::publicParams(const ::std::string& ns) const
{
  return this->params(NodeletType::getNodeHandle(), ns);
}

}