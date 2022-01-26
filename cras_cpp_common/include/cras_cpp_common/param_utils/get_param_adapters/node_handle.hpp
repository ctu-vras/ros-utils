#pragma once

/**
 * \file
 * \brief An adapter that allows getting ROS parameters from a node handle.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <ros/node_handle.h>
#include <XmlRpcValue.h>

#include <cras_cpp_common/param_utils/get_param_adapter.hpp>

namespace cras
{

/**
 * \brief An adapter that allows getting ROS parameters from a node handle.
 */
struct NodeHandleGetParamAdapter : public ::cras::GetParamAdapter
{
public:
  explicit NodeHandleGetParamAdapter(const ::ros::NodeHandle &nh) noexcept;
  ~NodeHandleGetParamAdapter() override;

  bool getParam(const ::std::string& name, ::XmlRpc::XmlRpcValue& v) const noexcept override;
  ::std::string getNamespace() const noexcept override;
  bool hasParam(const ::std::string& name) const noexcept override;
  ::std::shared_ptr<::cras::GetParamAdapter> getNamespaced(const ::std::string &ns) const noexcept(false) override;

protected:
  ros::NodeHandle nh;
};

}