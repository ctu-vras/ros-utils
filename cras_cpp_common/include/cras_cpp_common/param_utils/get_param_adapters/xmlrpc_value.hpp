#pragma once

/**
 * \file
 * \brief An adapter that allows getting ROS parameters from a XmlRpcValue struct.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <XmlRpcValue.h>

#include <cras_cpp_common/param_utils/get_param_adapter.hpp>

namespace cras
{

struct XmlRpcValueGetParamAdapter : public ::cras::GetParamAdapter
{
public:
  explicit XmlRpcValueGetParamAdapter(const ::XmlRpc::XmlRpcValue& baseParam, const ::std::string& baseNamespace);
  ~XmlRpcValueGetParamAdapter() override;

  bool getParam(const ::std::string& name, ::XmlRpc::XmlRpcValue& v) const noexcept override;
  bool hasParam(const ::std::string& name) const noexcept override;
  ::std::string getNamespace() const noexcept override;
  ::std::shared_ptr<::cras::GetParamAdapter> getNamespaced(const ::std::string &ns) const noexcept(false) override;

protected:
  ::XmlRpc::XmlRpcValue baseParam;
  ::std::string baseNamespace;
};

}