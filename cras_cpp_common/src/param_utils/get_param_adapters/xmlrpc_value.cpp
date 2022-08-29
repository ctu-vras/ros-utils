/**
 * \file
 * \brief An adapter that allows getting ROS parameters from a XmlRpcValue struct.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <ros/names.h>
#include <XmlRpcValue.h>

#include <cras_cpp_common/param_utils/get_param_adapter.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/xmlrpc_value_traits.hpp>

namespace cras
{

XmlRpcValueGetParamAdapter::XmlRpcValueGetParamAdapter(
  const ::XmlRpc::XmlRpcValue& baseParam, const ::std::string& baseNamespace)
  : baseParam(baseParam), baseNamespace(baseNamespace)
{
  if (baseParam.getType() != ::XmlRpc::XmlRpcValue::TypeStruct)
    throw ::std::runtime_error(::cras::format(
      "XmlRpcGetParamAdapter requires a struct base parameter, but %s was given.",
      ::cras::to_cstring(baseParam.getType())));
}

XmlRpcValueGetParamAdapter::~XmlRpcValueGetParamAdapter() = default;

bool XmlRpcValueGetParamAdapter::getParam(const ::std::string& name, ::XmlRpc::XmlRpcValue& v) const noexcept
{
  if (!this->baseParam.hasMember(name))
    return false;
  v = this->baseParam[name];
  return true;
}

bool XmlRpcValueGetParamAdapter::hasParam(const ::std::string& name) const noexcept
{
  return this->baseParam.hasMember(name);
}

::std::string XmlRpcValueGetParamAdapter::getNamespace() const noexcept
{
  return this->baseNamespace;
}

::std::shared_ptr<::cras::GetParamAdapter>
XmlRpcValueGetParamAdapter::getNamespaced(const ::std::string &ns) const noexcept(false)
{
  const auto newNs = ros::names::append(this->getNamespace(), ns);
  if (!this->hasParam(ns))
    throw ::std::runtime_error("Cannot find namespace " + newNs);
  ::XmlRpc::XmlRpcValue values;
  if (!this->getParam(ns, values))
    throw ::std::runtime_error("Parameter namespace " + newNs + " is invalid");
  return ::std::make_shared<::cras::XmlRpcValueGetParamAdapter>(values, newNs);
}

}
