/**
 * \file
 * \brief Utils for adding diagnostics to a topic via node handle.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cras_cpp_common/node_utils/node_handle_with_diagnostics.h>

#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>

#include <cras_cpp_common/node_utils/param_helper.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>

using namespace cras;

NodeHandleWithDiagnostics::NodeHandleWithDiagnostics(const std::string& ns, const ros::M_string& remappings) :
  ros::NodeHandle(ns, remappings), NodeParamHelper(ns, remappings), parentNh(ros::NodeHandle())
{
}

NodeHandleWithDiagnostics::NodeHandleWithDiagnostics(const ros::NodeHandle& parent, const std::string& ns) :
  ros::NodeHandle(parent, ns), NodeParamHelper(parent, ns), parentNh(parent)
{
}

NodeHandleWithDiagnostics::NodeHandleWithDiagnostics(const ros::NodeHandle& parent, const std::string& ns,
  const ros::M_string& remappings) :
    ros::NodeHandle(parent, ns, remappings), NodeParamHelper(parent, ns, remappings), parentNh(parent)
{
}

std::string NodeHandleWithDiagnostics::prefixDiagNamespace(const std::string& ns) const
{
  if (this->getNamespace() == ros::this_node::getNamespace() && !ns.empty() && ns[0] != '/')
    return ros::names::append(ros::this_node::getName(), ns);
  return ns;
}

BoundParamHelperPtr NodeHandleWithDiagnostics::getDiagParams(const std::string& diagNs, const std::string& topic) const
{
  if (!diagNs.empty() && diagNs[0] == '~')
    return this->paramsInNamespace(this->prefixDiagNamespace(diagNs.substr(1)));
  if (!diagNs.empty())
    return this->paramsInNamespace(diagNs);
  if (topic.empty())
    throw std::runtime_error("Either diagNs or topic has to be nonempty.");

  // Use topic as the default namespace. We need to prevent remapping the topic in standard getParam() calls, and that
  // is done by using XmlRpcValueGetParamAdapter that is initialized for some parent namespace. Searching in this
  // adapter is not affected by remapping.
  std::shared_ptr<cras::GetParamAdapter> adapter;
  // If the topic is absolute, we want to take it as is, so we construct the XmlRpcValueGetParamAdapter for all
  // parameters. This may be a little costly, but there's probably no better way.
  if (topic[0] == '/')
  {
    XmlRpc::XmlRpcValue xml;
    xml.begin();  // morph xml into a struct
    xml = this->getParam("/", xml, "", {false});
    adapter = std::make_shared<XmlRpcValueGetParamAdapter>(xml, "/");
  }
  else
  {
    // Find a suitable parent namespace. Names are remapped until this namespace, and the rest (topic name) is not
    // remapped.
    const auto parentNs = (ros::this_node::getNamespace() == this->getNamespace()) ? ros::this_node::getName() : "";
    const auto resolvedParentNs = ros::names::resolve(this->resolveName(parentNs));
    XmlRpc::XmlRpcValue xml;
    xml.begin();  // morph xml into a struct
    xml = this->getParam(resolvedParentNs, xml, "", {false});
    adapter = std::make_shared<XmlRpcValueGetParamAdapter>(xml, resolvedParentNs);
  }
  
  // Construct the final param adapter for the topic namespace, falling back to an empty adapter if the namespace does
  // not exist.
  const auto cleanTopic = cras::stripLeadingSlash(topic);
  std::shared_ptr<cras::GetParamAdapter> topicAdapter;
  if (adapter->hasParam(cleanTopic))
  {
    topicAdapter = adapter->getNamespaced(cleanTopic);
  }
  else
  {
    XmlRpc::XmlRpcValue xml;
    xml.begin();  // morph xml into a struct
    auto topicNs = ros::names::append(adapter->getNamespace(), cleanTopic);
    topicAdapter = std::make_shared<XmlRpcValueGetParamAdapter>(xml, topicNs);
  }
  
  return std::make_shared<BoundParamHelper>(this->log, topicAdapter);
}
