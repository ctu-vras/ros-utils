// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague
// SPDX-FileCopyrightText: 2009, Willow Garage, Inc.

/**
 * \file
 * \brief Node that can run both with and without a ROS master.
 * \author Martin Pecka
 */

// This file contains parts of BSD-licensed project https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp

#include <csignal>
#include <memory>
#include <string>

#include <boost/lexical_cast.hpp>
#include <cras_cpp_common/functional.hpp>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/node_utils/node_with_optional_master.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace ros
{
namespace this_node
{
extern void init(const std::string& names, const M_string& remappings, uint32_t options);
}
namespace file_log
{
extern void init(const M_string& remappings);
}
namespace master
{
extern void init(const M_string& remappings);
}
namespace network
{
extern void init(const M_string& remappings);
}

extern void check_ipv6_environment();
}

namespace cras
{

struct NodeWithOptionalMasterPrivate
{
  ros::M_string remappings;
  bool initialized {false};
  bool ok {false};
  bool usesMaster {false};

  std::shared_ptr<ros::NodeHandle> nh {};
};

NodeWithOptionalMaster::NodeWithOptionalMaster(const cras::LogHelperPtr& log) :
  cras::HasLogger(log), data(new NodeWithOptionalMasterPrivate{})
{
}

NodeWithOptionalMaster::~NodeWithOptionalMaster()
{
  this->shutdown();
}

namespace
{
bool g_ok {false};
void shutdownHandler()
{
  g_ok = false;
}
void atexitHandler() { shutdownHandler(); }
void signalHandler(const int signal) { shutdownHandler(); }
}

void NodeWithOptionalMaster::init(const ros::M_string& remappings, const std::string& name, const uint32_t options)
{
  ROSCONSOLE_AUTOINIT;
  // Disable SIGPIPE
#ifndef WIN32
  signal(SIGPIPE, SIG_IGN);
#else
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 0), &wsaData);
#endif
  ros::check_ipv6_environment();
  ros::network::init(remappings);
  ros::master::init(remappings);
  ros::this_node::init(name, remappings, options);
  ros::file_log::init(remappings);

  // Test if roscore can be reached
  if (ros::master::check())
  {
    CRAS_DEBUG("Running connected to ROS master.");
    this->data->usesMaster = true;

    ros::init(remappings, name, options);

    this->data->ok = true;
    g_ok = true;
    this->data->nh = std::make_unique<ros::NodeHandle>();
  }
  else
  {
    CRAS_INFO("Running without ROS master.");
    this->data->usesMaster = false;
    this->data->remappings = remappings;

    if (!(options & ros::init_options::NoSigintHandler))
      signal(SIGINT, &signalHandler);

    atexit(atexitHandler);

    this->data->ok = true;
    g_ok = true;

    ros::Time::init();
  }
  this->data->initialized = true;
}

void NodeWithOptionalMaster::init(int& argc, char** argv, const std::string& name, const uint32_t options)
{
  ros::M_string remappings;

  int full_argc = argc;
  // now, move the remapping argv's to the end, and decrement argc as needed
  for (int i = 0; i < argc; )
  {
    std::string arg = argv[i];
    size_t pos = arg.find(":=");
    if (pos != std::string::npos)
    {
      std::string local_name = arg.substr(0, pos);
      std::string external_name = arg.substr(pos + 2);

      CRAS_DEBUG("remap: %s => %s", local_name.c_str(), external_name.c_str());
      remappings[local_name] = external_name;

      // shuffle everybody down and stuff this guy at the end of argv
      char *tmp = argv[i];
      for (int j = i; j < full_argc - 1; j++)
        argv[j] = argv[j+1];
      argv[argc-1] = tmp;
      argc--;
    }
    else
    {
      i++;  // move on, since we didn't shuffle anybody here to replace it
    }
  }

  this->init(remappings, name, options);
}

void NodeWithOptionalMaster::init(const ros::VP_string& remappings, const std::string& name, const uint32_t options)
{
  ros::M_string remappings_map;
  for (const auto& [name, val] : remappings)
    remappings_map[name] = val;

  this->init(remappings_map, name, options);
}

void NodeWithOptionalMaster::shutdown()
{
  if (!this->data->initialized)
    return;

  this->data->ok = false;
  g_ok = false;
  this->data->initialized = false;

  if (this->usesMaster())
  {
    this->data->nh.reset();
  }
  else
  {
    ros::Time::shutdown();
    ros::console::shutdown();
  }
}

bool NodeWithOptionalMaster::isInitialized() const
{
  return this->data->initialized;
}

bool NodeWithOptionalMaster::usesMaster() const
{
  return this->data->usesMaster;
}

bool NodeWithOptionalMaster::ok() const
{
  return g_ok && (this->usesMaster() ? ros::ok() : this->data->ok);
}

std::string NodeWithOptionalMaster::resolveName(const std::string& name, const bool remap) const
{
  if (this->usesMaster())
    return this->data->nh->resolveName(name, remap);

  if (name.empty())
    return ros::this_node::getNamespace();

  std::string final = name;

  if (final[0] == '~')
  {
    std::stringstream ss;
    ss << "Using ~ names with NodeHandle methods is not allowed.  If you want to use private names with the NodeHandle";
    ss << " interface, construct a NodeHandle using a private name as its namespace.  e.g. ";
    ss << "ros::NodeHandle nh(\"~\");  ";
    ss << "nh.getParam(\"my_private_name\");";
    ss << " (name = [" << name << "])";
    throw ros::InvalidNameException(ss.str());
  }
  else if (final[0] == '/')
  {
    // do nothing
  }
  else if (!ros::this_node::getNamespace().empty())
  {
    final = ros::names::append(ros::this_node::getNamespace(), final);
  }

  final = ros::names::clean(final);

  if (remap)
    final = ros::names::remap(this->resolveName(final, false));

  return ros::names::resolve(final, false);
}

cras::BoundParamHelperPtr NodeWithOptionalMaster::getPrivateParams() const
{
  cras::GetParamAdapterPtr paramAdapter;

  if (this->usesMaster())
  {
    paramAdapter = std::make_shared<cras::NodeHandleGetParamAdapter>(ros::NodeHandle("~"));
  }
  else
  {
    XmlRpc::XmlRpcValue params;
    params.begin();

    for (const auto& [name, param] : this->data->remappings)
    {
      if (name.size() < 2)
        continue;

      if (name[0] == '_' && name[1] != '_')
      {
        const std::string local_name = name.substr(1);

        bool success = false;
        // We intentionally do not resolve the name because we anyway only read parameters from the current namespace.
        const auto& resolvedName = local_name;

        try
        {
          auto i = boost::lexical_cast<int32_t>(param);
          params[resolvedName] = i;
          success = true;
        }
        catch (const boost::bad_lexical_cast&)
        {
        }

        if (success)
          continue;

        try
        {
          auto d = boost::lexical_cast<double>(param);
          params[resolvedName] = d;
          success = true;
        }
        catch (const boost::bad_lexical_cast&)
        {
        }

        if (success)
          continue;

        if (param == "true" || param == "True" || param == "TRUE")
        {
          params[resolvedName] = true;
        }
        else if (param == "false" || param == "False" || param == "FALSE")
        {
          params[resolvedName] = false;
        }
        else
        {
          params[resolvedName] = param;
        }
      }
    }

    paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(params, ros::this_node::getName());
  }

  return std::make_shared<cras::BoundParamHelper>(this->log, paramAdapter);
}

}
