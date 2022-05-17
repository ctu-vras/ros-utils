/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This class is a copy-paste of LoaderROS from
// https://github.com/ros/nodelet_core/blob/indigo-devel/nodelet/src/loader.cpp which is not exported.
// It was only split to .h and .cpp file and imports were optimized. Also the namespace has been changed from
// nodelet:: to cras:: .

#include <string>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include <bondcpp/bond.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/nodelet_utils/loader_ros.h>

namespace cras
{

LoaderROS::LoaderROS(nodelet::Loader* parent, const ros::NodeHandle& nh) :
  parent_(parent), nh_(nh), bond_spinner_(1, &bond_callback_queue_)
{
  load_server_ = nh_.advertiseService("load_nodelet", &LoaderROS::serviceLoad, this);
  unload_server_ = nh_.advertiseService("unload_nodelet", &LoaderROS::serviceUnload, this);
  list_server_ = nh_.advertiseService("list", &LoaderROS::serviceList, this);

  bond_spinner_.start();
}

bool LoaderROS::serviceLoad(nodelet::NodeletLoad::Request& req, nodelet::NodeletLoad::Response& res)
{
  boost::mutex::scoped_lock lock(lock_);

  // build map
  nodelet::M_string remappings;
  if (req.remap_source_args.size() != req.remap_target_args.size())
  {
    ROS_ERROR("Bad remappings provided, target and source of different length");
  }
  else
  {
    for (size_t i = 0; i < req.remap_source_args.size(); ++i)
    {
      remappings[ros::names::resolve(req.remap_source_args[i])] = ros::names::resolve(req.remap_target_args[i]);
      ROS_DEBUG("%s:%s\n", ros::names::resolve(req.remap_source_args[i]).c_str(),
        remappings[ros::names::resolve(req.remap_source_args[i])].c_str());
    }
  }

  res.success = parent_->load(req.name, req.type, remappings, req.my_argv);

  // If requested, create bond to sister process
  if (res.success && !req.bond_id.empty())
  {
    bond::Bond* bond = new bond::Bond(nh_.getNamespace() + "/bond", req.bond_id);
    bond_map_.insert(req.name, bond);
    bond->setCallbackQueue(&bond_callback_queue_);
    bond->setBrokenCallback(boost::bind(&LoaderROS::unload, this, req.name));
    bond->start();
  }
  return res.success;
}

bool LoaderROS::serviceUnload(nodelet::NodeletUnload::Request& req, nodelet::NodeletUnload::Response& res)
{
  res.success = unload(req.name);
  return res.success;
}

bool LoaderROS::unload(const std::string& name)
{
  boost::mutex::scoped_lock lock(lock_);

  bool success = parent_->unload(name);
  if (!success)
  {
    ROS_ERROR("Failed to find nodelet with name '%s' to unload.", name.c_str());
    return success;
  }

  // break the bond, if there is one
  M_stringToBond::iterator it = bond_map_.find(name);
  if (it != bond_map_.end())
  {
    // disable callback for broken bond, as we are breaking it intentionally now
    it->second->setBrokenCallback(boost::function<void(void)>());
    // erase (and break) bond
    bond_map_.erase(name);
  }

  return success;
}

bool LoaderROS::serviceList(nodelet::NodeletList::Request&, nodelet::NodeletList::Response& res)
{
  res.nodelets = parent_->listLoadedNodelets();
  return true;
}

}
