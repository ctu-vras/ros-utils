#pragma once

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
// nodelet:: to cras:: and, few cosmetic changes have been made and documentation has been added.

#include <string>

#include <boost/thread/mutex.hpp>
#include <boost/ptr_container/ptr_map.hpp>

#include <bondcpp/bond.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

namespace cras
{

/**
 * \brief ROS interface for loading/unloading nodelets (internally using nodelet::Loader to do the work).
 * 
 * Advertised services are:
 * - load_nodelet (nodelet/NodeletLoad): Load a nodelet.
 * - unload_nodelet (nodelet/NodeletUnload): Unload a nodelet.
 * - list (nodelet/NodeletList): List the names of all currently loaded nodelets.
 */
class LoaderROS
{
public:
	/**
	 * \brief Construct the ROS interface for the given loader.
	 * \param[in] parent The loader to use for the actual nodelet loading/unloading.
	 * \param[in] nh The node handle to use for the advertised services.
	 */
  LoaderROS(::nodelet::Loader* parent, const ::ros::NodeHandle& nh);

protected:
	/**
	 * \brief Load a nodelet.
	 * \param[in] req Parameters of the nodelet.
	 * \param[out] res Whether the loading succeeded.
	 * \return res.success.
	 */
  bool serviceLoad(::nodelet::NodeletLoad::Request& req, ::nodelet::NodeletLoad::Response& res);

	/**
	 * \brief Unload a nodelet.
	 * \param[in] req Parameters of the nodelet.
	 * \param[out] res Whether the unloading succeeded.
	 * \return res.success.
	 */
  bool serviceUnload(::nodelet::NodeletUnload::Request& req, ::nodelet::NodeletUnload::Response& res);

	/**
	 * \brief Unload a nodelet of the given name.
	 * \param[in] name Name of the nodelet to unload.
	 * \return Whether the unloading succeeded.
	 */
  bool unload(const ::std::string& name);

	/**
	 * \brief List all loaded nodelets.
	 * \param[in] req Unused.
	 * \param[out] res The loaded nodelet names.
	 * \return True.
	 */
  bool serviceList(::nodelet::NodeletList::Request& req, ::nodelet::NodeletList::Response& res);

	//! \brief The actual nodelet loader to use.
  ::nodelet::Loader* parent_;
	
	//! \brief The node handle to use for the advertised services.
  ::ros::NodeHandle nh_;
	
	//! \brief Service server for load_nodelet service.
  ::ros::ServiceServer load_server_;

	//! \brief Service server for unload_nodelet service.
  ::ros::ServiceServer unload_server_;

	//! \brief Service server for list service.
  ::ros::ServiceServer list_server_;

	//! \brief Lock protecting access to parent_ and bond_map_.
  ::boost::mutex lock_;

	//! \brief Callback queue used for the created bonds.
  ::ros::CallbackQueue bond_callback_queue_;
	
	//! \brief Spinner of bond_callback_queue_.
  ::ros::AsyncSpinner bond_spinner_;
	
	//! \brief Type of the map of nodelet name->bond.
  typedef ::boost::ptr_map<::std::string, ::bond::Bond> M_stringToBond;

	//! \brief Map of nodelet name->bond.
  M_stringToBond bond_map_;
};

}
