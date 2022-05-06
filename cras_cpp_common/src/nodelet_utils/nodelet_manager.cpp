/**
 * \file
 * \brief A customizable nodelet manager.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>

#include <cras_cpp_common/nodelet_utils/loader_ros.h>
#include <cras_cpp_common/nodelet_utils/nodelet_manager.h>

namespace cras
{

NodeletManager::NodeletManager(::ros::NodeHandle nh)
{
	this->nh = nh;
}

void NodeletManager::init()
{
  this->classLoader = std::make_unique<ClassLoader>("nodelet", "nodelet::Nodelet");
  this->classLoader->refreshDeclaredClasses();

  auto createInstance = boost::bind(&NodeletManager::createInstance, this, boost::placeholders::_1);
  this->loader = std::make_unique<nodelet::Loader>(createInstance);
  this->loaderRos = std::make_unique<cras::LoaderROS>(this->loader.get(), this->nh);
}

boost::shared_ptr<nodelet::Nodelet> NodeletManager::createInstance(const std::string& lookupName)
{
  return this->classLoader->createInstance(lookupName);
}

}

