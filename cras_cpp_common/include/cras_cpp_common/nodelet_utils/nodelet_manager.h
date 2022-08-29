#pragma once

/**
 * \file
 * \brief A customizable nodelet manager.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <boost/shared_ptr.hpp>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>

#include <cras_cpp_common/nodelet_utils/loader_ros.h>

namespace cras
{

/**
 * \brief Nodelet manager with customizable instance creation mechanism.
 *
 * The standard usage of this class is creating its instance, calling init() and then ros::spin().
 */
class NodeletManager
{
public:
  /**
   * \brief Create the nodelet manager.
   * \param[in] nh The nodehandle used for nodelet load/unload ROS services.
   */
  explicit NodeletManager(::ros::NodeHandle nh = {"~"});

  /**
   * \brief Initialize all class members.
   */
  virtual void init();

protected:
  /**
   * \brief Create an instance of the given type.
   * \param[in] lookupName The type to create instance of.
   * \return The instance (or nullptr if creation failed).
   */
  virtual ::boost::shared_ptr<::nodelet::Nodelet> createInstance(const ::std::string& lookupName);

  //! \brief Type of the class loader used for loading nodelets.
  typedef ::pluginlib::ClassLoader<::nodelet::Nodelet> ClassLoader;

  //! \brief Class loader used for loading nodelets.
  ::std::unique_ptr<ClassLoader> classLoader;

  // loader has to be declared after classLoader, otherwise we get class_loader SEVERE_WARNING
  // about leaving managed instances in memory
  //! \brief Nodelet loader that loads and runs the nodelets.
  ::std::unique_ptr<::nodelet::Loader> loader;

  //! \brief ROS API for the nodelet manager.
  ::std::unique_ptr<::cras::LoaderROS> loaderRos;

  //! \brief Node handle for the ROS API services.
  ::ros::NodeHandle nh;
};

}
