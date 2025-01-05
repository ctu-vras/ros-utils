// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Node that can run both with and without a ROS master.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <ros/datatypes.h>

namespace cras
{

struct NodeWithOptionalMasterPrivate;

/**
 * \brief Node that can run both with and without a ROS master. This can be useful for one-off scripts that do not need
 *        to publish or subscribe anything, but they want to make use of the parameter processing and name remapping.
 *
 * This class provides an interface similar to ros::NodeHandle. However, when init() is called, it first checks whether
 * the rosmaster is running and can be contacted. If not, this class falls back to a mode with no master. It will still
 * provide parameters and name resolving, however, only for the parameters and remaps directly specified when launching
 * the node. When running without master, simulation time is never used.
 */
class NodeWithOptionalMaster : public ::cras::HasLogger
{
public:
  /**
   * \param[in] log Logger.
   */
  explicit NodeWithOptionalMaster(const ::cras::LogHelperPtr& log);
  virtual ~NodeWithOptionalMaster();

  /**
   * \brief Initialize the node, autodetecting whether a ros master is running or not.
   * \param[in] remappings Remappings of parameters and topics.
   * \param[in] name Node name.
   * \param[in] options Node init options.
   */
  void init(const ::ros::M_string& remappings, const ::std::string& name, uint32_t options);

  /**
   * \brief Initialize the node, autodetecting whether a ros master is running or not.
   * \param[in] argc Nr. of CLI args.
   * \param[in] argv CLI args.
   * \param[in] name Node name.
   * \param[in] options Node init options.
   */
  void init(int& argc, char** argv, const ::std::string& name, uint32_t options);

  /**
   * \brief Initialize the node, autodetecting whether a ros master is running or not.
   * \param[in] remappings Remappings of parameters and topics.
   * \param[in] name Node name.
   * \param[in] options Node init options.
   */
  void init(const ::ros::VP_string& remappings, const ::std::string& name, uint32_t options);

  /**
   * \brief Shutdown the node.
   */
  void shutdown();

  /**
   * \brief Has init() been called?
   */
  bool isInitialized() const;

  /**
   * \brief Is a live ROS master being used?
   */
  bool usesMaster() const;

  /**
   * \brief Equivalent of ros::ok() .
   */
  bool ok() const;

  /**
   * \brief Resolve a name relative to the global namespace.
   * \param name The name to be resolved (cannot start with ~).
   * \param remap Whether remapping should be applied.
   * \return The resolved name.
   */
  ::std::string resolveName(const ::std::string& name, bool remap = true) const;

  /**
   * \brief Get private parameters of the node.
   * \return The parameters object.
   * \remark When running without master, only parameters directly specified in remappings will be available.
   */
  ::cras::BoundParamHelperPtr getPrivateParams() const;

private:
  ::std::unique_ptr<NodeWithOptionalMasterPrivate> data;  //!< PIMPL
};

}
