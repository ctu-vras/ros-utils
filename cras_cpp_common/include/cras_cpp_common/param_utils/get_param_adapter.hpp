#pragma once

/**
 * \file
 * \brief An adapter that allows getting ROS parameters from various sources.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <XmlRpcValue.h>

namespace cras
{

/**
 * \brief An adapter that allows getting ROS parameters from various sources.
 */
struct GetParamAdapter
{
  virtual ~GetParamAdapter() = default;

  /**
   * \brief Get the value of a paramater as XmlRpcValue.
   * \param[in] name Name of the parameter.
   * \param[out] value Value of the parameter (unchanged if getting the parameter failed).
   * \return Whether getting the parameter succeeded.
   */
  virtual bool getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const noexcept = 0;

  /**
   * \brief Return whether this adapter is able to get a value of the given parameter.
   * \param[in] name Name of the parameter. 
   * \return Whether this adapter is able to get a value of the given parameter.
   */
  virtual bool hasParam(const std::string& name) const noexcept = 0;
  
  /**
   * \brief Get the namespace of this adapter. It should somehow represent the source of the parameters.
   * \return The namespace.
   */
  virtual std::string getNamespace() const noexcept = 0;
  
  /**
   * \brief Construct another adapter whose source are parameters that are in a sub-namespce of this adapter.
   * \param[in] ns The namespace to search parameters in. 
   * \return The constructed adapter. The function should never return null (throw an exception on failure).
   * \throws std::runtime_error If something fails.
   * \throws ros::InvalidNameException If the namespace specification does not follow ROS parameter name rules.
   */
  virtual std::shared_ptr<GetParamAdapter> getNamespaced(const std::string& ns) const noexcept(false) = 0;
};

typedef ::std::shared_ptr<::cras::GetParamAdapter> GetParamAdapterPtr;
}