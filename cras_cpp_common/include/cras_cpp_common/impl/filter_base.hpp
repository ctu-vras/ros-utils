#pragma once

/**
 * \file
 * \brief This is non-interesting code supporting the integration of ParamHelper and GetParamAdapter for filters.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <stdexcept>
#include <string>

#include <ros/common.h>
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_base.hpp>
#else
#include <filters/filter_base.h>
#endif

#include <rosconsole/macros_generated.h>
#include <XmlRpcValue.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/filter_base.hpp>
#include <cras_cpp_common/param_utils/get_param_adapter.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_*_NAMED macros.
 * 
 * \tparam F Type of the filtered data.
 */
template <typename F>
class FilterLogHelper : public ::cras::LogHelper
{
public:
  /**
   * \brief Create the log helper.
   * \param filter The filter whose name will be used in the log prints.
   *               Make sure the object is not destroyed for the whole lifetime of this helper.
   */
  explicit FilterLogHelper(const ::filters::FilterBase<F>& filter) : filter(filter) {}
  FilterLogHelper(::filters::FilterBase<F>&& filter) = delete;  // don't allow temporaries
  ~FilterLogHelper() = default;

protected:
  void printDebug(const ::std::string& text) const override { ROS_DEBUG_NAMED(filter.getName(), "%s", text.c_str()); }
  void printInfo(const ::std::string& text) const override { ROS_INFO_NAMED(filter.getName(), "%s", text.c_str()); }
  void printWarn(const ::std::string& text) const override { ROS_WARN_NAMED(filter.getName(), "%s", text.c_str()); }
  void printError(const ::std::string& text) const override { ROS_ERROR_NAMED(filter.getName(), "%s", text.c_str()); }
  void printFatal(const ::std::string& text) const override { ROS_FATAL_NAMED(filter.getName(), "%s", text.c_str()); }
  
  //! The filter whose name is used in the log prints.
  const ::filters::FilterBase<F>& filter;
};

/**
 * \brief Adapter that extracts ROS parameters from a configured filter.
 * \tparam F Type of the filtered data.
 */
template <typename F>
struct FilterGetParamAdapter : public ::cras::GetParamAdapter
{
  /**
   * \brief Construct the parameter adapter.
   * \param filter The filter from which the parameters will be extracted.
   *               Make sure the object is not destroyed for the whole lifetime of this adapter.
   */
  explicit FilterGetParamAdapter(const ::cras::FilterBase<F>& filter) noexcept : filter(filter) {}
  FilterGetParamAdapter(::cras::FilterBase<F>&& filter) = delete;  // don't allow temporaries
  virtual ~FilterGetParamAdapter() = default;

  bool getParam(const ::std::string& name, ::XmlRpc::XmlRpcValue& v) const noexcept override
  {
    return this->filter.::filters::template FilterBase<F>::getParam(name, (::XmlRpc::XmlRpcValue&)v);
  }

  bool hasParam(const ::std::string& name) const noexcept override
  {
    return this->filter.params_.find(name) != this->filter.params_.end();
  }

  ::std::string getNamespace() const noexcept override
  {
    return this->filter.getName();
  }

  ::std::shared_ptr<::cras::GetParamAdapter> getNamespaced(const ::std::string& ns) const noexcept(false) override
  {
    const auto newNs = this->getNamespace() + "/" + ns;
    if (!this->hasParam(ns))
      throw ::std::runtime_error("Cannot find namespace " + newNs);
    ::XmlRpc::XmlRpcValue values;
    if (!this->getParam(ns, values))
      throw ::std::runtime_error("Parameter namespace " + newNs + " is invalid");
    return ::std::make_shared<::cras::XmlRpcValueGetParamAdapter>(values, newNs);
  }

protected:
  //! \brief The filter whose parameters are provided by this adapter.
  const ::cras::FilterBase<F>& filter;
};


}
