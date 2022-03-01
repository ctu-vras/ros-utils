#pragma once

/**
 * \file
 * \brief Helpers for working with filters based on filters::FilterBase.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <memory>

#include <ros/common.h>
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_base.hpp>
#else
#include <filters/filter_base.h>
#endif

#include <nodelet/nodelet.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>

namespace cras
{

// these are declared in the impl file
template <typename F> class FilterLogHelper;
template <typename F> struct FilterGetParamAdapter;

/**
 * This FilterBase implementation adds access to the templated getParam() methods from param_utils. It also adds
 * possibility to inform the filter about the nodelet it is running in (if it is running in one).
 * @tparam F Type of the filtered data.
 */
template<typename F>
class FilterBase : public ::filters::FilterBase<F>, public ::cras::BoundParamHelper
{

public:
  /**
   * \brief Construct the filter and pass the corresponding Log and GetParam helpers.
   */
  FilterBase() : ::cras::BoundParamHelper(
    ::std::make_shared<::cras::FilterLogHelper<F>>(*this),
    ::std::make_shared<::cras::FilterGetParamAdapter<F>>(*this))
  {
  }

  ~FilterBase() override = default;

  /**
   * \brief Inform this filter that it is running inside the passed nodelet. This should be called after configure().
   * @param nodelet The nodelet running this filter.
   */
  void setNodelet(const ::nodelet::Nodelet* nodelet)
  {
    this->nodelet = nodelet;
  }

  // Allow FilterGetParamAdapter to call getParam() methods.
  friend struct ::cras::FilterGetParamAdapter<F>;

protected:
  
  // Use getParam() implementation provided by BoundParamHelper.
  using ::cras::BoundParamHelper::getParam;

  //! \brief The nodelet this filter is running in. It should be set via setNodelet() after the filter is configured.
  const ::nodelet::Nodelet* nodelet {nullptr};
};

}

#include "impl/filter_base.hpp"