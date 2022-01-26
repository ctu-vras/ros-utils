/**
 * \file
 * \brief Filter chain implementation. This file should cease to exist when reference_pointers_ is made protected in
 *        Melodic and Noetic.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// TODO https://github.com/ros/filters/pull/60 gives the protected access
#include <sstream>  // has to be there, otherwise we encounter build problems
#include <ros/common.h>
#define private protected
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_chain.h>
#endif
#undef private

#include <cras_cpp_common/filter_chain.hpp>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace cras
{

template <typename F>
const ::std::vector<::boost::shared_ptr<::filters::FilterBase<F>>>& FilterChain<F>::getFilters() const
{
  return this->reference_pointers_;
}

template class FilterChain<sensor_msgs::LaserScan>;
template class FilterChain<sensor_msgs::PointCloud2>;

}