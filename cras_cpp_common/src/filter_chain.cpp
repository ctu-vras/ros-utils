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

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>

namespace cras
{

template <typename F>
::std::vector<::boost::shared_ptr<::filters::FilterBase<F>>>& FilterChain<F>::getFilters()
{
  return this->reference_pointers_;
}

template <typename F>
const ::std::vector<::boost::shared_ptr<::filters::FilterBase<F>>>& FilterChain<F>::getFilters() const
{
  return this->reference_pointers_;
}

template class FilterChain<int>;
template class FilterChain<float>;
template class FilterChain<double>;
template class FilterChain<std::string>;
template class FilterChain<sensor_msgs::CompressedImage>;
template class FilterChain<sensor_msgs::Image>;
template class FilterChain<sensor_msgs::Imu>;
template class FilterChain<sensor_msgs::Joy>;
template class FilterChain<sensor_msgs::JoyFeedback>;
template class FilterChain<sensor_msgs::LaserScan>;
template class FilterChain<sensor_msgs::MagneticField>;
template class FilterChain<sensor_msgs::MultiEchoLaserScan>;
template class FilterChain<sensor_msgs::NavSatFix>;
template class FilterChain<sensor_msgs::PointCloud>;
template class FilterChain<sensor_msgs::PointCloud2>;
template class FilterChain<sensor_msgs::Range>;
template class FilterChain<sensor_msgs::RelativeHumidity>;
template class FilterChain<sensor_msgs::Temperature>;

}