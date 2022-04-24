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

#include "cras_cpp_common/filter_utils/filter_chain.hpp"

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
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
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

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
template class FilterChain<geometry_msgs::Accel>;
template class FilterChain<geometry_msgs::AccelStamped>;
template class FilterChain<geometry_msgs::AccelWithCovariance>;
template class FilterChain<geometry_msgs::AccelWithCovarianceStamped>;
template class FilterChain<geometry_msgs::Inertia>;
template class FilterChain<geometry_msgs::InertiaStamped>;
template class FilterChain<geometry_msgs::Point>;
template class FilterChain<geometry_msgs::PointStamped>;
template class FilterChain<geometry_msgs::Point32>;
template class FilterChain<geometry_msgs::Polygon>;
template class FilterChain<geometry_msgs::PolygonStamped>;
template class FilterChain<geometry_msgs::PoseArray>;
template class FilterChain<geometry_msgs::Pose>;
template class FilterChain<geometry_msgs::PoseStamped>;
template class FilterChain<geometry_msgs::PoseWithCovariance>;
template class FilterChain<geometry_msgs::PoseWithCovarianceStamped>;
template class FilterChain<geometry_msgs::Pose2D>;
template class FilterChain<geometry_msgs::Quaternion>;
template class FilterChain<geometry_msgs::QuaternionStamped>;
template class FilterChain<geometry_msgs::Transform>;
template class FilterChain<geometry_msgs::TransformStamped>;
template class FilterChain<geometry_msgs::Twist>;
template class FilterChain<geometry_msgs::TwistStamped>;
template class FilterChain<geometry_msgs::TwistWithCovariance>;
template class FilterChain<geometry_msgs::TwistWithCovarianceStamped>;
template class FilterChain<geometry_msgs::Vector3>;
template class FilterChain<geometry_msgs::Vector3Stamped>;
template class FilterChain<geometry_msgs::Wrench>;
template class FilterChain<geometry_msgs::WrenchStamped>;
template class FilterChain<std_msgs::Duration>;
template class FilterChain<std_msgs::Float32>;
template class FilterChain<std_msgs::Float64>;
template class FilterChain<std_msgs::Int8>;
template class FilterChain<std_msgs::Int16>;
template class FilterChain<std_msgs::Int32>;
template class FilterChain<std_msgs::Int64>;
template class FilterChain<std_msgs::String>;
template class FilterChain<std_msgs::Time>;
template class FilterChain<std_msgs::UInt8>;
template class FilterChain<std_msgs::UInt16>;
template class FilterChain<std_msgs::UInt32>;
template class FilterChain<std_msgs::UInt64>;
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