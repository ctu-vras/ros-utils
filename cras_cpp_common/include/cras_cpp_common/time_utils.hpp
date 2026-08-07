// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 */

#pragma once

#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cras_cpp_common/expected.hpp>
#include <rcl/time.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>
#include <rmw/time.h>

namespace cras {

#if __cpp_lib_chrono >= 201907L || (defined(__GNUC__) && __GNUC__ >= 12 && __cplusplus >= 202002L)
#define cras_has_chrono_clocks_support 1
#endif

// *INDENT-OFF*

template<typename T> inline constexpr auto is_duration_v = false;
template<typename R, typename P> inline constexpr auto is_duration_v<::std::chrono::duration<R, P>> = true;

template<typename T> inline constexpr auto is_time_point_v = false;
template<typename C, typename D> inline constexpr auto is_time_point_v<::std::chrono::time_point<C, D>> = true;

/**
 * \brief Traits class for all std::chrono clock types that can be converted to system_time.
 */
template<typename, typename = void> struct ChronoTimeClock : ::std::false_type {};

#ifdef cras_has_chrono_clocks_support
// Add all clocks that can be cast to system_clock (which is not e.g. steady_clock).
template<typename C>
struct ChronoTimeClock<
    C,
    ::std::void_t<
      ::std::enable_if_t<::std::chrono::is_clock_v<C>>,
      decltype(::std::chrono::clock_cast<::std::chrono::system_clock>(::std::declval<typename C::time_point>()))
    >
  > : ::std::true_type {};
#else
template<> struct ChronoTimeClock<::std::chrono::system_clock> : ::std::true_type {};
#endif

/**
 * \brief Traits class for all std::chrono::time_point variants that can be converted to system_clock::time_point .
 */
template<typename, typename = void> struct ChronoTimeType : ::std::false_type {};

template<typename C, typename D>
struct ChronoTimeType<
  ::std::chrono::time_point<C, D>,
  ::std::enable_if_t<::cras::ChronoTimeClock<C>::value && ::cras::is_duration_v<D>>
> : ::std::true_type {};

/**
 * Type trait designating all types that can represent a time point.
 */
template<typename, typename = void> struct TimeType : ::std::false_type {};

template<> struct TimeType<::rclcpp::Time> : ::std::true_type {};
template<> struct TimeType<::builtin_interfaces::msg::Time> : ::std::true_type {};
template<> struct TimeType<rcl_time_point_t> : ::std::true_type {};
template<> struct TimeType<rcutils_time_point_value_t> : ::std::true_type {};
template<> struct TimeType<rmw_time_t> : ::std::true_type {};
template<> struct TimeType<double> : ::std::true_type {};
template<> struct TimeType<::tm> : ::std::true_type {};
template<typename C, typename D>
struct TimeType<
  ::std::chrono::time_point<C, D>,
  ::std::enable_if_t<::cras::ChronoTimeClock<C>::value && ::cras::is_duration_v<D>>
> : ::std::true_type {};
// *INDENT-ON*

/**
 * \brief Identity function for time types.
 * \tparam T The time type.
 * \param[in] t The input time.
 * \return t
 */
template<typename T, typename ::std::enable_if_t<::cras::TimeType<T>::value>* = nullptr>
T convertTime(const T& t) {
  return t;
}

/**
 * Convert between various time representations.
 *
 * This template is the fallback used in case two time types are given but their conversion is not yet implemented.
 *
 * \tparam T2 The desired type.
 * \tparam T1 The input type.
 * \param[in] t The time to convert.
 * \return The converted time.
 */
template<
    typename T2, typename T1,
    typename ::std::enable_if_t<
        ::cras::TimeType<T1>::value &&
        ::cras::TimeType<T2>::value &&
        !::std::is_same_v<T1, T2> &&
        !(::cras::is_time_point_v<T2> && ::cras::is_time_point_v<T1>)
    >* = nullptr
>
T2 convertTime(const T1& t) = delete;

// *INDENT-OFF*
template<> ::builtin_interfaces::msg::Time convertTime(const ::rclcpp::Time& t);
template<> rcl_time_point_t convertTime(const ::rclcpp::Time& t);
template<> rcutils_time_point_value_t convertTime(const ::rclcpp::Time& t);
template<> rmw_time_t convertTime(const ::rclcpp::Time& t);
template<> double convertTime(const ::rclcpp::Time& t);
template<> ::tm convertTime(const ::rclcpp::Time& t);
template<> ::std::chrono::system_clock::time_point convertTime(const ::rclcpp::Time& t);

template<> ::rclcpp::Time convertTime(const ::builtin_interfaces::msg::Time& t);
template<> rcl_time_point_t convertTime(const ::builtin_interfaces::msg::Time& t);
template<> rcutils_time_point_value_t convertTime(const ::builtin_interfaces::msg::Time& t);
template<> rmw_time_t convertTime(const ::builtin_interfaces::msg::Time& t);
template<> double convertTime(const ::builtin_interfaces::msg::Time& t);
template<> ::tm convertTime(const ::builtin_interfaces::msg::Time& t);
template<> ::std::chrono::system_clock::time_point convertTime(const ::builtin_interfaces::msg::Time& t);

template<> ::rclcpp::Time convertTime(const rcl_time_point_t& t);
template<> ::builtin_interfaces::msg::Time convertTime(const rcl_time_point_t& t);
template<> rcutils_time_point_value_t convertTime(const rcl_time_point_t& t);
template<> rmw_time_t convertTime(const rcl_time_point_t& t);
template<> double convertTime(const rcl_time_point_t& t);
template<> ::tm convertTime(const rcl_time_point_t& t);
template<> ::std::chrono::system_clock::time_point convertTime(const rcl_time_point_t& t);

template<> ::rclcpp::Time convertTime(const rcutils_time_point_value_t& t);
template<> ::builtin_interfaces::msg::Time convertTime(const rcutils_time_point_value_t& t);
template<> rcl_time_point_t convertTime(const rcutils_time_point_value_t& t);
template<> rmw_time_t convertTime(const rcutils_time_point_value_t& t);
template<> double convertTime(const rcutils_time_point_value_t& t);
template<> ::tm convertTime(const rcutils_time_point_value_t& t);
template<> ::std::chrono::system_clock::time_point convertTime(const rcutils_time_point_value_t& t);

template<> ::rclcpp::Time convertTime(const rmw_time_t& t);
template<> ::builtin_interfaces::msg::Time convertTime(const rmw_time_t& t);
template<> rcl_time_point_t convertTime(const rmw_time_t& t);
template<> rcutils_time_point_value_t convertTime(const rmw_time_t& t);
template<> double convertTime(const rmw_time_t& t);
template<> ::tm convertTime(const rmw_time_t& t);
template<> ::std::chrono::system_clock::time_point convertTime(const rmw_time_t& t);

template<> ::rclcpp::Time convertTime(const double& t);
template<> ::builtin_interfaces::msg::Time convertTime(const double& t);
template<> rcl_time_point_t convertTime(const double& t);
template<> rcutils_time_point_value_t convertTime(const double& t);
template<> rmw_time_t convertTime(const double& t);
template<> ::tm convertTime(const double& t);
template<> ::std::chrono::system_clock::time_point convertTime(const double& t);

::cras::expected<::rclcpp::Time, ::std::string> fromStructTm(const tm& time);
template<> ::rclcpp::Time convertTime(const ::tm& t);
template<> ::builtin_interfaces::msg::Time convertTime(const ::tm& t);
template<> rcl_time_point_t convertTime(const ::tm& t);
template<> rcutils_time_point_value_t convertTime(const ::tm& t);
template<> rmw_time_t convertTime(const ::tm& t);
template<> double convertTime(const ::tm& t);
template<> ::std::chrono::system_clock::time_point convertTime(const ::tm& t);

template<> ::rclcpp::Time convertTime(const ::std::chrono::system_clock::time_point& t);
template<> ::builtin_interfaces::msg::Time convertTime(const ::std::chrono::system_clock::time_point& t);
template<> rcl_time_point_t convertTime(const ::std::chrono::system_clock::time_point& t);
template<> rcutils_time_point_value_t convertTime(const ::std::chrono::system_clock::time_point& t);
template<> rmw_time_t convertTime(const ::std::chrono::system_clock::time_point& t);
template<> double convertTime(const ::std::chrono::system_clock::time_point& t);
template<> ::tm convertTime(const ::std::chrono::system_clock::time_point& t);

template<
  typename T, typename D,
  ::std::enable_if_t<
    ::cras::TimeType<T>::value &&
    !::std::is_same_v<D, ::std::chrono::nanoseconds> &&
    !::cras::is_time_point_v<T>
  >* = nullptr
>
T convertTime(const ::std::chrono::time_point<::std::chrono::system_clock, D>& t) {
  if constexpr (::std::is_same_v<T, ::std::chrono::time_point<::std::chrono::system_clock, D>>) {
    return t;
  }
  return ::cras::convertTime<T>(::std::chrono::time_point_cast<::std::chrono::nanoseconds>(t));
}

template<
  typename T2, typename T1,
  ::std::enable_if_t<
    ::cras::is_time_point_v<T2> && ::cras::is_time_point_v<T1> &&
    !::std::is_same_v<typename T2::duration, typename T1::duration>
  >* = nullptr
>
T2 convertTime(const T1& t) {
  return ::std::chrono::time_point_cast<typename T2::duration>(t);
}

#ifdef cras_has_chrono_clocks_support
template<
  typename T, typename C, typename D,
  ::std::enable_if_t<
    ::cras::TimeType<T>::value &&
    !::std::is_same_v<C, ::std::chrono::system_clock> &&
    !::cras::is_time_point_v<T>
  >* = nullptr
>
T convertTime(const ::std::chrono::time_point<C, D>& t) {
  return convertTime<T>(::std::chrono::clock_cast<::std::chrono::system_clock>(t));
}

template<
  typename T2, typename T1,
  ::std::enable_if_t<
    ::cras::is_time_point_v<T2> && ::cras::is_time_point_v<T1> &&
    !::std::is_same_v<typename T2::clock, typename T1::clock>
  >* = nullptr
>
T2 convertTime(const T1& t) {
  return ::std::chrono::clock_cast<typename T2::clock>(t);
}
#endif
// *INDENT-ON*

/**
 * Convert between various time representations.
 *
 * \tparam T1 The output type.
 * \param[in] t The time to convert.
 * \param[in] clock_type The clock type of the output.
 * \return The converted time with the given clock type.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
::rclcpp::Time convertTime(const T1& t, const ::rcl_clock_type_t clock_type) {
  return ::rclcpp::Time(convertTime<rcutils_time_point_value_t>(t), clock_type);
}

/**
 * Convert time from sec+nsec to nsec since time origin.
 *
 * \param sec The seconds part.
 * \param nsec The nanoseconds part.
 * \return The corresponding number of nanoseconds.
 */
int64_t secNsecToNSec(const int32_t sec, const uint32_t nsec);

/**
 * Split the given time to seconds and the fractional second converted to nanoseconds.
 *
 * \tparam T1 Time type.
 * \param t Time to convert.
 * \return A pair consisting of the whole number of seconds and the number of nanoseconds in the fractional second.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
std::pair<int32_t, uint32_t> secNsec(const T1& t) {
  const auto time = convertTime<::builtin_interfaces::msg::Time>(t);
  return {time.sec, time.nanosec};
}

/**
 * Get the whole number of seconds from the given time.
 *
 * \tparam T1 Time type.
 * \param t Time to convert.
 * \return The number of whole seconds.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
int32_t sec(const T1& t) {
  return ::cras::secNsec(t).first;
}

/**
 * Get the fractional second converted to nanoseconds.
 *
 * \tparam T1 Time type.
 * \param t Time to convert.
 * \return The number of nanoseconds in the fractional second.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
uint32_t nanosec(const T1& t) {
  return ::cras::secNsec(t).second;
}

/**
 * Get the time converted to float seconds.
 *
 * \tparam T1 Time type.
 * \param t Time to convert.
 * \return The float seconds.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
double float_secs(const T1& t) {
  return ::cras::convertTime<double>(t);
}

// *INDENT-OFF*
/**
 * Type trait designating all types that can represent a time duration.
 */
template<typename, typename = void> struct DurationType : ::std::false_type {};

template<> struct DurationType<::rclcpp::Duration> : ::std::true_type {};
template<> struct DurationType<::builtin_interfaces::msg::Duration> : ::std::true_type {};
template<> struct DurationType<rcl_duration_t> : ::std::true_type {};
template<> struct DurationType<rcutils_duration_value_t> : ::std::true_type {};
template<> struct DurationType<rmw_time_t> : ::std::true_type {};  // yes, this is correct, rmw_time_t is duration, too
template<> struct DurationType<double> : ::std::true_type {};
template<typename R, typename P> struct DurationType<
    ::std::chrono::duration<R, P>,
    ::std::enable_if_t<::cras::is_duration_v<::std::chrono::duration<R, P>>>
> : ::std::true_type {};
// *INDENT-ON*

/**
 * \brief Identity function for duration types.
 * \tparam T The duration type.
 * \param[in] t The input duration.
 * \return t
 */
template<typename T, typename ::std::enable_if_t<::cras::DurationType<T>::value>* = nullptr>
T convertDuration(const T& t) {
  return t;
}

/**
 * Convert between various duration representations.
 *
 * This template is the fallback used in case two duration types are given but their conversion is not yet implemented.
 *
 * \tparam D2 The desired type.
 * \tparam D1 The input type.
 * \param[in] t The duration to convert.
 * \return The converted duration.
 */
template<
    typename D2, typename D1,
    typename ::std::enable_if_t<
        ::cras::DurationType<D1>::value &&
        ::cras::DurationType<D2>::value &&
        !::std::is_same_v<D1, D2> &&
        // the following case needs to be explicitly removed from overload resolution
        !::cras::is_duration_v<D1>
    >* = nullptr
>
D2 convertDuration(const D1& t) = delete;

// *INDENT-OFF*
template<> ::builtin_interfaces::msg::Duration convertDuration(const ::rclcpp::Duration& t);
template<> rcl_duration_t convertDuration(const ::rclcpp::Duration& t);
template<> rcutils_duration_value_t convertDuration(const ::rclcpp::Duration& t);
template<> rmw_time_t convertDuration(const ::rclcpp::Duration& t);
template<> double convertDuration(const ::rclcpp::Duration& t);
template<typename D, typename ::std::enable_if_t<::cras::is_duration_v<D>>* = nullptr>
D convertDuration(const ::rclcpp::Duration& t) {
  return t.to_chrono<D>();
}

template<> ::rclcpp::Duration convertDuration(const ::builtin_interfaces::msg::Duration& t);
template<> rcl_duration_t convertDuration(const ::builtin_interfaces::msg::Duration& t);
template<> rcutils_duration_value_t convertDuration(const ::builtin_interfaces::msg::Duration& t);
template<> rmw_time_t convertDuration(const ::builtin_interfaces::msg::Duration& t);
template<> double convertDuration(const ::builtin_interfaces::msg::Duration& t);
template<typename D, typename ::std::enable_if_t<::cras::is_duration_v<D>>* = nullptr>
D convertDuration(const ::builtin_interfaces::msg::Duration& t) {
  return rclcpp::Duration(t).to_chrono<D>();
}

template<> ::rclcpp::Duration convertDuration(const rcl_duration_t& t);
template<> ::builtin_interfaces::msg::Duration convertDuration(const rcl_duration_t& t);
template<> rcutils_duration_value_t convertDuration(const rcl_duration_t& t);
template<> rmw_time_t convertDuration(const rcl_duration_t& t);
template<> double convertDuration(const rcl_duration_t& t);
template<typename D, typename ::std::enable_if_t<::cras::is_duration_v<D>>* = nullptr>
D convertDuration(const rcl_duration_t& t) {
  return ::std::chrono::duration_cast<D>(::std::chrono::nanoseconds(t.nanoseconds));
}

template<> ::rclcpp::Duration convertDuration(const rcutils_duration_value_t& t);
template<> ::builtin_interfaces::msg::Duration convertDuration(const rcutils_duration_value_t& t);
template<> rcl_duration_t convertDuration(const rcutils_duration_value_t& t);
template<> rmw_time_t convertDuration(const rcutils_duration_value_t& t);
template<> double convertDuration(const rcutils_duration_value_t& t);
template<typename D, typename ::std::enable_if_t<::cras::is_duration_v<D>>* = nullptr>
D convertDuration(const rcutils_duration_value_t& t) {
  return ::std::chrono::duration_cast<D>(::std::chrono::nanoseconds(t));
}

template<> ::rclcpp::Duration convertDuration(const rmw_time_t& t);
template<> ::builtin_interfaces::msg::Duration convertDuration(const rmw_time_t& t);
template<> rcl_duration_t convertDuration(const rmw_time_t& t);
template<> rcutils_duration_value_t convertDuration(const rmw_time_t& t);
template<> double convertDuration(const rmw_time_t& t);
template<typename D, typename ::std::enable_if_t<::cras::is_duration_v<D>>* = nullptr>
D convertDuration(const rmw_time_t& t) {
  return ::rclcpp::Duration::from_rmw_time(t).to_chrono<D>();
}

template<> ::rclcpp::Duration convertDuration(const double& t);
template<> ::builtin_interfaces::msg::Duration convertDuration(const double& t);
template<> rcl_duration_t convertDuration(const double& t);
template<> rcutils_duration_value_t convertDuration(const double& t);
template<> rmw_time_t convertDuration(const double& t);
template<typename D, typename ::std::enable_if_t<::cras::is_duration_v<D>>* = nullptr>
D convertDuration(const double& t) {
  return ::rclcpp::Duration::from_seconds(t).to_chrono<D>();
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<::cras::is_duration_v<D1> && ::std::is_same_v<D2, ::rclcpp::Duration>>* = nullptr
>
D2 convertDuration(const D1& t) {
  return ::rclcpp::Duration::from_nanoseconds(::std::chrono::duration_cast<::std::chrono::nanoseconds>(t).count());
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<
    ::cras::is_duration_v<D1> &&
    ::std::is_same_v<D2, ::builtin_interfaces::msg::Duration>
  >* = nullptr
>
D2 convertDuration(const D1& t) {
  return ::rclcpp::Duration::from_nanoseconds(::std::chrono::duration_cast<::std::chrono::nanoseconds>(t).count());
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<::cras::is_duration_v<D1> && ::std::is_same_v<D2, rcl_duration_t>>* = nullptr
>
D2 convertDuration(const D1& t) {
  return {::std::chrono::duration_cast<::std::chrono::nanoseconds>(t).count()};
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<::cras::is_duration_v<D1> && ::std::is_same_v<D2, rcutils_duration_value_t>>* = nullptr
>
D2 convertDuration(const D1& t) {
  return ::std::chrono::duration_cast<::std::chrono::nanoseconds>(t).count();
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<::cras::is_duration_v<D1> && ::std::is_same_v<D2, rmw_time_t>>* = nullptr
>
D2 convertDuration(const D1& t) {
  return rmw_time_from_nsec(::std::chrono::duration_cast<::std::chrono::nanoseconds>(t).count());
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<::cras::is_duration_v<D1> && ::std::is_same_v<D2, double>>* = nullptr
>
D2 convertDuration(const D1& t) {
  const auto ns = static_cast<::rcutils_duration_value_t>(
    ::std::chrono::duration_cast<::std::chrono::nanoseconds>(t).count());
  return ::rclcpp::Duration::from_nanoseconds(ns).seconds();
}

template<
  typename D2, typename D1,
  typename ::std::enable_if_t<
      ::cras::is_duration_v<D1> && ::cras::is_duration_v<D2> &&
      !::std::is_same_v<D1, D2>
  >* = nullptr
>
D2 convertDuration(const D1& t) {
  return ::std::chrono::duration_cast<D2>(t);
}
// *INDENT-ON*

/**
 * Split the given duration to seconds and the fractional second converted to nanoseconds.
 *
 * \tparam T1 Duration type.
 * \param t Duration to convert.
 * \return A pair consisting of the whole number of seconds and the number of nanoseconds in the fractional second.
 */
template<
    typename T1,
    typename ::std::enable_if_t<::cras::DurationType<T1>::value && !::cras::TimeType<T1>::value>* = nullptr
>
std::pair<int32_t, uint32_t> secNsec(const T1& t) {
  const auto time = convertDuration<::builtin_interfaces::msg::Duration>(t);
  return {time.sec, time.nanosec};
}

/**
 * Get the whole number of seconds from the given duration.
 *
 * \tparam T1 Duration type.
 * \param t Duration to convert.
 * \return The number of whole seconds.
 */
template<
    typename T1,
    typename ::std::enable_if_t<::cras::DurationType<T1>::value && !::cras::TimeType<T1>::value>* = nullptr
>
int32_t sec(const T1& t) {
  return ::cras::secNsec(t).first;
}

/**
 * Get the fractional second converted to nanoseconds.
 *
 * \tparam T1 Duration type.
 * \param t Duration to convert.
 * \return The number of nanoseconds in the fractional second.
 */
template<
    typename T1,
    typename ::std::enable_if_t<::cras::DurationType<T1>::value && !::cras::TimeType<T1>::value>* = nullptr
>
uint32_t nanosec(const T1& t) {
  return ::cras::secNsec(t).second;
}

/**
 * \brief Return remaining time to timeout from the query time.
 * \param[in] query The query time, e.g. of a TF.
 * \param[in] timeout Maximum time to wait from the query time onwards.
 * \param[in] clock The clock to use.
 * \return The remaining time.
 */
::rclcpp::Duration remainingTime(const ::rclcpp::Time& query, double timeout,
    const ::rclcpp::Clock::ConstSharedPtr& clock = ::std::make_shared<::rclcpp::Clock>(RCL_SYSTEM_TIME));

/**
 * \brief Return remaining time to timeout from the query time.
 * \param[in] query The query time, e.g. of a TF.
 * \param[in] timeout Maximum time to wait from the query time onwards.
 * \param[in] clock The clock to use.
 * \return The remaining time.
 */
::rclcpp::Duration remainingTime(const ::rclcpp::Time& query, const ::rclcpp::Duration& timeout,
    const ::rclcpp::Clock::ConstSharedPtr& clock = ::std::make_shared<::rclcpp::Clock>(RCL_SYSTEM_TIME));

/**
 * \brief Return the frequency represented by the given rate.
 * \param[in] rate The rate to convert.
 * \param[in] max_period_means_zero If true, return 0 frequency in case the rate's cycle time is the maximum duration.
 * \return The frequency.
 */
double frequency(const ::rclcpp::Rate& rate, bool max_period_means_zero = false);

/**
 * \brief Return a rate representing the given frequency. If the frequency is zero or too small, return min/max
 * representable rate.
 * \param[in] frequency The frequency to convert.
 * \param[in] clock The clock to use.
 * \return The corresponding Rate object.
 */
::rclcpp::Rate safeRate(double frequency,
    const ::rclcpp::Clock::SharedPtr& clock = ::std::make_shared<::rclcpp::Clock>(RCL_SYSTEM_TIME));

/**
 * \brief Return a rate representing the given frequency. If the frequency is zero or too small, return min/max
 * representable rate.
 * \param[in] frequency The frequency to convert.
 * \return The corresponding Rate object.
 */
::rclcpp::WallRate safeWallRate(double frequency);

/**
 * \brief Return current ROS time if it has already been initialized, or current wall time.
 * \return Current time.
 */
// ::ros::Time nowFallbackToWall();

/**
 * \brief Add the given duration to the given time, but saturate the result instead of throwing exception on overflow.
 * \param[in] time The time to be added to.
 * \param[in] duration The duration to add.
 * \return The time plus the duration saturated between 0 and TIME_MAX.
 */
::rclcpp::Time saturateAdd(const ::rclcpp::Time& time, const ::rclcpp::Duration& duration);

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The ROS time.
 * \return The year.
 */
int getYear(const ::rclcpp::Time& time);

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The chrono time.
 * \return The year.
 */
int getYear(const ::std::chrono::system_clock::time_point& time);

/**
 * \brief Trivial NodeClockInterface that just gives access to the given clock instance.
 */
class SimpleClockInterface : public ::rclcpp::node_interfaces::NodeClockInterface{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(SimpleClockInterface)

  explicit SimpleClockInterface(const ::rclcpp::Clock::SharedPtr& clock) : clock_(clock) {
  }

  ::rclcpp::Clock::SharedPtr get_clock() override{
    return this->clock_;
  }

  ::rclcpp::Clock::ConstSharedPtr get_clock() const override{
    return this->clock_;
  }

protected:
  ::rclcpp::Clock::SharedPtr clock_;
};

}  // namespace cras

namespace rclcpp {

/**
 * \brief Test two rates for equality.
 * \param[in] r1 First rate.
 * \param[in] r2 Second rate.
 * \return Whether the rates are exactly equal and use the same clock type.
 */
bool operator==(const ::rclcpp::Rate& r1, const ::rclcpp::Rate& r2);

/**
 * \brief Multiply two durations. The units of the result are [s^2].
 * \param[in] val1 Duration to multiply.
 * \param[in] val2 Duration to multiply.
 * \return The multiple.
 * \note This function will throw an exception if the result is out of bounds of the standard ROS duration range.
 */
::rclcpp::Duration operator*(const ::rclcpp::Duration& val1, const ::rclcpp::Duration& val2);

/**
 * \brief Divide duration val1 by val2. The result is unitless.
 * \param[in] numerator Duration to multiply.
 * \param[in] denominator Duration to multiply.
 * \return The fraction.
 * \note This function will throw an exception if the result is out of bounds of the standard ROS duration range.
 */
::rclcpp::Duration operator/(const ::rclcpp::Duration& numerator, const ::rclcpp::Duration& denominator);

}  // namespace rclcpp
