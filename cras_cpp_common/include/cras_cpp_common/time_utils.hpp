/**
 * \file
 * \brief Utilities for working with time.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
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

namespace cras
{

// *INDENT-OFF*
/**
 * Type trait designating all types that can represent a time point.
 */
template<typename> struct TimeType : ::std::false_type {};

template<> struct TimeType<::rclcpp::Time> : ::std::true_type {};
template<> struct TimeType<::builtin_interfaces::msg::Time> : ::std::true_type {};
template<> struct TimeType<rcl_time_point_t> : ::std::true_type {};
template<> struct TimeType<rcl_time_point_value_t> : ::std::true_type {};
template<> struct TimeType<rmw_time_t> : ::std::true_type {};
template<> struct TimeType<double> : ::std::true_type {};
template<> struct TimeType<::tm> : ::std::true_type {};
template<typename C, typename D> struct TimeType<::std::chrono::time_point<C, D>> : ::std::true_type {};
// *INDENT-ON*

/**
 * \brief Identity function for time types.
 * \tparam T The time type.
 * \param[in] t The input time.
 * \return t
 */
template<typename T, typename ::std::enable_if_t<::cras::TimeType<T>::value>* = nullptr>
T convertTime(const T& t)
{
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
    !::std::is_same_v<T1, T2>>* = nullptr
>
T2 convertTime(const T1& t) = delete;

// *INDENT-OFF*
template<> ::builtin_interfaces::msg::Time convertTime(const ::rclcpp::Time& t);
template<> double convertTime(const ::rclcpp::Time& t);
template<> rcl_time_point_value_t convertTime(const ::rclcpp::Time& t);
template<> ::tm convertTime(const ::rclcpp::Time& t);
template<> ::std::chrono::system_clock::time_point convertTime(const ::rclcpp::Time& t);

template<> ::rclcpp::Time convertTime(const ::builtin_interfaces::msg::Time& t);

template<> ::rclcpp::Time convertTime(const double& t);

template<> ::rclcpp::Time convertTime(const rcl_time_point_value_t& t);
template<> double convertTime(const rcl_time_point_value_t& t);

template<> ::rclcpp::Time convertTime(const rmw_time_t& t);

template<> ::rclcpp::Time convertTime(const ::tm& t);
::cras::expected<::rclcpp::Time, ::std::string> fromStructTm(const tm& time);
// *INDENT-ON*

/**
 * Convert between various time representations.
 *
 * \tparam T1 The output type.
 * \param[in] t The time to convert.
 * \param[in] clockType The clock type of the output.
 * \return The converted time with the given clock type.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
::rclcpp::Time convertTime(const T1& t, const ::rcl_clock_type_t clockType)
{
  return ::rclcpp::Time(convertTime<::rclcpp::Time>(t).nanoseconds(), clockType);
}

/**
 * Split the given time to seconds and the fractional second converted to nanoseconds.
 *
 * \tparam T1 Time type.
 * \param t Time to convert.
 * \return A pair consisting of the whole number of seconds and the number of nanoseconds in the fractional second.
 */
template<typename T1, typename ::std::enable_if_t<::cras::TimeType<T1>::value>* = nullptr>
std::pair<int32_t, uint32_t> secNsec(const T1& t)
{
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
int32_t sec(const T1& t)
{
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
uint32_t nanosec(const T1& t)
{
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
double float_secs(const T1& t)
{
  return ::cras::convertTime<::rclcpp::Time>(t).seconds();
}

// *INDENT-OFF*
/**
 * Type trait designating all types that can represent a time duration.
 */
template<typename> struct DurationType : ::std::false_type {};

template<> struct DurationType<::rclcpp::Duration> : ::std::true_type {};
template<> struct DurationType<::builtin_interfaces::msg::Duration> : ::std::true_type {};
template<> struct DurationType<rcl_duration_t> : ::std::true_type {};
template<> struct DurationType<rcl_duration_value_t> : ::std::true_type {};
template<> struct DurationType<rmw_time_t> : ::std::true_type {};
// template<> struct DurationType<rmw_duration_t> : ::std::true_type {};  // identical underlying type
template<> struct DurationType<double> : ::std::true_type {};
template<typename R, typename P> struct DurationType<::std::chrono::duration<R, P>> : ::std::true_type {};
// *INDENT-ON*

/**
 * \brief Identity function for duration types.
 * \tparam T The duration type.
 * \param[in] t The input duration.
 * \return t
 */
template<typename T, typename ::std::enable_if_t<::cras::DurationType<T>::value>* = nullptr>
T convertDuration(const T& t)
{
  return t;
}

/**
 * Convert between various duration representations.
 *
 * This template is the fallback used in case two duration types are given but their conversion is not yet implemented.
 *
 * \tparam T2 The desired type.
 * \tparam T1 The input type.
 * \param[in] t The duration to convert.
 * \return The converted duration.
 */
template<
  typename T2, typename T1,
  typename ::std::enable_if_t<
    ::cras::DurationType<T1>::value &&
    ::cras::DurationType<T2>::value &&
    !::std::is_same_v<T1, T2>>* = nullptr
>
T2 convertDuration(const T1& t) = delete;

// *INDENT-OFF*
template<> ::builtin_interfaces::msg::Duration convertDuration(const ::rclcpp::Duration& t);
template<> double convertDuration(const ::rclcpp::Duration& t);
template<> rcl_duration_value_t convertDuration(const ::rclcpp::Duration& t);
template<> rmw_time_t convertDuration(const ::rclcpp::Duration& t);

template<typename R, typename P>
std::chrono::duration<R, P> convertDuration(const ::rclcpp::Duration& t)
{
  return t.to_chrono<std::chrono::duration<R, P>>();
}

template<> ::rclcpp::Duration convertDuration(const ::builtin_interfaces::msg::Duration& t);

template<> ::rclcpp::Duration convertDuration(const double& t);

template<> ::rclcpp::Duration convertDuration(const rcl_duration_value_t& t);
template<> double convertDuration(const rcl_duration_value_t& t);

template<> ::rclcpp::Duration convertDuration(const rcl_duration_t& t);

template<> ::rclcpp::Duration convertDuration(const rmw_time_t& t);
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
std::pair<int32_t, uint32_t> secNsec(const T1& t)
{
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
int32_t sec(const T1& t)
{
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
uint32_t nanosec(const T1& t)
{
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
 * \param[in] maxPeriodMeansZero If true, return 0 frequency in case the rate's cycle time is the maximum duration.
 * \return The frequency.
 */
double frequency(const ::rclcpp::Rate& rate, bool maxPeriodMeansZero = false);

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
class SimpleClockInterface : public ::rclcpp::node_interfaces::NodeClockInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(SimpleClockInterface)

  explicit SimpleClockInterface(const ::rclcpp::Clock::SharedPtr& clock) : clock(clock)
  {
  }

  ::rclcpp::Clock::SharedPtr get_clock() override
  {
    return this->clock;
  }

  ::rclcpp::Clock::ConstSharedPtr get_clock() const override
  {
    return this->clock;
  }

protected:
  ::rclcpp::Clock::SharedPtr clock;
};

}

namespace rclcpp
{

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

}
