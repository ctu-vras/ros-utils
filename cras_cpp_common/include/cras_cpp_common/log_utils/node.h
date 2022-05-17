#pragma once

/**
 * \file
 * \brief Log helper redirecting the logging calls to ROS_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/log_utils.h>

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class NodeLogHelper : public ::cras::LogHelper
{
protected:
  void printDebug(const ::std::string& text) const override
  {
    ROS_DEBUG("%s", text.c_str());
  }
  void printDebugNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_DEBUG_NAMED(name, "%s", text.c_str());
  }
  void printDebugCond(const bool condition, const ::std::string& text) const override
  {
    ROS_DEBUG_COND(condition, "%s", text.c_str());
  }
  void printDebugCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_DEBUG_COND_NAMED(condition, name, "%s", text.c_str());
  }
  void printDebugOnce(const ::std::string& text) const override
  {
    ROS_DEBUG_ONCE("%s", text.c_str());
  }
  void printDebugOnceNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_DEBUG_ONCE_NAMED(name, "%s", text.c_str());
  }
  void printDebugThrottle(const double period, const ::std::string& text) const override
  {
    ROS_DEBUG_THROTTLE(period, "%s", text.c_str());
  }
  void printDebugThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_DEBUG_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printDebugDelayedThrottle(const double period, const ::std::string& text) const override
  {
    ROS_DEBUG_DELAYED_THROTTLE(period, "%s", text.c_str());
  }
  void printDebugDelayedThrottleNamed(
    const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_DEBUG_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printDebugFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
  {
    ROS_DEBUG_FILTER(filter, "%s", text.c_str());
  }
  void printDebugFilterNamed(
    ::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_DEBUG_FILTER_NAMED(filter, name, "%s", text.c_str());
  }

  void printInfo(const ::std::string& text) const override
  {
    ROS_INFO("%s", text.c_str());
  }
  void printInfoNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_INFO_NAMED(name, "%s", text.c_str());
  }
  void printInfoCond(const bool condition, const ::std::string& text) const override
  {
    ROS_INFO_COND(condition, "%s", text.c_str());
  }
  void printInfoCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_INFO_COND_NAMED(condition, name, "%s", text.c_str());
  }
  void printInfoOnce(const ::std::string& text) const override
  {
    ROS_INFO_ONCE("%s", text.c_str());
  }
  void printInfoOnceNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_INFO_ONCE_NAMED(name, "%s", text.c_str());
  }
  void printInfoThrottle(const double period, const ::std::string& text) const override
  {
    ROS_INFO_THROTTLE(period, "%s", text.c_str());
  }
  void printInfoThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_INFO_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printInfoDelayedThrottle(const double period, const ::std::string& text) const override
  {
    ROS_INFO_DELAYED_THROTTLE(period, "%s", text.c_str());
  }
  void printInfoDelayedThrottleNamed(
    const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_INFO_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printInfoFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
  {
    ROS_INFO_FILTER(filter, "%s", text.c_str());
  }
  void printInfoFilterNamed(
    ::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_INFO_FILTER_NAMED(filter, name, "%s", text.c_str());
  }

  void printWarn(const ::std::string& text) const override
  {
    ROS_WARN("%s", text.c_str());
  }
  void printWarnNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_WARN_NAMED(name, "%s", text.c_str());
  }
  void printWarnCond(const bool condition, const ::std::string& text) const override
  {
    ROS_WARN_COND(condition, "%s", text.c_str());
  }
  void printWarnCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_WARN_COND_NAMED(condition, name, "%s", text.c_str());
  }
  void printWarnOnce(const ::std::string& text) const override
  {
    ROS_WARN_ONCE("%s", text.c_str());
  }
  void printWarnOnceNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_WARN_ONCE_NAMED(name, "%s", text.c_str());
  }
  void printWarnThrottle(const double period, const ::std::string& text) const override
  {
    ROS_WARN_THROTTLE(period, "%s", text.c_str());
  }
  void printWarnThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_WARN_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printWarnDelayedThrottle(const double period, const ::std::string& text) const override
  {
    ROS_WARN_DELAYED_THROTTLE(period, "%s", text.c_str());
  }
  void printWarnDelayedThrottleNamed(
    const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_WARN_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printWarnFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
  {
    ROS_WARN_FILTER(filter, "%s", text.c_str());
  }
  void printWarnFilterNamed(
    ::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_WARN_FILTER_NAMED(filter, name, "%s", text.c_str());
  }

  void printError(const ::std::string& text) const override
  {
    ROS_ERROR("%s", text.c_str());
  }
  void printErrorNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_ERROR_NAMED(name, "%s", text.c_str());
  }
  void printErrorCond(const bool condition, const ::std::string& text) const override
  {
    ROS_ERROR_COND(condition, "%s", text.c_str());
  }
  void printErrorCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_ERROR_COND_NAMED(condition, name, "%s", text.c_str());
  }
  void printErrorOnce(const ::std::string& text) const override
  {
    ROS_ERROR_ONCE("%s", text.c_str());
  }
  void printErrorOnceNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_ERROR_ONCE_NAMED(name, "%s", text.c_str());
  }
  void printErrorThrottle(const double period, const ::std::string& text) const override
  {
    ROS_ERROR_THROTTLE(period, "%s", text.c_str());
  }
  void printErrorThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_ERROR_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printErrorDelayedThrottle(const double period, const ::std::string& text) const override
  {
    ROS_ERROR_DELAYED_THROTTLE(period, "%s", text.c_str());
  }
  void printErrorDelayedThrottleNamed(
    const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_ERROR_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printErrorFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
  {
    ROS_ERROR_FILTER(filter, "%s", text.c_str());
  }
  void printErrorFilterNamed(
    ::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_ERROR_FILTER_NAMED(filter, name, "%s", text.c_str());
  }
  
  void printFatal(const ::std::string& text) const override
  {
    ROS_FATAL("%s", text.c_str());
  }
  void printFatalNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_FATAL_NAMED(name, "%s", text.c_str());
  }
  void printFatalCond(const bool condition, const ::std::string& text) const override
  {
    ROS_FATAL_COND(condition, "%s", text.c_str());
  }
  void printFatalCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_FATAL_COND_NAMED(condition, name, "%s", text.c_str());
  }
  void printFatalOnce(const ::std::string& text) const override
  {
    ROS_FATAL_ONCE("%s", text.c_str());
  }
  void printFatalOnceNamed(const ::std::string& name, const ::std::string& text) const override
  {
    ROS_FATAL_ONCE_NAMED(name, "%s", text.c_str());
  }
  void printFatalThrottle(const double period, const ::std::string& text) const override
  {
    ROS_FATAL_THROTTLE(period, "%s", text.c_str());
  }
  void printFatalThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_FATAL_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printFatalDelayedThrottle(const double period, const ::std::string& text) const override
  {
    ROS_FATAL_DELAYED_THROTTLE(period, "%s", text.c_str());
  }
  void printFatalDelayedThrottleNamed(
    const double period, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_FATAL_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
  }
  void printFatalFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
  {
    ROS_FATAL_FILTER(filter, "%s", text.c_str());
  }
  void printFatalFilterNamed(
    ::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
  {
    ROS_FATAL_FILTER_NAMED(filter, name, "%s", text.c_str());
  }
};

}