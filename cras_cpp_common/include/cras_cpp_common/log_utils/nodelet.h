#pragma once

/**
 * \file
 * \brief Log helper redirecting the logging calls to NODELET_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <string>

#include <nodelet/nodelet.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/nodelet_utils/log_macros.h>

namespace cras
{
/**
 * Log helper delegating the logging calls to the NODELET_ macros.
 */
class NodeletLogHelper : public ::cras::LogHelper
{
public:
  
  //! \brief Type of the function returning the name of the nodelet.
  typedef ::std::function<const ::std::string&()> GetNameFn;
  
  /**
   * Create the log helper reporting as the nodelet of name returned by getNameFn.
   * @param getNameFn A function returning the name of the nodelet.
   */
  explicit NodeletLogHelper(const GetNameFn& getNameFn);
  ~NodeletLogHelper() override;

protected:
  /**
   * \brief Used by the NODELET_ logging macros.
   * \return Name of the bound nodelet.
   */
  const ::std::string& getName() const;

	/**
	 * \brief Used by the NODELET_ logging macros.
	 * \param[in] suffix The suffix.
	 * \return Suffixed name of the bound nodelet.
	 */
	::std::string getSuffixedName(const ::std::string& suffix) const;

  void printDebug(const ::std::string& text) const override
	{
		NODELET_DEBUG("%s", text.c_str());
	}
	void printDebugNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_DEBUG_NAMED(name, "%s", text.c_str());
	}
	void printDebugCond(const bool condition, const ::std::string& text) const override
	{
		NODELET_DEBUG_COND(condition, "%s", text.c_str());
	}
	void printDebugCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_DEBUG_COND_NAMED(condition, name, "%s", text.c_str());
	}
	void printDebugOnce(const ::std::string& text) const override
	{
		NODELET_DEBUG_ONCE("%s", text.c_str());
	}
	void printDebugOnceNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_DEBUG_ONCE_NAMED(name, "%s", text.c_str());
	}
	void printDebugThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_DEBUG_THROTTLE(period, "%s", text.c_str());
	}
	void printDebugThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_DEBUG_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printDebugDelayedThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_DEBUG_DELAYED_THROTTLE(period, "%s", text.c_str());
	}
	void printDebugDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_DEBUG_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printDebugFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
	{
		NODELET_DEBUG_FILTER(filter, "%s", text.c_str());
	}
	void printDebugFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_DEBUG_FILTER_NAMED(filter, name, "%s", text.c_str());
	}
	
  void printInfo(const ::std::string& text) const override
	{
		auto name = this->getName();
		NODELET_INFO("%s", text.c_str());
	}
	void printInfoNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_INFO_NAMED(name, "%s", text.c_str());
	}
	void printInfoCond(const bool condition, const ::std::string& text) const override
	{
		NODELET_INFO_COND(condition, "%s", text.c_str());
	}
	void printInfoCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_INFO_COND_NAMED(condition, name, "%s", text.c_str());
	}
	void printInfoOnce(const ::std::string& text) const override
	{
		NODELET_INFO_ONCE("%s", text.c_str());
	}
	void printInfoOnceNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_INFO_ONCE_NAMED(name, "%s", text.c_str());
	}
	void printInfoThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_INFO_THROTTLE(period, "%s", text.c_str());
	}
	void printInfoThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_INFO_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printInfoDelayedThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_INFO_DELAYED_THROTTLE(period, "%s", text.c_str());
	}
	void printInfoDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_INFO_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printInfoFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
	{
		NODELET_INFO_FILTER(filter, "%s", text.c_str());
	}
	void printInfoFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_INFO_FILTER_NAMED(filter, name, "%s", text.c_str());
	}
	
  void printWarn(const ::std::string& text) const override
	{
		NODELET_WARN("%s", text.c_str());
	}
	void printWarnNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_WARN_NAMED(name, "%s", text.c_str());
	}
	void printWarnCond(const bool condition, const ::std::string& text) const override
	{
		NODELET_WARN_COND(condition, "%s", text.c_str());
	}
	void printWarnCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_WARN_COND_NAMED(condition, name, "%s", text.c_str());
	}
	void printWarnOnce(const ::std::string& text) const override
	{
		NODELET_WARN_ONCE("%s", text.c_str());
	}
	void printWarnOnceNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_WARN_ONCE_NAMED(name, "%s", text.c_str());
	}
	void printWarnThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_WARN_THROTTLE(period, "%s", text.c_str());
	}
	void printWarnThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_WARN_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printWarnDelayedThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_WARN_DELAYED_THROTTLE(period, "%s", text.c_str());
	}
	void printWarnDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_WARN_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printWarnFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
	{
		NODELET_WARN_FILTER(filter, "%s", text.c_str());
	}
	void printWarnFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_WARN_FILTER_NAMED(filter, name, "%s", text.c_str());
	}
	
  void printError(const ::std::string& text) const override
	{
		NODELET_ERROR("%s", text.c_str());
	}
	void printErrorNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_ERROR_NAMED(name, "%s", text.c_str());
	}
	void printErrorCond(const bool condition, const ::std::string& text) const override
	{
		NODELET_ERROR_COND(condition, "%s", text.c_str());
	}
	void printErrorCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_ERROR_COND_NAMED(condition, name, "%s", text.c_str());
	}
	void printErrorOnce(const ::std::string& text) const override
	{
		NODELET_ERROR_ONCE("%s", text.c_str());
	}
	void printErrorOnceNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_ERROR_ONCE_NAMED(name, "%s", text.c_str());
	}
	void printErrorThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_ERROR_THROTTLE(period, "%s", text.c_str());
	}
	void printErrorThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_ERROR_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printErrorDelayedThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_ERROR_DELAYED_THROTTLE(period, "%s", text.c_str());
	}
	void printErrorDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_ERROR_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printErrorFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
	{
		NODELET_ERROR_FILTER(filter, "%s", text.c_str());
	}
	void printErrorFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_ERROR_FILTER_NAMED(filter, name, "%s", text.c_str());
	}
	
  void printFatal(const ::std::string& text) const override
	{
		NODELET_FATAL("%s", text.c_str());
	}
	void printFatalNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_FATAL_NAMED(name, "%s", text.c_str());
	}
	void printFatalCond(const bool condition, const ::std::string& text) const override
	{
		NODELET_FATAL_COND(condition, "%s", text.c_str());
	}
	void printFatalCondNamed(const bool condition, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_FATAL_COND_NAMED(condition, name, "%s", text.c_str());
	}
	void printFatalOnce(const ::std::string& text) const override
	{
		NODELET_FATAL_ONCE("%s", text.c_str());
	}
	void printFatalOnceNamed(const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_FATAL_ONCE_NAMED(name, "%s", text.c_str());
	}
	void printFatalThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_FATAL_THROTTLE(period, "%s", text.c_str());
	}
	void printFatalThrottleNamed(const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_FATAL_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printFatalDelayedThrottle(const double period, const ::std::string& text) const override
	{
		NODELET_FATAL_DELAYED_THROTTLE(period, "%s", text.c_str());
	}
	void printFatalDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_FATAL_DELAYED_THROTTLE_NAMED(period, name, "%s", text.c_str());
	}
	void printFatalFilter(::ros::console::FilterBase* filter, const ::std::string& text) const override
	{
		NODELET_FATAL_FILTER(filter, "%s", text.c_str());
	}
	void printFatalFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const override
	{
		NODELET_FATAL_FILTER_NAMED(filter, name, "%s", text.c_str());
	}

  //! \brief Function returning the name of the nodelet.
  GetNameFn getNameFn;
};

}