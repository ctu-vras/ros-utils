#pragma once

/**
 * \file
 * \brief ROS logging helpers.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cstdarg>
#include <memory>
#include <string>

#include <ros/console.h>

#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

/**
 * \brief This class (reps. its descendants) provides unified access to ROS logging functions, be it ROS_* or NODELET_*.
 * 
 * Subclasses only need to implement the print* functions that print a pre-formatted string.
 */
class LogHelper
{
public:
  virtual ~LogHelper() = default;
  
	// DEBUG //

  /**
   * \brief Log a debug message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebug(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebug(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebug(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebug(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugCond(const bool condition, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugCond(const bool condition, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugCondNamed(const bool condition, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugCondNamed(const bool condition, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugOnce(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugOnce(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugOnceNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugOnceNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugThrottleNamed(const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugDelayedThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugDelayedThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period using a named logger, waiting period before printing the
   *        first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugDelayedThrottleNamed(
		const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message at most once per period using a named logger, waiting period before printing the
   * 			  first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugFilter(::ros::console::FilterBase* filter, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugFilter(::ros::console::FilterBase* filter, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
  /**
   * \brief Log a debug message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a debug message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logDebugFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG
    va_list(args);
    va_start(args, format);
    this->printDebugFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };

	// INFO //

  /**
   * \brief Log an info message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfo(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfo(::cras::format(format, args));
    va_end(args);
#endif
  };

  /**
   * \brief Log an info message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfo(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfo(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoCond(const bool condition, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoCond(const bool condition, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoCondNamed(const bool condition, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoCondNamed(const bool condition, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoOnce(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoOnce(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoOnceNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoOnceNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoThrottleNamed(const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoDelayedThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoDelayedThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period using a named logger, waiting period before printing the
   *        first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoDelayedThrottleNamed(
		const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message at most once per period using a named logger, waiting period before printing the
   * 			  first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoFilter(::ros::console::FilterBase* filter, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoFilter(::ros::console::FilterBase* filter, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
  /**
   * \brief Log a info message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a info message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logInfoFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_INFO
    va_list(args);
    va_start(args, format);
    this->printInfoFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
	// WARN //

  /**
   * \brief Log a warning message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarn(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarn(::cras::format(format, args));
    va_end(args);
#endif
  };

  /**
   * \brief Log a warning message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarn(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarn(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnCond(const bool condition, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnCond(const bool condition, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnCondNamed(const bool condition, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnCondNamed(const bool condition, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnOnce(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnOnce(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnOnceNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnOnceNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnThrottleNamed(const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnDelayedThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnDelayedThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period using a named logger, waiting period before printing the
   *        first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnDelayedThrottleNamed(
		const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message at most once per period using a named logger, waiting period before printing the
   * 			  first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnFilter(::ros::console::FilterBase* filter, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnFilter(::ros::console::FilterBase* filter, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
  /**
   * \brief Log a warn message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a warn message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logWarnFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN
    va_list(args);
    va_start(args, format);
    this->printWarnFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
	// ERROR //

  /**
   * \brief Log an error message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logError(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printError(::cras::format(format, args));
    va_end(args);
#endif
  };

  /**
   * \brief Log an error message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logError(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printError(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorCond(const bool condition, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorCond(const bool condition, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorCondNamed(const bool condition, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorCondNamed(const bool condition, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorOnce(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorOnce(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorOnceNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorOnceNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorThrottleNamed(const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorDelayedThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorDelayedThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period using a named logger, waiting period before printing the
   *        first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorDelayedThrottleNamed(
		const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message at most once per period using a named logger, waiting period before printing the
   * 			  first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
	inline void logErrorFilter(::ros::console::FilterBase* filter, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorFilter(::ros::console::FilterBase* filter, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
  /**
   * \brief Log a error message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a error message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logErrorFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_ERROR
    va_list(args);
    va_start(args, format);
    this->printErrorFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
	// FATAL //

  /**
   * \brief Log a fatal message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatal(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatal(::cras::format(format, args));
    va_end(args);
#endif
  };

  /**
   * \brief Log a fatal message.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatal(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatal(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalCond(const bool condition, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalCond(const bool condition, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalCond(condition, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalCondNamed(const bool condition, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalCondNamed(const bool condition, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalCondNamed(condition, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalOnce(const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message once.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalOnce(const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalOnce(::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalOnceNamed(const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalOnceNamed(const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalOnceNamed(name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalThrottleNamed(const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalDelayedThrottle(const double period, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalDelayedThrottle(const double period, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalDelayedThrottle(period, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period using a named logger, waiting period before printing the
   *        first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalDelayedThrottleNamed(
		const double period, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message at most once per period using a named logger, waiting period before printing the
   * 			  first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalDelayedThrottleNamed(
		const double period, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalDelayedThrottleNamed(period, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalFilter(::ros::console::FilterBase* filter, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message passing it through the given filter.
	 * \param[in] filter The filter.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalFilter(::ros::console::FilterBase* filter, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalFilter(filter, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
  /**
   * \brief Log a fatal message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const char* format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
  
  /**
   * \brief Log a fatal message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void logFatalFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string format, ...) const
  {
#if ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_FATAL
    va_list(args);
    va_start(args, format);
    this->printFatalFilterNamed(filter, name, ::cras::format(format, args));
    va_end(args);
#endif
  };
	
	// GENERIC LOGGING //

  /**
   * \brief Log a message using the given log severity.
   * \param[in] level Log severity level (one of the ros::console::Level enum constants).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void log(::ros::console::Level level, const char* format, ...) const
  {
    if (level < ROSCONSOLE_MIN_SEVERITY)
      return;

    va_list(args);
    va_start(args, format);
    const auto& text = ::cras::format(format, args);
    va_end(args);
    this->print(level, text);
  }

  /**
   * \brief Log a message using the given log severity.
   * \param[in] level Log severity level (one of the ros::console::Level enum constants).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  inline void log(::ros::console::Level level, const ::std::string format, ...) const
  {
    if (level < ROSCONSOLE_MIN_SEVERITY)
      return;

    va_list(args);
    va_start(args, format);
    const auto& text = ::cras::format(format, args);
    va_end(args);
    this->print(level, text);
  }

  /**
   * \brief Log a message using the given log severity.
   * \param[in] level Log severity level (one of the ros::console::Level enum constants).
   * \param[in] text The message to log.
   */
  inline void print(::ros::console::Level level, const ::std::string& text) const
  {
    switch (level)
    {
      case ::ros::console::Level::Debug:
        this->printDebug(text);
        break;
      case ::ros::console::Level::Info:
        this->printInfo(text);
        break;
      case ::ros::console::Level::Warn:
        this->printWarn(text);
        break;
      case ::ros::console::Level::Error:
        this->printError(text);
        break;
      case ::ros::console::Level::Fatal:
        this->printFatal(text);
        break;
      default:
        this->logError("Invalid log level %i. Printing as error level.", level);
        this->printError(text);
        break;
    }
  }

protected:
  /**
   * \brief Log a debug message.
   * \param[in] text The message to log. 
   */
  virtual void printDebug(const ::std::string& text) const = 0;
	
  /**
   * \brief Log a debug message using a named logger.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] text The message to log. 
   */
  virtual void printDebugNamed(const ::std::string& name, const ::std::string& text) const = 0;
	
  /**
   * \brief Log a debug message if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] text The message to log. 
   */
  virtual void printDebugCond(bool condition, const ::std::string& text) const = 0;
	
  /**
   * \brief Log a debug message using a named logger if condition is true.
   * \param[in] condition If true, the message will be logged.
   * \param[in] name Name (suffix) of the named logger.
   * \param[in] text The message to log. 
   */
  virtual void printDebugCondNamed(bool condition, const ::std::string& name, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message once.
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugOnce(const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugOnceNamed(const ::std::string& name, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message at most once per period.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugThrottle(double period, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugThrottleNamed(double period, const ::std::string& name, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugDelayedThrottle(double period, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message at most once per period using a named logger, waiting period before printing the first
	 *        message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugDelayedThrottleNamed(
		double period, const ::std::string& name, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugFilter(::ros::console::FilterBase* filter, const ::std::string& text) const = 0;
	
	/**
	 * \brief Log a debug message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printDebugFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const = 0;

  /**
   * \brief Log an info message.
   * \param[in] text The message to log. 
   */
  virtual void printInfo(const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoCond(bool condition, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message using a named logger if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoCondNamed(bool condition, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message once.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoOnce(const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoOnceNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message at most once per period.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoThrottleNamed(double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoDelayedThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message at most once per period using a named logger, waiting period before printing the first
	 *        message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoDelayedThrottleNamed(
		double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoFilter(::ros::console::FilterBase* filter, const ::std::string& text) const = 0;

	/**
	 * \brief Log a info message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printInfoFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const = 0;

  /**
   * \brief Log a warning message.
   * \param[in] text The message to log. 
   */
  virtual void printWarn(const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnCond(bool condition, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message using a named logger if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnCondNamed(bool condition, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message once.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnOnce(const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnOnceNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message at most once per period.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnThrottleNamed(double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnDelayedThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message at most once per period using a named logger, waiting period before printing the first
	 *        message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnDelayedThrottleNamed(
		double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnFilter(::ros::console::FilterBase* filter, const ::std::string& text) const = 0;

	/**
	 * \brief Log a warn message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printWarnFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const = 0;

  /**
   * \brief Log an error message.
   * \param[in] text The message to log. 
   */
  virtual void printError(const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorCond(bool condition, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message using a named logger if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorCondNamed(bool condition, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message once.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorOnce(const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorOnceNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message at most once per period.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorThrottleNamed(double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorDelayedThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message at most once per period using a named logger, waiting period before printing the first
	 *        message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorDelayedThrottleNamed(
		double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorFilter(::ros::console::FilterBase* filter, const ::std::string& text) const = 0;

	/**
	 * \brief Log a error message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printErrorFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const = 0;

  /**
   * \brief Log a fatal message.
   * \param[in] text The message to log. 
   */
  virtual void printFatal(const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalCond(bool condition, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message using a named logger if condition is true.
	 * \param[in] condition If true, the message will be logged.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalCondNamed(bool condition, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message once.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalOnce(const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message once using a named logger.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalOnceNamed(const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message at most once per period.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message at most once per period using a named logger.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalThrottleNamed(double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message at most once per period, waiting period before printing the first message.
	 * \param[in] period The throttling period (s).
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalDelayedThrottle(double period, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message at most once per period using a named logger, waiting period before printing the first
	 *        message.
	 * \param[in] period The throttling period (s).
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalDelayedThrottleNamed(
		double period, const ::std::string& name, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalFilter(::ros::console::FilterBase* filter, const ::std::string& text) const = 0;

	/**
	 * \brief Log a fatal message using a named logger passing it through the given filter.
	 * \param[in] filter The filter.
	 * \param[in] name Name (suffix) of the named logger.
	 * \param[in] text The message to log. 
	 */
	virtual void printFatalFilterNamed(
		::ros::console::FilterBase* filter, const ::std::string& name, const ::std::string& text) const = 0;
};

typedef ::std::shared_ptr<::cras::LogHelper> LogHelperPtr;

}