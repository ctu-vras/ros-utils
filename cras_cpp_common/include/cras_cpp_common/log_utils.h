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
   * \brief Log an info message.
   * \param[in] text The message to log. 
   */
  virtual void printInfo(const ::std::string& text) const = 0;

  /**
   * \brief Log a warning message.
   * \param[in] text The message to log. 
   */
  virtual void printWarn(const ::std::string& text) const = 0;

  /**
   * \brief Log an error message.
   * \param[in] text The message to log. 
   */
  virtual void printError(const ::std::string& text) const = 0;

  /**
   * \brief Log a fatal message.
   * \param[in] text The message to log. 
   */
  virtual void printFatal(const ::std::string& text) const = 0;
};

typedef ::std::shared_ptr<::cras::LogHelper> LogHelperPtr;

}