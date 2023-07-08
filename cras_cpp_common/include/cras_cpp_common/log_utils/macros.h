#pragma once

/**
 * \file
 * \brief ROS logging helpers macros. Do not include this file directly. Include log_utils.h instead.
 *        These logging macros are almost a copy of those from rosconsole/macros_generated.h and ros/console.h, but
 *        modified so that the underlying logger implementation can be dynamically changed, and also an alternative time
 *        source can be provided for the `THROTTLE` macros. To get the logger, these macros call `getCrasLogger()`
 *        method. If nothing special is done, the compiler will pick up the `::getCrasLogger()` global function
 *        returning a globally usable logger that mimicks the `ROS_*` logging macros. If you want a different logger to
 *        be used, just call the macros in an environment where a more specific function named `getCrasLogger()` is
 *        defined - it can be a class method or a locally defined lambda.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// This file contains code heavily inspired by ros/console.h and rosconsole/macros_generated.h, both
// Copyright (c) 2008, Willow Garage, Inc. under the BSD-3 license.

#include <algorithm>
#include <unordered_map>
#include <vector>

/**
 * \def CRASCONSOLE_AUTOINIT
 * \brief Initializes the logging library.  Usually unnecessary to call directly.
 *
 * \param[in] logger The logger to use.
 */
#define CRASCONSOLE_AUTOINIT(logger) \
  do \
  { \
    if (ROS_UNLIKELY(!(logger)->initialized)) \
    { \
      (logger)->initialize(); \
    } \
  } while (false)

#define CRASCONSOLE_DEFINE_LOCATION(logger, cond, level, name) \
  CRASCONSOLE_AUTOINIT((logger)); \
  static ::std::unordered_map<const void*, ::ros::console::LogLocation> __rosconsole_define_location__map(1); \
  const auto __cras_logger_id__ = (logger)->getId(); \
  if (ROS_UNLIKELY( \
      __rosconsole_define_location__map.find(__cras_logger_id__) == __rosconsole_define_location__map.end())) \
    { \
      __rosconsole_define_location__map.emplace(__cras_logger_id__, \
        ::ros::console::LogLocation{false, false, ::ros::console::levels::Count, NULL}); \
    } \
  auto& __rosconsole_define_location__loc = __rosconsole_define_location__map[__cras_logger_id__]; \
  if (ROS_UNLIKELY(!__rosconsole_define_location__loc.initialized_)) \
  { \
    (logger)->initializeLogLocation(&__rosconsole_define_location__loc, (name), (level)); \
  } \
  if (ROS_UNLIKELY(__rosconsole_define_location__loc.level_ != (level))) \
  { \
    (logger)->setLogLocationLevel(&__rosconsole_define_location__loc, level); \
    (logger)->checkLogLocationEnabled(&__rosconsole_define_location__loc); \
  } \
  bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (cond);

#define CRASCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(logger, filter, ...) \
  (logger)->print((filter), __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, \
    __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__)

#define CRASCONSOLE_PRINT_AT_LOCATION(logger, ...) \
    CRASCONSOLE_PRINT_AT_LOCATION_WITH_FILTER((logger), NULL, __VA_ARGS__)

#define CRASCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(logger, filter, args) \
  do \
  { \
    std::stringstream __rosconsole_print_stream_at_location_with_filter__ss__; \
    __rosconsole_print_stream_at_location_with_filter__ss__ << args; \
    (logger)->print((filter), __rosconsole_define_location__loc.logger_, \
      __rosconsole_define_location__loc.level_, __rosconsole_print_stream_at_location_with_filter__ss__, \
      __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__); \
  } while (0)

#define CRASCONSOLE_PRINT_STREAM_AT_LOCATION(logger, args) \
    CRASCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER((logger), NULL, args)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with
 *        printf-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param[in] logger The logger to use.
 * \param[in] cond Boolean condition to be evaluated
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>". Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 */
#define CRAS_LOG_COND(logger, cond, level, name, ...) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), (cond), (level), (name)); \
    \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled)) \
    { \
      CRASCONSOLE_PRINT_AT_LOCATION(logger, __VA_ARGS__); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with
 *        stream-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param[in] logger The logger to use.
 * \param[in] cond Boolean condition to be evaluated
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] args The string to print, possibly containing streaming operators `<<`.
 */
#define CRAS_LOG_STREAM_COND(logger, cond, level, name, args) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), (cond), (level), (name)); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled)) \
    { \
      CRASCONSOLE_PRINT_STREAM_AT_LOCATION((logger), args); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with
 *        printf-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 */
#define CRAS_LOG_ONCE(logger, level, name, ...) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    static ::std::vector<const void*> hitSet; /* vector instead of set as we assume only a few elements */ \
    const auto hit = ::std::find(hitSet.begin(), hitSet.end(), __cras_logger_id__) != hitSet.end(); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROS_UNLIKELY(!hit)) \
    { \
      hitSet.push_back(__cras_logger_id__); \
      CRASCONSOLE_PRINT_AT_LOCATION((logger), __VA_ARGS__); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with
 *        printf-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] args The string to print, possibly containing streaming operators `<<`.
 */
#define CRAS_LOG_STREAM_ONCE(logger, level, name, args) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    static ::std::vector<const void*> hitSet; /* vector instead of set as we assume only a few elements */ \
    const auto hit = ::std::find(hitSet.begin(), hitSet.end(), __cras_logger_id__) != hitSet.end(); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROS_UNLIKELY(!hit)) \
    { \
      hitSet.push_back(__cras_logger_id__); \
      CRASCONSOLE_PRINT_STREAM_AT_LOCATION((logger), args); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with
 *        printf-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] period The period it should actually trigger at most. If ROS time has moved backwards, it will trigger
 *                   regardless.
 */
#define CRAS_LOG_THROTTLE(logger, period, level, name, ...) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    static ::std::unordered_map<const void*, double> lastHitMap(1); \
    auto& lastHit = lastHitMap[__cras_logger_id__]; \
    const auto now = (logger)->getTimeNow().toSec(); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROSCONSOLE_THROTTLE_CHECK(now, lastHit, (period))) \
    { \
      lastHit = now; \
      CRASCONSOLE_PRINT_AT_LOCATION((logger), __VA_ARGS__); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with
 *        printf-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] period The period it should actually trigger at most. If ROS time has moved backwards, it will trigger
 *                   regardless.
 * \param[in] args The string to print, possibly containing streaming operators `<<`.
 */
#define CRAS_LOG_STREAM_THROTTLE(logger, period, level, name, args) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    static ::std::unordered_map<const void*, double> lastHitMap(1); \
    auto& lastHit = lastHitMap[__cras_logger_id__]; \
    const auto now = (logger)->getTimeNow().toSec(); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROSCONSOLE_THROTTLE_CHECK(now, lastHit, (period))) \
    { \
      lastHit = now; \
      CRASCONSOLE_PRINT_STREAM_AT_LOCATION((logger), args); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with
 *        printf-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] period The period it should actually trigger at most, and the delay before which no message will be shown.
 *                   If ROS time has moved backwards, it will trigger regardless.
 */
#define CRAS_LOG_DELAYED_THROTTLE(logger, period, level, name, ...) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    const auto now = (logger)->getTimeNow().toSec(); \
    static ::std::unordered_map<const void*, double> lastHitMap(1); \
    lastHitMap.emplace(__cras_logger_id__, now); \
    auto& lastHit = lastHitMap[__cras_logger_id__]; \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROSCONSOLE_THROTTLE_CHECK(now, lastHit, (period)))\
    { \
      lastHit = now; \
      CRASCONSOLE_PRINT_AT_LOCATION((logger), __VA_ARGS__); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing and postponed
 *        first message
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] period The period it should actually trigger at most, and the delay before which no message will be shown.
 *                   If ROS time has moved backwards, it will trigger regardless.
 * \param[in] args The string to print, possibly containing streaming operators `<<`.
 */
#define CRAS_LOG_STREAM_DELAYED_THROTTLE(logger, period, level, name, args) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    const auto now = (logger)->getTimeNow().toSec(); \
    static ::std::unordered_map<const void*, double> lastHitMap(1); \
    lastHitMap.emplace(__cras_logger_id__, now); \
    auto& lastHit = lastHitMap[__cras_logger_id__]; \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROSCONSOLE_THROTTLE_CHECK(now, lastHit, (period))) \
    { \
      lastHit = now; \
      CRASCONSOLE_PRINT_STREAM_AT_LOCATION((logger), args); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with printf-style
 *        formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] filter pointer to the filter to be used
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 */
#define CRAS_LOG_FILTER(logger, filter, level, name, ...) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (filter)->isEnabled()) \
    { \
      CRASCONSOLE_PRINT_AT_LOCATION_WITH_FILTER((logger), filter, __VA_ARGS__); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with stream-style
 *        formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] filter pointer to the filter to be used
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] args The string to print, possibly containing streaming operators `<<`.
 */
#define CRAS_LOG_STREAM_FILTER(logger, filter, level, name, args) \
  do \
  { \
    CRASCONSOLE_DEFINE_LOCATION((logger), true, (level), (name)); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (filter)->isEnabled()) \
    { \
      CRASCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER((logger), filter, args); \
    } \
  } while (false)

/**
 * \brief Log to a given named logger at a given verbosity level, with printf-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 */
#define CRAS_LOG(logger, level, name, ...) CRAS_LOG_COND((logger), true, (level), (name), __VA_ARGS__)

/**
 * \brief Log to a given named logger at a given verbosity level, with stream-style formatting
 *
 * \param[in] logger The logger to use.
 * \param[in] level One of the levels specified in `::ros::console::levels::Level`
 * \param[in] name Name of the logger.  Note that this is the fully qualified name, and does NOT include
 *                 "ros.<package_name>".  Use `ROSCONSOLE_DEFAULT_NAME` if you would like to use the default name.
 * \param[in] args The string to print, possibly containing streaming operators `<<`.
 */
#define CRAS_LOG_STREAM(logger, level, name, args) CRAS_LOG_STREAM_COND((logger), true, (level), (name), args)

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_DEBUG)
#define CRAS_LOG_DEBUG(logger, ...)
#define CRAS_LOG_DEBUG_STREAM(logger, args)
#define CRAS_LOG_DEBUG_NAMED(logger, name, ...)
#define CRAS_LOG_DEBUG_STREAM_NAMED(logger, name, args)
#define CRAS_LOG_DEBUG_COND(logger, cond, ...)
#define CRAS_LOG_DEBUG_STREAM_COND(logger, cond, args)
#define CRAS_LOG_DEBUG_COND_NAMED(logger, cond, name, ...)
#define CRAS_LOG_DEBUG_STREAM_COND_NAMED(logger, cond, name, args)
#define CRAS_LOG_DEBUG_ONCE(logger, ...)
#define CRAS_LOG_DEBUG_STREAM_ONCE(logger, args)
#define CRAS_LOG_DEBUG_ONCE_NAMED(logger, name, ...)
#define CRAS_LOG_DEBUG_STREAM_ONCE_NAMED(logger, name, args)
#define CRAS_LOG_DEBUG_THROTTLE(logger, period, ...)
#define CRAS_LOG_DEBUG_STREAM_THROTTLE(logger, period, args)
#define CRAS_LOG_DEBUG_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_DEBUG_STREAM_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_DEBUG_DELAYED_THROTTLE(logger, period, ...)
#define CRAS_LOG_DEBUG_STREAM_DELAYED_THROTTLE(logger, period, args)
#define CRAS_LOG_DEBUG_DELAYED_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_DEBUG_FILTER(logger, filter, ...)
#define CRAS_LOG_DEBUG_STREAM_FILTER(logger, filter, args)
#define CRAS_LOG_DEBUG_FILTER_NAMED(logger, filter, name, ...)
#define CRAS_LOG_DEBUG_STREAM_FILTER_NAMED(logger, filter, name, args)
#define CRAS_DEBUG(...)
#define CRAS_DEBUG_STREAM(args)
#define CRAS_DEBUG_NAMED(name, ...)
#define CRAS_DEBUG_STREAM_NAMED(name, args)
#define CRAS_DEBUG_COND(cond, ...)
#define CRAS_DEBUG_STREAM_COND(cond, args)
#define CRAS_DEBUG_COND_NAMED(cond, name, ...)
#define CRAS_DEBUG_STREAM_COND_NAMED(cond, name, args)
#define CRAS_DEBUG_ONCE(...)
#define CRAS_DEBUG_STREAM_ONCE(args)
#define CRAS_DEBUG_ONCE_NAMED(name, ...)
#define CRAS_DEBUG_STREAM_ONCE_NAMED(name, args)
#define CRAS_DEBUG_THROTTLE(period, ...)
#define CRAS_DEBUG_STREAM_THROTTLE(period, args)
#define CRAS_DEBUG_THROTTLE_NAMED(period, name, ...)
#define CRAS_DEBUG_STREAM_THROTTLE_NAMED(period, name, args)
#define CRAS_DEBUG_DELAYED_THROTTLE(period, ...)
#define CRAS_DEBUG_STREAM_DELAYED_THROTTLE(period, args)
#define CRAS_DEBUG_DELAYED_THROTTLE_NAMED(period, name, ...)
#define CRAS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)
#define CRAS_DEBUG_FILTER(filter, ...)
#define CRAS_DEBUG_STREAM_FILTER(filter, args)
#define CRAS_DEBUG_FILTER_NAMED(filter, name, ...)
#define CRAS_DEBUG_STREAM_FILTER_NAMED(filter, name, args)
#else
#define CRAS_LOG_DEBUG(logger, ...) CRAS_LOG((logger), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM(logger, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_DEBUG_NAMED(logger, name, ...) CRAS_LOG((logger), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_NAMED(logger, name, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_DEBUG_COND(logger, cond, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_COND(logger, cond, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_DEBUG_COND_NAMED(logger, cond, name, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_COND_NAMED(logger, cond, name, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_DEBUG_ONCE(logger, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_ONCE(logger, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_DEBUG_ONCE_NAMED(logger, name, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_ONCE_NAMED(logger, name, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_DEBUG_THROTTLE(logger, period, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_THROTTLE(logger, period, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_DEBUG_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_DEBUG_DELAYED_THROTTLE(logger, period, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_DELAYED_THROTTLE(logger, period, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_DEBUG_DELAYED_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_DEBUG_FILTER(logger, filter, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_FILTER(logger, filter, args) CRAS_LOG_STREAM_FILTER((logger), (filter), ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_DEBUG_FILTER_NAMED(logger, filter, name, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_DEBUG_STREAM_FILTER_NAMED(logger, filter, name, args) CRAS_LOG_STREAM_FILTER(getCrasLogger(), (filter), ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_DEBUG(...) CRAS_LOG_DEBUG(getCrasLogger(), __VA_ARGS__)
#define CRAS_DEBUG_STREAM(args) CRAS_LOG_DEBUG_STREAM(getCrasLogger(), args)
#define CRAS_DEBUG_NAMED(name, ...) CRAS_LOG_DEBUG_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_NAMED(name, args) CRAS_LOG_DEBUG_STREAM_NAMED(getCrasLogger(), (name), args)
#define CRAS_DEBUG_COND(cond, ...) CRAS_LOG_DEBUG_COND(getCrasLogger(), (cond), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_COND(cond, args) CRAS_LOG_DEBUG_STREAM_COND(getCrasLogger(), (cond), args)
#define CRAS_DEBUG_COND_NAMED(cond, name, ...) CRAS_LOG_DEBUG_COND_NAMED(getCrasLogger(), (cond), (name), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_COND_NAMED(cond, name, args) CRAS_LOG_DEBUG_STREAM_COND_NAMED(getCrasLogger(), (cond), (name), args)  /* NOLINT */
#define CRAS_DEBUG_ONCE(...) CRAS_LOG_DEBUG_ONCE(getCrasLogger(), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_ONCE(args) CRAS_LOG_DEBUG_STREAM_ONCE(getCrasLogger(), args)
#define CRAS_DEBUG_ONCE_NAMED(name, ...) CRAS_LOG_DEBUG_ONCE_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_ONCE_NAMED(name, args) CRAS_LOG_DEBUG_STREAM_ONCE_NAMED(getCrasLogger(), (name), args)
#define CRAS_DEBUG_THROTTLE(period, ...) CRAS_LOG_DEBUG_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_THROTTLE(period, args) CRAS_LOG_DEBUG_STREAM_THROTTLE(getCrasLogger(), (period), args)
#define CRAS_DEBUG_THROTTLE_NAMED(period, name, ...) CRAS_LOG_DEBUG_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_DEBUG_STREAM_THROTTLE_NAMED(period, name, args) CRAS_LOG_DEBUG_STREAM_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_DEBUG_DELAYED_THROTTLE(period, ...) CRAS_LOG_DEBUG_DELAYED_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_DELAYED_THROTTLE(period, args) CRAS_LOG_DEBUG_STREAM_DELAYED_THROTTLE(getCrasLogger(), (period), args)  /* NOLINT */
#define CRAS_DEBUG_DELAYED_THROTTLE_NAMED(period, name, ...) CRAS_LOG_DEBUG_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(period, name, args) CRAS_LOG_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_DEBUG_FILTER(filter, ...) CRAS_LOG_DEBUG_FILTER(getCrasLogger(), (filter), __VA_ARGS__)
#define CRAS_DEBUG_STREAM_FILTER(filter, args) CRAS_LOG_DEBUG_STREAM_FILTER(getCrasLogger(), (filter), args)
#define CRAS_DEBUG_FILTER_NAMED(filter, name, ...) CRAS_LOG_DEBUG_FILTER_NAMED(getCrasLogger(), (filter), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_DEBUG_STREAM_FILTER_NAMED(filter, name, args) CRAS_LOG_DEBUG_STREAM_FILTER_NAMED(getCrasLogger(), (filter), (name), args)  /* NOLINT */
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_INFO)
#define CRAS_LOG_INFO(logger, ...)
#define CRAS_LOG_INFO_STREAM(logger, args)
#define CRAS_LOG_INFO_NAMED(logger, name, ...)
#define CRAS_LOG_INFO_STREAM_NAMED(logger, name, args)
#define CRAS_LOG_INFO_COND(logger, cond, ...)
#define CRAS_LOG_INFO_STREAM_COND(logger, cond, args)
#define CRAS_LOG_INFO_COND_NAMED(logger, cond, name, ...)
#define CRAS_LOG_INFO_STREAM_COND_NAMED(logger, cond, name, args)
#define CRAS_LOG_INFO_ONCE(logger, ...)
#define CRAS_LOG_INFO_STREAM_ONCE(logger, args)
#define CRAS_LOG_INFO_ONCE_NAMED(logger, name, ...)
#define CRAS_LOG_INFO_STREAM_ONCE_NAMED(logger, name, args)
#define CRAS_LOG_INFO_THROTTLE(logger, period, ...)
#define CRAS_LOG_INFO_STREAM_THROTTLE(logger, period, args)
#define CRAS_LOG_INFO_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_INFO_STREAM_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_INFO_DELAYED_THROTTLE(logger, period, ...)
#define CRAS_LOG_INFO_STREAM_DELAYED_THROTTLE(logger, period, args)
#define CRAS_LOG_INFO_DELAYED_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_INFO_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_INFO_FILTER(logger, filter, ...)
#define CRAS_LOG_INFO_STREAM_FILTER(logger, filter, args)
#define CRAS_LOG_INFO_FILTER_NAMED(logger, filter, name, ...)
#define CRAS_LOG_INFO_STREAM_FILTER_NAMED(logger, filter, name, args)
#define CRAS_INFO(...)
#define CRAS_INFO_STREAM(args)
#define CRAS_INFO_NAMED(name, ...)
#define CRAS_INFO_STREAM_NAMED(name, args)
#define CRAS_INFO_COND(cond, ...)
#define CRAS_INFO_STREAM_COND(cond, args)
#define CRAS_INFO_COND_NAMED(cond, name, ...)
#define CRAS_INFO_STREAM_COND_NAMED(cond, name, args)
#define CRAS_INFO_ONCE(...)
#define CRAS_INFO_STREAM_ONCE(args)
#define CRAS_INFO_ONCE_NAMED(name, ...)
#define CRAS_INFO_STREAM_ONCE_NAMED(name, args)
#define CRAS_INFO_THROTTLE(period, ...)
#define CRAS_INFO_STREAM_THROTTLE(period, args)
#define CRAS_INFO_THROTTLE_NAMED(period, name, ...)
#define CRAS_INFO_STREAM_THROTTLE_NAMED(period, name, args)
#define CRAS_INFO_DELAYED_THROTTLE(period, ...)
#define CRAS_INFO_STREAM_DELAYED_THROTTLE(period, args)
#define CRAS_INFO_DELAYED_THROTTLE_NAMED(period, name, ...)
#define CRAS_INFO_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)
#define CRAS_INFO_FILTER(filter, ...)
#define CRAS_INFO_STREAM_FILTER(filter, args)
#define CRAS_INFO_FILTER_NAMED(filter, name, ...)
#define CRAS_INFO_STREAM_FILTER_NAMED(filter, name, args)
#else
#define CRAS_LOG_INFO(logger, ...) CRAS_LOG((logger), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM(logger, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_INFO_NAMED(logger, name, ...) CRAS_LOG((logger), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_NAMED(logger, name, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_INFO_COND(logger, cond, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_COND(logger, cond, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_INFO_COND_NAMED(logger, cond, name, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_COND_NAMED(logger, cond, name, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_INFO_ONCE(logger, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_ONCE(logger, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_INFO_ONCE_NAMED(logger, name, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_ONCE_NAMED(logger, name, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_INFO_THROTTLE(logger, period, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_THROTTLE(logger, period, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_INFO_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_INFO_DELAYED_THROTTLE(logger, period, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_DELAYED_THROTTLE(logger, period, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_INFO_DELAYED_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_INFO_FILTER(logger, filter, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_FILTER(logger, filter, args) CRAS_LOG_STREAM_FILTER((logger), (filter), ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_INFO_FILTER_NAMED(logger, filter, name, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_INFO_STREAM_FILTER_NAMED(logger, filter, name, args) CRAS_LOG_STREAM_FILTER(getCrasLogger(), (filter), ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_INFO(...) CRAS_LOG_INFO(getCrasLogger(), __VA_ARGS__)
#define CRAS_INFO_STREAM(args) CRAS_LOG_INFO_STREAM(getCrasLogger(), args)
#define CRAS_INFO_NAMED(name, ...) CRAS_LOG_INFO_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_INFO_STREAM_NAMED(name, args) CRAS_LOG_INFO_STREAM_NAMED(getCrasLogger(), (name), args)
#define CRAS_INFO_COND(cond, ...) CRAS_LOG_INFO_COND(getCrasLogger(), (cond), __VA_ARGS__)
#define CRAS_INFO_STREAM_COND(cond, args) CRAS_LOG_INFO_STREAM_COND(getCrasLogger(), (cond), args)
#define CRAS_INFO_COND_NAMED(cond, name, ...) CRAS_LOG_INFO_COND_NAMED(getCrasLogger(), (cond), (name), __VA_ARGS__)
#define CRAS_INFO_STREAM_COND_NAMED(cond, name, args) CRAS_LOG_INFO_STREAM_COND_NAMED(getCrasLogger(), (cond), (name), args)  /* NOLINT */
#define CRAS_INFO_ONCE(...) CRAS_LOG_INFO_ONCE(getCrasLogger(), __VA_ARGS__)
#define CRAS_INFO_STREAM_ONCE(args) CRAS_LOG_INFO_STREAM_ONCE(getCrasLogger(), args)
#define CRAS_INFO_ONCE_NAMED(name, ...) CRAS_LOG_INFO_ONCE_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_INFO_STREAM_ONCE_NAMED(name, args) CRAS_LOG_INFO_STREAM_ONCE_NAMED(getCrasLogger(), (name), args)
#define CRAS_INFO_THROTTLE(period, ...) CRAS_LOG_INFO_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_INFO_STREAM_THROTTLE(period, args) CRAS_LOG_INFO_STREAM_THROTTLE(getCrasLogger(), (period), args)
#define CRAS_INFO_THROTTLE_NAMED(period, name, ...) CRAS_LOG_INFO_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_INFO_STREAM_THROTTLE_NAMED(period, name, args) CRAS_LOG_INFO_STREAM_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_INFO_DELAYED_THROTTLE(period, ...) CRAS_LOG_INFO_DELAYED_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_INFO_STREAM_DELAYED_THROTTLE(period, args) CRAS_LOG_INFO_STREAM_DELAYED_THROTTLE(getCrasLogger(), (period), args)  /* NOLINT */
#define CRAS_INFO_DELAYED_THROTTLE_NAMED(period, name, ...) CRAS_LOG_INFO_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_INFO_STREAM_DELAYED_THROTTLE_NAMED(period, name, args) CRAS_LOG_INFO_STREAM_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_INFO_FILTER(filter, ...) CRAS_LOG_INFO_FILTER(getCrasLogger(), (filter), __VA_ARGS__)
#define CRAS_INFO_STREAM_FILTER(filter, args) CRAS_LOG_INFO_STREAM_FILTER(getCrasLogger(), (filter), args)
#define CRAS_INFO_FILTER_NAMED(filter, name, ...) CRAS_LOG_INFO_FILTER_NAMED(getCrasLogger(), (filter), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_INFO_STREAM_FILTER_NAMED(filter, name, args) CRAS_LOG_INFO_STREAM_FILTER_NAMED(getCrasLogger(), (filter), (name), args)  /* NOLINT */
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_WARN)
#define CRAS_LOG_WARN(logger, ...)
#define CRAS_LOG_WARN_STREAM(logger, args)
#define CRAS_LOG_WARN_NAMED(logger, name, ...)
#define CRAS_LOG_WARN_STREAM_NAMED(logger, name, args)
#define CRAS_LOG_WARN_COND(logger, cond, ...)
#define CRAS_LOG_WARN_STREAM_COND(logger, cond, args)
#define CRAS_LOG_WARN_COND_NAMED(logger, cond, name, ...)
#define CRAS_LOG_WARN_STREAM_COND_NAMED(logger, cond, name, args)
#define CRAS_LOG_WARN_ONCE(logger, ...)
#define CRAS_LOG_WARN_STREAM_ONCE(logger, args)
#define CRAS_LOG_WARN_ONCE_NAMED(logger, name, ...)
#define CRAS_LOG_WARN_STREAM_ONCE_NAMED(logger, name, args)
#define CRAS_LOG_WARN_THROTTLE(logger, period, ...)
#define CRAS_LOG_WARN_STREAM_THROTTLE(logger, period, args)
#define CRAS_LOG_WARN_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_WARN_STREAM_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_WARN_DELAYED_THROTTLE(logger, period, ...)
#define CRAS_LOG_WARN_STREAM_DELAYED_THROTTLE(logger, period, args)
#define CRAS_LOG_WARN_DELAYED_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_WARN_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_WARN_FILTER(logger, filter, ...)
#define CRAS_LOG_WARN_STREAM_FILTER(logger, filter, args)
#define CRAS_LOG_WARN_FILTER_NAMED(logger, filter, name, ...)
#define CRAS_LOG_WARN_STREAM_FILTER_NAMED(logger, filter, name, args)
#define CRAS_WARN(...)
#define CRAS_WARN_STREAM(args)
#define CRAS_WARN_NAMED(name, ...)
#define CRAS_WARN_STREAM_NAMED(name, args)
#define CRAS_WARN_COND(cond, ...)
#define CRAS_WARN_STREAM_COND(cond, args)
#define CRAS_WARN_COND_NAMED(cond, name, ...)
#define CRAS_WARN_STREAM_COND_NAMED(cond, name, args)
#define CRAS_WARN_ONCE(...)
#define CRAS_WARN_STREAM_ONCE(args)
#define CRAS_WARN_ONCE_NAMED(name, ...)
#define CRAS_WARN_STREAM_ONCE_NAMED(name, args)
#define CRAS_WARN_THROTTLE(period, ...)
#define CRAS_WARN_STREAM_THROTTLE(period, args)
#define CRAS_WARN_THROTTLE_NAMED(period, name, ...)
#define CRAS_WARN_STREAM_THROTTLE_NAMED(period, name, args)
#define CRAS_WARN_DELAYED_THROTTLE(period, ...)
#define CRAS_WARN_STREAM_DELAYED_THROTTLE(period, args)
#define CRAS_WARN_DELAYED_THROTTLE_NAMED(period, name, ...)
#define CRAS_WARN_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)
#define CRAS_WARN_FILTER(filter, ...)
#define CRAS_WARN_STREAM_FILTER(filter, args)
#define CRAS_WARN_FILTER_NAMED(filter, name, ...)
#define CRAS_WARN_STREAM_FILTER_NAMED(filter, name, args)
#else
#define CRAS_LOG_WARN(logger, ...) CRAS_LOG((logger), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM(logger, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_WARN_NAMED(logger, name, ...) CRAS_LOG((logger), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_NAMED(logger, name, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_WARN_COND(logger, cond, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_COND(logger, cond, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_WARN_COND_NAMED(logger, cond, name, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_COND_NAMED(logger, cond, name, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_WARN_ONCE(logger, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_ONCE(logger, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_WARN_ONCE_NAMED(logger, name, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_ONCE_NAMED(logger, name, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_WARN_THROTTLE(logger, period, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_THROTTLE(logger, period, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_WARN_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_WARN_DELAYED_THROTTLE(logger, period, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_DELAYED_THROTTLE(logger, period, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_WARN_DELAYED_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_WARN_FILTER(logger, filter, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_FILTER(logger, filter, args) CRAS_LOG_STREAM_FILTER((logger), (filter), ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_WARN_FILTER_NAMED(logger, filter, name, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_WARN_STREAM_FILTER_NAMED(logger, filter, name, args) CRAS_LOG_STREAM_FILTER(getCrasLogger(), (filter), ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_WARN(...) CRAS_LOG_WARN(getCrasLogger(), __VA_ARGS__)
#define CRAS_WARN_STREAM(args) CRAS_LOG_WARN_STREAM(getCrasLogger(), args)
#define CRAS_WARN_NAMED(name, ...) CRAS_LOG_WARN_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_WARN_STREAM_NAMED(name, args) CRAS_LOG_WARN_STREAM_NAMED(getCrasLogger(), (name), args)
#define CRAS_WARN_COND(cond, ...) CRAS_LOG_WARN_COND(getCrasLogger(), (cond), __VA_ARGS__)
#define CRAS_WARN_STREAM_COND(cond, args) CRAS_LOG_WARN_STREAM_COND(getCrasLogger(), (cond), args)
#define CRAS_WARN_COND_NAMED(cond, name, ...) CRAS_LOG_WARN_COND_NAMED(getCrasLogger(), (cond), (name), __VA_ARGS__)
#define CRAS_WARN_STREAM_COND_NAMED(cond, name, args) CRAS_LOG_WARN_STREAM_COND_NAMED(getCrasLogger(), (cond), (name), args)  /* NOLINT */
#define CRAS_WARN_ONCE(...) CRAS_LOG_WARN_ONCE(getCrasLogger(), __VA_ARGS__)
#define CRAS_WARN_STREAM_ONCE(args) CRAS_LOG_WARN_STREAM_ONCE(getCrasLogger(), args)
#define CRAS_WARN_ONCE_NAMED(name, ...) CRAS_LOG_WARN_ONCE_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_WARN_STREAM_ONCE_NAMED(name, args) CRAS_LOG_WARN_STREAM_ONCE_NAMED(getCrasLogger(), (name), args)
#define CRAS_WARN_THROTTLE(period, ...) CRAS_LOG_WARN_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_WARN_STREAM_THROTTLE(period, args) CRAS_LOG_WARN_STREAM_THROTTLE(getCrasLogger(), (period), args)
#define CRAS_WARN_THROTTLE_NAMED(period, name, ...) CRAS_LOG_WARN_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_WARN_STREAM_THROTTLE_NAMED(period, name, args) CRAS_LOG_WARN_STREAM_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_WARN_DELAYED_THROTTLE(period, ...) CRAS_LOG_WARN_DELAYED_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_WARN_STREAM_DELAYED_THROTTLE(period, args) CRAS_LOG_WARN_STREAM_DELAYED_THROTTLE(getCrasLogger(), (period), args)  /* NOLINT */
#define CRAS_WARN_DELAYED_THROTTLE_NAMED(period, name, ...) CRAS_LOG_WARN_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_WARN_STREAM_DELAYED_THROTTLE_NAMED(period, name, args) CRAS_LOG_WARN_STREAM_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_WARN_FILTER(filter, ...) CRAS_LOG_WARN_FILTER(getCrasLogger(), (filter), __VA_ARGS__)
#define CRAS_WARN_STREAM_FILTER(filter, args) CRAS_LOG_WARN_STREAM_FILTER(getCrasLogger(), (filter), args)
#define CRAS_WARN_FILTER_NAMED(filter, name, ...) CRAS_LOG_WARN_FILTER_NAMED(getCrasLogger(), (filter), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_WARN_STREAM_FILTER_NAMED(filter, name, args) CRAS_LOG_WARN_STREAM_FILTER_NAMED(getCrasLogger(), (filter), (name), args)  /* NOLINT */
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_ERROR)
#define CRAS_LOG_ERROR(logger, ...)
#define CRAS_LOG_ERROR_STREAM(logger, args)
#define CRAS_LOG_ERROR_NAMED(logger, name, ...)
#define CRAS_LOG_ERROR_STREAM_NAMED(logger, name, args)
#define CRAS_LOG_ERROR_COND(logger, cond, ...)
#define CRAS_LOG_ERROR_STREAM_COND(logger, cond, args)
#define CRAS_LOG_ERROR_COND_NAMED(logger, cond, name, ...)
#define CRAS_LOG_ERROR_STREAM_COND_NAMED(logger, cond, name, args)
#define CRAS_LOG_ERROR_ONCE(logger, ...)
#define CRAS_LOG_ERROR_STREAM_ONCE(logger, args)
#define CRAS_LOG_ERROR_ONCE_NAMED(logger, name, ...)
#define CRAS_LOG_ERROR_STREAM_ONCE_NAMED(logger, name, args)
#define CRAS_LOG_ERROR_THROTTLE(logger, period, ...)
#define CRAS_LOG_ERROR_STREAM_THROTTLE(logger, period, args)
#define CRAS_LOG_ERROR_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_ERROR_STREAM_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_ERROR_DELAYED_THROTTLE(logger, period, ...)
#define CRAS_LOG_ERROR_STREAM_DELAYED_THROTTLE(logger, period, args)
#define CRAS_LOG_ERROR_DELAYED_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_ERROR_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_ERROR_FILTER(logger, filter, ...)
#define CRAS_LOG_ERROR_STREAM_FILTER(logger, filter, args)
#define CRAS_LOG_ERROR_FILTER_NAMED(logger, filter, name, ...)
#define CRAS_LOG_ERROR_STREAM_FILTER_NAMED(logger, filter, name, args)
#define CRAS_ERROR(...)
#define CRAS_ERROR_STREAM(args)
#define CRAS_ERROR_NAMED(name, ...)
#define CRAS_ERROR_STREAM_NAMED(name, args)
#define CRAS_ERROR_COND(cond, ...)
#define CRAS_ERROR_STREAM_COND(cond, args)
#define CRAS_ERROR_COND_NAMED(cond, name, ...)
#define CRAS_ERROR_STREAM_COND_NAMED(cond, name, args)
#define CRAS_ERROR_ONCE(...)
#define CRAS_ERROR_STREAM_ONCE(args)
#define CRAS_ERROR_ONCE_NAMED(name, ...)
#define CRAS_ERROR_STREAM_ONCE_NAMED(name, args)
#define CRAS_ERROR_THROTTLE(period, ...)
#define CRAS_ERROR_STREAM_THROTTLE(period, args)
#define CRAS_ERROR_THROTTLE_NAMED(period, name, ...)
#define CRAS_ERROR_STREAM_THROTTLE_NAMED(period, name, args)
#define CRAS_ERROR_DELAYED_THROTTLE(period, ...)
#define CRAS_ERROR_STREAM_DELAYED_THROTTLE(period, args)
#define CRAS_ERROR_DELAYED_THROTTLE_NAMED(period, name, ...)
#define CRAS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)
#define CRAS_ERROR_FILTER(filter, ...)
#define CRAS_ERROR_STREAM_FILTER(filter, args)
#define CRAS_ERROR_FILTER_NAMED(filter, name, ...)
#define CRAS_ERROR_STREAM_FILTER_NAMED(filter, name, args)
#else
#define CRAS_LOG_ERROR(logger, ...) CRAS_LOG((logger), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM(logger, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_ERROR_NAMED(logger, name, ...) CRAS_LOG((logger), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_NAMED(logger, name, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_ERROR_COND(logger, cond, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_COND(logger, cond, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_ERROR_COND_NAMED(logger, cond, name, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_COND_NAMED(logger, cond, name, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_ERROR_ONCE(logger, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_ONCE(logger, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_ERROR_ONCE_NAMED(logger, name, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_ONCE_NAMED(logger, name, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_ERROR_THROTTLE(logger, period, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_THROTTLE(logger, period, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_ERROR_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_ERROR_DELAYED_THROTTLE(logger, period, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_DELAYED_THROTTLE(logger, period, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_ERROR_DELAYED_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_ERROR_FILTER(logger, filter, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_FILTER(logger, filter, args) CRAS_LOG_STREAM_FILTER((logger), (filter), ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_ERROR_FILTER_NAMED(logger, filter, name, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_ERROR_STREAM_FILTER_NAMED(logger, filter, name, args) CRAS_LOG_STREAM_FILTER(getCrasLogger(), (filter), ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_ERROR(...) CRAS_LOG_ERROR(getCrasLogger(), __VA_ARGS__)
#define CRAS_ERROR_STREAM(args) CRAS_LOG_ERROR_STREAM(getCrasLogger(), args)
#define CRAS_ERROR_NAMED(name, ...) CRAS_LOG_ERROR_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_ERROR_STREAM_NAMED(name, args) CRAS_LOG_ERROR_STREAM_NAMED(getCrasLogger(), (name), args)
#define CRAS_ERROR_COND(cond, ...) CRAS_LOG_ERROR_COND(getCrasLogger(), (cond), __VA_ARGS__)
#define CRAS_ERROR_STREAM_COND(cond, args) CRAS_LOG_ERROR_STREAM_COND(getCrasLogger(), (cond), args)
#define CRAS_ERROR_COND_NAMED(cond, name, ...) CRAS_LOG_ERROR_COND_NAMED(getCrasLogger(), (cond), (name), __VA_ARGS__)
#define CRAS_ERROR_STREAM_COND_NAMED(cond, name, args) CRAS_LOG_ERROR_STREAM_COND_NAMED(getCrasLogger(), (cond), (name), args)  /* NOLINT */
#define CRAS_ERROR_ONCE(...) CRAS_LOG_ERROR_ONCE(getCrasLogger(), __VA_ARGS__)
#define CRAS_ERROR_STREAM_ONCE(args) CRAS_LOG_ERROR_STREAM_ONCE(getCrasLogger(), args)
#define CRAS_ERROR_ONCE_NAMED(name, ...) CRAS_LOG_ERROR_ONCE_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_ERROR_STREAM_ONCE_NAMED(name, args) CRAS_LOG_ERROR_STREAM_ONCE_NAMED(getCrasLogger(), (name), args)
#define CRAS_ERROR_THROTTLE(period, ...) CRAS_LOG_ERROR_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_ERROR_STREAM_THROTTLE(period, args) CRAS_LOG_ERROR_STREAM_THROTTLE(getCrasLogger(), (period), args)
#define CRAS_ERROR_THROTTLE_NAMED(period, name, ...) CRAS_LOG_ERROR_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_ERROR_STREAM_THROTTLE_NAMED(period, name, args) CRAS_LOG_ERROR_STREAM_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_ERROR_DELAYED_THROTTLE(period, ...) CRAS_LOG_ERROR_DELAYED_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_ERROR_STREAM_DELAYED_THROTTLE(period, args) CRAS_LOG_ERROR_STREAM_DELAYED_THROTTLE(getCrasLogger(), (period), args)  /* NOLINT */
#define CRAS_ERROR_DELAYED_THROTTLE_NAMED(period, name, ...) CRAS_LOG_ERROR_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(period, name, args) CRAS_LOG_ERROR_STREAM_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_ERROR_FILTER(filter, ...) CRAS_LOG_ERROR_FILTER(getCrasLogger(), (filter), __VA_ARGS__)
#define CRAS_ERROR_STREAM_FILTER(filter, args) CRAS_LOG_ERROR_STREAM_FILTER(getCrasLogger(), (filter), args)
#define CRAS_ERROR_FILTER_NAMED(filter, name, ...) CRAS_LOG_ERROR_FILTER_NAMED(getCrasLogger(), (filter), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_ERROR_STREAM_FILTER_NAMED(filter, name, args) CRAS_LOG_ERROR_STREAM_FILTER_NAMED(getCrasLogger(), (filter), (name), args)  /* NOLINT */
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_FATAL)
#define CRAS_LOG_FATAL(logger, ...)
#define CRAS_LOG_FATAL_STREAM(logger, args)
#define CRAS_LOG_FATAL_NAMED(logger, name, ...)
#define CRAS_LOG_FATAL_STREAM_NAMED(logger, name, args)
#define CRAS_LOG_FATAL_COND(logger, cond, ...)
#define CRAS_LOG_FATAL_STREAM_COND(logger, cond, args)
#define CRAS_LOG_FATAL_COND_NAMED(logger, cond, name, ...)
#define CRAS_LOG_FATAL_STREAM_COND_NAMED(logger, cond, name, args)
#define CRAS_LOG_FATAL_ONCE(logger, ...)
#define CRAS_LOG_FATAL_STREAM_ONCE(logger, args)
#define CRAS_LOG_FATAL_ONCE_NAMED(logger, name, ...)
#define CRAS_LOG_FATAL_STREAM_ONCE_NAMED(logger, name, args)
#define CRAS_LOG_FATAL_THROTTLE(logger, period, ...)
#define CRAS_LOG_FATAL_STREAM_THROTTLE(logger, period, args)
#define CRAS_LOG_FATAL_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_FATAL_STREAM_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_FATAL_DELAYED_THROTTLE(logger, period, ...)
#define CRAS_LOG_FATAL_STREAM_DELAYED_THROTTLE(logger, period, args)
#define CRAS_LOG_FATAL_DELAYED_THROTTLE_NAMED(logger, period, name, ...)
#define CRAS_LOG_FATAL_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args)
#define CRAS_LOG_FATAL_FILTER(logger, filter, ...)
#define CRAS_LOG_FATAL_STREAM_FILTER(logger, filter, args)
#define CRAS_LOG_FATAL_FILTER_NAMED(logger, filter, name, ...)
#define CRAS_LOG_FATAL_STREAM_FILTER_NAMED(logger, filter, name, args)
#define CRAS_FATAL(...)
#define CRAS_FATAL_STREAM(args)
#define CRAS_FATAL_NAMED(name, ...)
#define CRAS_FATAL_STREAM_NAMED(name, args)
#define CRAS_FATAL_COND(cond, ...)
#define CRAS_FATAL_STREAM_COND(cond, args)
#define CRAS_FATAL_COND_NAMED(cond, name, ...)
#define CRAS_FATAL_STREAM_COND_NAMED(cond, name, args)
#define CRAS_FATAL_ONCE(...)
#define CRAS_FATAL_STREAM_ONCE(args)
#define CRAS_FATAL_ONCE_NAMED(name, ...)
#define CRAS_FATAL_STREAM_ONCE_NAMED(name, args)
#define CRAS_FATAL_THROTTLE(period, ...)
#define CRAS_FATAL_STREAM_THROTTLE(period, args)
#define CRAS_FATAL_THROTTLE_NAMED(period, name, ...)
#define CRAS_FATAL_STREAM_THROTTLE_NAMED(period, name, args)
#define CRAS_FATAL_DELAYED_THROTTLE(period, ...)
#define CRAS_FATAL_STREAM_DELAYED_THROTTLE(period, args)
#define CRAS_FATAL_DELAYED_THROTTLE_NAMED(period, name, ...)
#define CRAS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)
#define CRAS_FATAL_FILTER(filter, ...)
#define CRAS_FATAL_STREAM_FILTER(filter, args)
#define CRAS_FATAL_FILTER_NAMED(filter, name, ...)
#define CRAS_FATAL_STREAM_FILTER_NAMED(filter, name, args)
#else
#define CRAS_LOG_FATAL(logger, ...) CRAS_LOG((logger), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM(logger, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_FATAL_NAMED(logger, name, ...) CRAS_LOG((logger), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_NAMED(logger, name, args) CRAS_LOG_STREAM((logger), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_FATAL_COND(logger, cond, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_COND(logger, cond, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_FATAL_COND_NAMED(logger, cond, name, ...) CRAS_LOG_COND((logger), (cond), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_COND_NAMED(logger, cond, name, args) CRAS_LOG_STREAM_COND((logger), (cond), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_FATAL_ONCE(logger, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_ONCE(logger, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_FATAL_ONCE_NAMED(logger, name, ...) CRAS_LOG_ONCE((logger), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_ONCE_NAMED(logger, name, args) CRAS_LOG_STREAM_ONCE((logger), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_FATAL_THROTTLE(logger, period, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_THROTTLE(logger, period, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_FATAL_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_THROTTLE((logger), (period), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_THROTTLE((logger), (period), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_FATAL_DELAYED_THROTTLE(logger, period, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_DELAYED_THROTTLE(logger, period, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_FATAL_DELAYED_THROTTLE_NAMED(logger, period, name, ...) CRAS_LOG_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_DELAYED_THROTTLE_NAMED(logger, period, name, args) CRAS_LOG_STREAM_DELAYED_THROTTLE((logger), (period), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_LOG_FATAL_FILTER(logger, filter, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_FILTER(logger, filter, args) CRAS_LOG_STREAM_FILTER((logger), (filter), ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)  /* NOLINT */
#define CRAS_LOG_FATAL_FILTER_NAMED(logger, filter, name, ...) CRAS_LOG_FILTER((logger), (filter), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_LOG_FATAL_STREAM_FILTER_NAMED(logger, filter, name, args) CRAS_LOG_STREAM_FILTER(getCrasLogger(), (filter), ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + (name), args)  /* NOLINT */
#define CRAS_FATAL(...) CRAS_LOG_FATAL(getCrasLogger(), __VA_ARGS__)
#define CRAS_FATAL_STREAM(args) CRAS_LOG_FATAL_STREAM(getCrasLogger(), args)
#define CRAS_FATAL_NAMED(name, ...) CRAS_LOG_FATAL_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_FATAL_STREAM_NAMED(name, args) CRAS_LOG_FATAL_STREAM_NAMED(getCrasLogger(), (name), args)
#define CRAS_FATAL_COND(cond, ...) CRAS_LOG_FATAL_COND(getCrasLogger(), (cond), __VA_ARGS__)
#define CRAS_FATAL_STREAM_COND(cond, args) CRAS_LOG_FATAL_STREAM_COND(getCrasLogger(), (cond), args)
#define CRAS_FATAL_COND_NAMED(cond, name, ...) CRAS_LOG_FATAL_COND_NAMED(getCrasLogger(), (cond), (name), __VA_ARGS__)
#define CRAS_FATAL_STREAM_COND_NAMED(cond, name, args) CRAS_LOG_FATAL_STREAM_COND_NAMED(getCrasLogger(), (cond), (name), args)  /* NOLINT */
#define CRAS_FATAL_ONCE(...) CRAS_LOG_FATAL_ONCE(getCrasLogger(), __VA_ARGS__)
#define CRAS_FATAL_STREAM_ONCE(args) CRAS_LOG_FATAL_STREAM_ONCE(getCrasLogger(), args)
#define CRAS_FATAL_ONCE_NAMED(name, ...) CRAS_LOG_FATAL_ONCE_NAMED(getCrasLogger(), (name), __VA_ARGS__)
#define CRAS_FATAL_STREAM_ONCE_NAMED(name, args) CRAS_LOG_FATAL_STREAM_ONCE_NAMED(getCrasLogger(), (name), args)
#define CRAS_FATAL_THROTTLE(period, ...) CRAS_LOG_FATAL_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_FATAL_STREAM_THROTTLE(period, args) CRAS_LOG_FATAL_STREAM_THROTTLE(getCrasLogger(), (period), args)
#define CRAS_FATAL_THROTTLE_NAMED(period, name, ...) CRAS_LOG_FATAL_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_FATAL_STREAM_THROTTLE_NAMED(period, name, args) CRAS_LOG_FATAL_STREAM_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_FATAL_DELAYED_THROTTLE(period, ...) CRAS_LOG_FATAL_DELAYED_THROTTLE(getCrasLogger(), (period), __VA_ARGS__)
#define CRAS_FATAL_STREAM_DELAYED_THROTTLE(period, args) CRAS_LOG_FATAL_STREAM_DELAYED_THROTTLE(getCrasLogger(), (period), args)  /* NOLINT */
#define CRAS_FATAL_DELAYED_THROTTLE_NAMED(period, name, ...) CRAS_LOG_FATAL_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(period, name, args) CRAS_LOG_FATAL_STREAM_DELAYED_THROTTLE_NAMED(getCrasLogger(), (period), (name), args)  /* NOLINT */
#define CRAS_FATAL_FILTER(filter, ...) CRAS_LOG_FATAL_FILTER(getCrasLogger(), (filter), __VA_ARGS__)
#define CRAS_FATAL_STREAM_FILTER(filter, args) CRAS_LOG_FATAL_STREAM_FILTER(getCrasLogger(), (filter), args)
#define CRAS_FATAL_FILTER_NAMED(filter, name, ...) CRAS_LOG_FATAL_FILTER_NAMED(getCrasLogger(), (filter), (name), __VA_ARGS__)  /* NOLINT */
#define CRAS_FATAL_STREAM_FILTER_NAMED(filter, name, args) CRAS_LOG_FATAL_STREAM_FILTER_NAMED(getCrasLogger(), (filter), (name), args)  /* NOLINT */
#endif
