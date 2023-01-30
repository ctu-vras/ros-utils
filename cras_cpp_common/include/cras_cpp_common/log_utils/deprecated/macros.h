#pragma once

/**
 * \file
 * \brief ROS logging helpers deprecated macros. This is a transitional file helping the switch to the `CRAS_*` logging
 *        macros and it should not be used in new code. This header will be removed in a future release.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#define logDebug(...) setGlobalLogger(); CRAS_DEBUG(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugNamed(...) setGlobalLogger(); CRAS_DEBUG_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugCond(...) setGlobalLogger(); CRAS_DEBUG_COND(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugCondNamed(...) setGlobalLogger(); CRAS_DEBUG_COND_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugOnce(...) setGlobalLogger(); CRAS_DEBUG_ONCE(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugOnceNamed(...) setGlobalLogger(); CRAS_DEBUG_ONCE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugThrottle(...) setGlobalLogger(); CRAS_DEBUG_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugThrottleNamed(...) setGlobalLogger(); CRAS_DEBUG_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logDebugDelayedThrottle(...) setGlobalLogger(); CRAS_DEBUG_DELAYED_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logDebugDelayedThrottleNamed(...) setGlobalLogger(); CRAS_DEBUG_DELAYED_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logDebugFilter(...) setGlobalLogger(); CRAS_DEBUG_FILTER(__VA_ARGS__); restorePreviousCrasLogger();
#define logDebugFilterNamed(...) setGlobalLogger(); CRAS_DEBUG_FILTER_NAMED(__VA_ARGS__); restorePreviousCrasLogger();

#define logInfo(...) setGlobalLogger(); CRAS_INFO(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoNamed(...) setGlobalLogger(); CRAS_INFO_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoCond(...) setGlobalLogger(); CRAS_INFO_COND(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoCondNamed(...) setGlobalLogger(); CRAS_INFO_COND_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoOnce(...) setGlobalLogger(); CRAS_INFO_ONCE(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoOnceNamed(...) setGlobalLogger(); CRAS_INFO_ONCE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoThrottle(...) setGlobalLogger(); CRAS_INFO_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoThrottleNamed(...) setGlobalLogger(); CRAS_INFO_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoDelayedThrottle(...) setGlobalLogger(); CRAS_INFO_DELAYED_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logInfoDelayedThrottleNamed(...) setGlobalLogger(); CRAS_INFO_DELAYED_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logInfoFilter(...) setGlobalLogger(); CRAS_INFO_FILTER(__VA_ARGS__); restorePreviousCrasLogger();
#define logInfoFilterNamed(...) setGlobalLogger(); CRAS_INFO_FILTER_NAMED(__VA_ARGS__); restorePreviousCrasLogger();

#define logWarn(...) setGlobalLogger(); CRAS_WARN(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnNamed(...) setGlobalLogger(); CRAS_WARN_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnCond(...) setGlobalLogger(); CRAS_WARN_COND(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnCondNamed(...) setGlobalLogger(); CRAS_WARN_COND_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnOnce(...) setGlobalLogger(); CRAS_WARN_ONCE(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnOnceNamed(...) setGlobalLogger(); CRAS_WARN_ONCE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnThrottle(...) setGlobalLogger(); CRAS_WARN_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnThrottleNamed(...) setGlobalLogger(); CRAS_WARN_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnDelayedThrottle(...) setGlobalLogger(); CRAS_WARN_DELAYED_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logWarnDelayedThrottleNamed(...) setGlobalLogger(); CRAS_WARN_DELAYED_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logWarnFilter(...) setGlobalLogger(); CRAS_WARN_FILTER(__VA_ARGS__); restorePreviousCrasLogger();
#define logWarnFilterNamed(...) setGlobalLogger(); CRAS_WARN_FILTER_NAMED(__VA_ARGS__); restorePreviousCrasLogger();

#define logError(...) setGlobalLogger(); CRAS_ERROR(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorNamed(...) setGlobalLogger(); CRAS_ERROR_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorCond(...) setGlobalLogger(); CRAS_ERROR_COND(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorCondNamed(...) setGlobalLogger(); CRAS_ERROR_COND_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorOnce(...) setGlobalLogger(); CRAS_ERROR_ONCE(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorOnceNamed(...) setGlobalLogger(); CRAS_ERROR_ONCE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorThrottle(...) setGlobalLogger(); CRAS_ERROR_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorThrottleNamed(...) setGlobalLogger(); CRAS_ERROR_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logErrorDelayedThrottle(...) setGlobalLogger(); CRAS_ERROR_DELAYED_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logErrorDelayedThrottleNamed(...) setGlobalLogger(); CRAS_ERROR_DELAYED_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logErrorFilter(...) setGlobalLogger(); CRAS_ERROR_FILTER(__VA_ARGS__); restorePreviousCrasLogger();
#define logErrorFilterNamed(...) setGlobalLogger(); CRAS_ERROR_FILTER_NAMED(__VA_ARGS__); restorePreviousCrasLogger();

#define logFatal(...) setGlobalLogger(); CRAS_FATAL(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalNamed(...) setGlobalLogger(); CRAS_FATAL_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalCond(...) setGlobalLogger(); CRAS_FATAL_COND(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalCondNamed(...) setGlobalLogger(); CRAS_FATAL_COND_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalOnce(...) setGlobalLogger(); CRAS_FATAL_ONCE(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalOnceNamed(...) setGlobalLogger(); CRAS_FATAL_ONCE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalThrottle(...) setGlobalLogger(); CRAS_FATAL_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalThrottleNamed(...) setGlobalLogger(); CRAS_FATAL_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logFatalDelayedThrottle(...) setGlobalLogger(); CRAS_FATAL_DELAYED_THROTTLE(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logFatalDelayedThrottleNamed(...) setGlobalLogger(); CRAS_FATAL_DELAYED_THROTTLE_NAMED(__VA_ARGS__); restorePreviousCrasLogger();  /* NOLINT */
#define logFatalFilter(...) setGlobalLogger(); CRAS_FATAL_FILTER(__VA_ARGS__); restorePreviousCrasLogger();
#define logFatalFilterNamed(...) setGlobalLogger(); CRAS_FATAL_FILTER_NAMED(__VA_ARGS__); restorePreviousCrasLogger();
