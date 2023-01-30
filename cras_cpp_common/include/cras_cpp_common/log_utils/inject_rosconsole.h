/**
 * \file
 * \brief Include this file if you want to override the ROS_ or NODELET_ logging macros with CRAS logging macros. Type
 *        `CRAS_RESTORE_ROS_LOG` to get the original definitions back. Use this with care.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma push_macro("ROSCONSOLE_AUTOINIT")
#pragma push_macro("ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER")
#pragma push_macro("ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER")
#pragma push_macro("ROSCONSOLE_DEFINE_LOCATION")
#pragma push_macro("ROS_LOG_THROTTLE")
#pragma push_macro("ROS_LOG_STREAM_THROTTLE")
#pragma push_macro("ROS_LOG_DELAYED_THROTTLE")
#pragma push_macro("ROS_LOG_STREAM_DELAYED_THROTTLE")

#undef ROSCONSOLE_AUTOINIT
#undef ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER
#undef ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER
#undef ROSCONSOLE_DEFINE_LOCATION
#undef ROS_LOG_THROTTLE
#undef ROS_LOG_STREAM_THROTTLE
#undef ROS_LOG_DELAYED_THROTTLE
#undef ROS_LOG_STREAM_DELAYED_THROTTLE

#define ROSCONSOLE_AUTOINIT(...) CRASCONSOLE_AUTOINIT(getCrasLogger(), __VA_ARGS__)
#define ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(...) CRASCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(getCrasLogger(), __VA_ARGS__)  /* NOLINT */
#define ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(...) CRASCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(getCrasLogger(), __VA_ARGS__)  /* NOLINT */
#define ROSCONSOLE_DEFINE_LOCATION(...) CRASCONSOLE_DEFINE_LOCATION(getCrasLogger(), __VA_ARGS__)
#define ROS_LOG_THROTTLE(...) CRAS_LOG_THROTTLE(getCrasLogger(), __VA_ARGS__)
#define ROS_LOG_STREAM_THROTTLE(...) CRAS_LOG_STREAM_THROTTLE(getCrasLogger(), __VA_ARGS__)
#define ROS_LOG_DELAYED_THROTTLE(...) CRAS_LOG_DELAYED_THROTTLE(getCrasLogger(), __VA_ARGS__)
#define ROS_LOG_STREAM_DELAYED_THROTTLE(...) CRAS_LOG_STREAM_DELAYED_THROTTLE(getCrasLogger(), __VA_ARGS__)

#ifndef CRAS_RESTORE_ROS_LOG
/**
 * \brief Restore original `ROS_*` and `NODELET_*` macro definitions after they were altered by including
 *        inject_rosconsole.h.
 */
#define CRAS_RESTORE_ROS_LOG \
  _Pragma("pop_macro(\"ROSCONSOLE_AUTOINIT\")") \
  _Pragma("pop_macro(\"ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER\")") \
  _Pragma("pop_macro(\"ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER\")") \
  _Pragma("pop_macro(\"ROSCONSOLE_DEFINE_LOCATION\")") \
  _Pragma("pop_macro(\"ROS_LOG_THROTTLE\")") \
  _Pragma("pop_macro(\"ROS_LOG_STREAM_THROTTLE\")") \
  _Pragma("pop_macro(\"ROS_LOG_DELAYED_THROTTLE\")") \
  _Pragma("pop_macro(\"ROS_LOG_STREAM_DELAYED_THROTTLE\")")
#endif
