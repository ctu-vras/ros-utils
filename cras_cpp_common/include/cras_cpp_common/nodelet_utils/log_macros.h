/**
 * \file
 * \brief This file adds the missing DELAYED_THROTTLE logging macros for nodelets.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#define NODELET_DEBUG_DELAYED_THROTTLE(rate, ...) ROS_DEBUG_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_INFO_DELAYED_THROTTLE(rate, ...) ROS_INFO_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_WARN_DELAYED_THROTTLE(rate, ...) ROS_WARN_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_ERROR_DELAYED_THROTTLE(rate, ...) ROS_ERROR_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_FATAL_DELAYED_THROTTLE(rate, ...) ROS_FATAL_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)

#define NODELET_DEBUG_STREAM_DELAYED_THROTTLE(rate, ...) \
  ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM_DELAYED_THROTTLE(rate, ...) \
  ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM_DELAYED_THROTTLE(rate, ...) \
  ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_DELAYED_THROTTLE(rate, ...) \
  ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_FATAL_STREAM_DELAYED_THROTTLE(rate, ...) \
  ROS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
