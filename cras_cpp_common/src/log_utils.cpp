// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS logging helpers.
 * \author Martin Pecka
 */

#include <rcl_interfaces/msg/log.hpp>

#include <cras_cpp_common/log_utils.h>

namespace cras
{

int8_t logLevelToMsgLevel(const RCUTILS_LOG_SEVERITY rosLevel)
{
  switch (rosLevel)
  {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      return rcl_interfaces::msg::Log::DEBUG;
    case RCUTILS_LOG_SEVERITY_INFO:
      return rcl_interfaces::msg::Log::INFO;
    case RCUTILS_LOG_SEVERITY_WARN:
      return rcl_interfaces::msg::Log::WARN;
    case RCUTILS_LOG_SEVERITY_ERROR:
      return rcl_interfaces::msg::Log::ERROR;
    default:
      return rcl_interfaces::msg::Log::FATAL;
  }
}

RCUTILS_LOG_SEVERITY msgLevelToLogLevel(const uint8_t msgLevel)
{
  switch (msgLevel)
  {
    case rcl_interfaces::msg::Log::DEBUG:
      return RCUTILS_LOG_SEVERITY_DEBUG;
    case rcl_interfaces::msg::Log::INFO:
      return RCUTILS_LOG_SEVERITY_INFO;
    case rcl_interfaces::msg::Log::WARN:
      return RCUTILS_LOG_SEVERITY_WARN;
    case rcl_interfaces::msg::Log::ERROR:
      return RCUTILS_LOG_SEVERITY_ERROR;
    default:
      return RCUTILS_LOG_SEVERITY_FATAL;
  }
}

}
