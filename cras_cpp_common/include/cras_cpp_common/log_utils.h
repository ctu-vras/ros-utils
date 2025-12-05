#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS logging helpers.
 * \author Martin Pecka
 */

#include <cras_cpp_common/string_utils.hpp>
#include <rcutils/logging.h>

namespace cras
{

/**
 * \brief Convert the given RCL logging level to `rcl_interfaces::msg::Log` level constant.
 * \param[in] rosLevel The RCL logging level.
 * \return The `rosgraph_msgs::Log` level constant.
 */
int8_t logLevelToMsgLevel(RCUTILS_LOG_SEVERITY rosLevel);

/**
 * \brief Convert the given `rcl_interfaces::msg::Log` level constant to a given RCL
 * \param[in] msgLevel A `rosgraph_msgs::Log` level constant.
 * \return The rosconsole logging level.
 */
RCUTILS_LOG_SEVERITY msgLevelToLogLevel(uint8_t msgLevel);

}
