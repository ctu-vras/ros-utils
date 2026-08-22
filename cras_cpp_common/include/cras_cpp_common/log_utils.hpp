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

namespace cras {

/**
 * \brief Convert the given RCL logging level to `rcl_interfaces::msg::Log` level constant.
 * \param[in] ros_level The RCL logging level.
 * \return The `rosgraph_msgs::Log` level constant.
 */
int8_t logLevelToMsgLevel(RCUTILS_LOG_SEVERITY ros_level);

/**
 * \brief Convert the given `rcl_interfaces::msg::Log` level constant to a given RCL
 * \param[in] msg_level A `rosgraph_msgs::Log` level constant.
 * \return The rosconsole logging level.
 */
RCUTILS_LOG_SEVERITY msgLevelToLogLevel(uint8_t msg_level);

}  // namespace cras
