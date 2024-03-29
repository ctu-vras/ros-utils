#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Log helper redirecting the logging calls to ROS_ macros.
 * \author Martin Pecka
 */

#include <cras_cpp_common/log_utils.h>

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class NodeLogHelper : public ::cras::RosconsoleLogHelper
{
};

}
