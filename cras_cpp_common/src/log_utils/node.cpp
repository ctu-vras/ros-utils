/**
 * \file
 * \brief Log helper redirecting the logging calls to ROS_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/log_utils/node.h>

using namespace ::cras;

void NodeLogHelper::printDebug(const ::std::string& text) const
{
  ROS_DEBUG("%s", text.c_str());
}

void NodeLogHelper::printInfo(const ::std::string& text) const
{
  ROS_INFO("%s", text.c_str());
}

void NodeLogHelper::printWarn(const ::std::string& text) const
{
  ROS_WARN("%s", text.c_str());
}

void NodeLogHelper::printError(const ::std::string& text) const
{
  ROS_ERROR("%s", text.c_str());
}

void NodeLogHelper::printFatal(const ::std::string& text) const
{
  ROS_FATAL("%s", text.c_str());
}