#pragma once

/**
 * \file
 * \brief Log helper redirecting the logging calls to ROS_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/log_utils.h>

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class NodeLogHelper : public ::cras::LogHelper
{
protected:
  void printDebug(const ::std::string& text) const override
	{
		ROS_DEBUG("%s", text.c_str());
	}
  void printInfo(const ::std::string& text) const override
	{
		ROS_INFO("%s", text.c_str());
	}
  void printWarn(const ::std::string& text) const override
	{
		ROS_WARN("%s", text.c_str());
	}
  void printError(const ::std::string& text) const override
	{
		ROS_ERROR("%s", text.c_str());
	}
  void printFatal(const ::std::string& text) const override
	{
		ROS_FATAL("%s", text.c_str());
	}
};

}