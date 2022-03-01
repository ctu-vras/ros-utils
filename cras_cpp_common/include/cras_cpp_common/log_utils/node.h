#pragma once

/**
 * \file
 * \brief Log helper redirecting the logging calls to ROS_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <cras_cpp_common/log_utils.h>

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class NodeLogHelper : public ::cras::LogHelper
{
protected:
  void printDebug(const ::std::string& text) const override;
  void printInfo(const ::std::string& text) const override;
  void printWarn(const ::std::string& text) const override;
  void printError(const ::std::string& text) const override;
  void printFatal(const ::std::string& text) const override;
};

}