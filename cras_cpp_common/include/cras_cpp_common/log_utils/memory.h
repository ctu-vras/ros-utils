#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Log helper storing all messages in memory.
 * \author Martin Pecka
 */

#include <list>
#include <string>

#include <ros/console.h>
#include <ros/time.h>
#include <rosgraph_msgs/Log.h>

#include <cras_cpp_common/log_utils.h>

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class MemoryLogHelper : public ::cras::LogHelper
{
public:
  void initializeImpl() const override;

  void initializeLogLocationImpl(::ros::console::LogLocation* loc, const ::std::string& name,
    ::ros::console::Level level) const override;

  void setLogLocationLevel(::ros::console::LogLocation* loc, ::ros::console::Level level) const override;

  void checkLogLocationEnabled(::ros::console::LogLocation* loc) const override;

  void logString(void* logger, ::ros::console::Level level, const std::string& str, const char* file, uint32_t line,
    const char* function) const override;

  /**
   * \brief Return all messages logged so far.
   * \return The messages.
   */
  const ::std::list<::rosgraph_msgs::Log>& getMessages() const;

  /**
   * \brief Delete all messages logged so far.
   */
  void clear() const;

protected:
  mutable ::std::list<::rosgraph_msgs::Log> messages;  //!< \brief The list of logged messages.
  mutable ::std::list<::std::string> loggerNames;  //!< \brief Cache of names of known loggers. Do not ever erase items.
};

}
