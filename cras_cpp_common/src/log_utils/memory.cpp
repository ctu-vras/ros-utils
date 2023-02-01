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

#include <cras_cpp_common/log_utils/memory.h>

namespace cras
{

void MemoryLogHelper::initializeImpl() const
{
}

void MemoryLogHelper::initializeLogLocationImpl(ros::console::LogLocation* loc, const std::string& name,
  ros::console::Level level) const
{
  // Store the logger name in the logger_ variable. It has to be a pointer, so we store a list of known logger names
  // and return a pointer to an element in this list.
  this->loggerNames.emplace_back(name);
  loc->logger_ = &this->loggerNames.back();
  loc->logger_enabled_ = true;
  loc->level_ = level;
  loc->initialized_ = true;
}

void MemoryLogHelper::setLogLocationLevel(ros::console::LogLocation* loc, ros::console::Level level) const
{
  loc->level_ = level;
}

void MemoryLogHelper::checkLogLocationEnabled(ros::console::LogLocation* loc) const
{
  loc->logger_enabled_ = true;
}

void MemoryLogHelper::logString(void* logger, ros::console::Level level, const std::string& str, const char* file,
  uint32_t line, const char* function) const
{
  const auto& loggerName = (logger != nullptr) ?
    *reinterpret_cast<std::string*>(logger) : ROSCONSOLE_DEFAULT_NAME; // NOLINT

  auto message = rosgraph_msgs::Log{};
  message.header.stamp = this->getTimeNow();
  message.name = loggerName;
  message.level = cras::logLevelToRosgraphMsgLevel(level);
  message.msg = str;
  message.file = file;
  message.function = function;
  message.line = line;
  this->messages.push_back(message);
}

const std::list<rosgraph_msgs::Log>& MemoryLogHelper::getMessages() const
{
  return this->messages;
}

void MemoryLogHelper::clear() const
{
  this->messages.clear();
}

}
