// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Log helper storing all messages in memory.
 * \author Martin Pecka
 */

#include <list>
#include <string>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/time.hpp>

namespace cras
{

static MemoryLoggingInterface* g_currentLoggingInterface {nullptr};

MemoryLoggingInterface::MemoryLoggingInterface(const std::string& name, const RCUTILS_LOG_SEVERITY severity)
  : name(name)
{
  this->prevLoggingInterface = g_currentLoggingInterface;
  g_currentLoggingInterface = this;

  auto ret = rcutils_logging_initialize();
  if (ret != RCUTILS_RET_OK)
    rclcpp::exceptions::throw_from_rcl_error(ret, "");

  this->prevLogLevel = static_cast<RCUTILS_LOG_SEVERITY>(rcutils_logging_get_default_logger_level());
  rcutils_logging_set_default_logger_level(severity);

  const auto handler = [](const rcutils_log_location_t* location, const int level,
    const char* name, const rcutils_time_point_value_t timestamp, const char* format, va_list* args)
  {
    if (g_currentLoggingInterface == nullptr)
      return;

    rcl_interfaces::msg::Log msg;
    msg.name = name;
    msg.file = location->file_name;
    msg.function = location->function_name;
    msg.line = location->line_number;
    msg.level = cras::logLevelToMsgLevel(static_cast<RCUTILS_LOG_SEVERITY>(level));
    msg.msg = cras::snprintf(format, *args);
    msg.stamp = rclcpp::Time(timestamp);
    g_currentLoggingInterface->addLogMessage(msg);
  };

  this->prevHandler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(handler);
}

MemoryLoggingInterface::~MemoryLoggingInterface()
{
  rcutils_logging_set_output_handler(this->prevHandler);
  rcutils_logging_set_default_logger_level(this->prevLogLevel);
  const auto _ = rcutils_logging_shutdown();
}

rclcpp::Logger MemoryLoggingInterface::get_logger() const
{
  return rclcpp::get_logger(this->name);
}

const char* MemoryLoggingInterface::get_logger_name() const
{
  return this->name.c_str();
}

void MemoryLoggingInterface::create_logger_services(
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
{
}

void MemoryLoggingInterface::addLogMessage(const rcl_interfaces::msg::Log& msg)
{
  this->messages.push_back(msg);
}

const std::list<rcl_interfaces::msg::Log>& MemoryLoggingInterface::getMessages() const
{
  return this->messages;
}

void MemoryLoggingInterface::clear()
{
  this->messages.clear();
}

}
