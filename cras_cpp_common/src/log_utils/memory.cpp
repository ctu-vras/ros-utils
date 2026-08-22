// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Log helper storing all messages in memory.
 * \author Martin Pecka
 */

#include <list>
#include <string>

#include <cras_cpp_common/log_utils.hpp>
#include <cras_cpp_common/log_utils/memory.hpp>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/version.h>

namespace cras {

static MemoryLoggingInterface* g_current_logging_interface {nullptr};

MemoryLoggingInterface::MemoryLoggingInterface(const std::string& name, const RCUTILS_LOG_SEVERITY severity)
    : name_(name) {
  this->prev_logging_interface_ = g_current_logging_interface;
  g_current_logging_interface = this;

  auto ret = rcutils_logging_initialize();
  if (ret != RCUTILS_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "");
  }

  this->prev_log_level_ = static_cast<RCUTILS_LOG_SEVERITY>(rcutils_logging_get_default_logger_level());
  rcutils_logging_set_default_logger_level(severity);

  const auto handler =
    [](const rcutils_log_location_t* location, const int level,
       const char* name, const rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
      if (g_current_logging_interface == nullptr) {
        return;
      }

      rcl_interfaces::msg::Log msg;
      msg.name = name;
      msg.file = location->file_name;
      msg.function = location->function_name;
      msg.line = location->line_number;
      msg.level = cras::logLevelToMsgLevel(static_cast<RCUTILS_LOG_SEVERITY>(level));
      msg.msg = cras::snprintf(format, *args);
      msg.stamp = rclcpp::Time(timestamp);
      g_current_logging_interface->addLogMessage(msg);
    };

  this->prev_handler_ = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(handler);
}

MemoryLoggingInterface::~MemoryLoggingInterface() {
  rcutils_logging_set_output_handler(this->prev_handler_);
  rcutils_logging_set_default_logger_level(this->prev_log_level_);
  const auto _ = rcutils_logging_shutdown();
}

rclcpp::Logger MemoryLoggingInterface::get_logger() const {
  return rclcpp::get_logger(this->name_);
}

const char* MemoryLoggingInterface::get_logger_name() const {
  return this->name_.c_str();
}

void MemoryLoggingInterface::create_logger_services(
#if RCLCPP_VERSION_GTE(31, 0, 0)
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr& node_services)
#else
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
#endif
{
}

void MemoryLoggingInterface::addLogMessage(const rcl_interfaces::msg::Log& msg) {
  this->messages_.push_back(msg);
}

const std::list<rcl_interfaces::msg::Log>& MemoryLoggingInterface::getMessages() const {
  return this->messages_;
}

void MemoryLoggingInterface::clear() {
  this->messages_.clear();
}

}  // namespace cras
