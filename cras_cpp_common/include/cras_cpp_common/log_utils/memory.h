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

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>

namespace cras
{

/**
 * Log helper redirecting storing logged messages in memory.
 */
class MemoryLoggingInterface : public ::rclcpp::node_interfaces::NodeLoggingInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(MemoryLoggingInterface)

  explicit MemoryLoggingInterface(const std::string& name = "log",
    RCUTILS_LOG_SEVERITY severity = RCUTILS_LOG_SEVERITY_DEBUG);

  ~MemoryLoggingInterface() override;

  rclcpp::Logger get_logger() const override;
  const char* get_logger_name() const override;
  void create_logger_services(rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services) override;

  virtual void addLogMessage(const rcl_interfaces::msg::Log& msg);

  /**
   * \brief Return all messages logged so far.
   * \return The messages.
   */
  virtual const ::std::list<::rcl_interfaces::msg::Log>& getMessages() const;

  /**
   * \brief Delete all messages logged so far.
   */
  void clear();

protected:
  ::std::list<::rcl_interfaces::msg::Log> messages;  //!< \brief The list of logged messages.

  ::std::string name;

  ::RCUTILS_LOG_SEVERITY prevLogLevel {RCUTILS_LOG_SEVERITY_UNSET};
  ::rcutils_logging_output_handler_t prevHandler {nullptr};
  ::cras::MemoryLoggingInterface* prevLoggingInterface {nullptr};
};

}
