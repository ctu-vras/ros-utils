// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS logging helpers.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <nodelet/nodelet.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/suppress_warnings.h>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

void LogHelper::print(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const char* file, int line, const char* function, const std::string fmt, ...) const
{
  va_list(args);
  va_start(args, fmt);
  const auto str = cras::format(fmt, args);
  va_end(args);
  this->logString(filter, logger, level, str, file, line, function);
}

void LogHelper::print(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const char* file, int line, const char* function, const char* fmt, ...) const
{
  va_list(args);
  va_start(args, fmt);
  const auto str = cras::format(fmt, args);
  va_end(args);
  this->logString(filter, logger, level, str, file, line, function);
}

void LogHelper::print(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const std::stringstream& ss, const char* file, int line, const char* function) const
{
  this->logString(filter, logger, level, ss.str(), file, line, function);
}

ros::Time LogHelper::getTimeNow() const
{
  return cras::nowFallbackToWall();
}

const void* LogHelper::getId() const
{
  return this;
}

void LogHelper::setGlobalLogger() const
{
  CRAS_IGNORE_DEPRECATED_WARNING_BEGIN
  setCrasLogger(std::make_shared<WrapperLogHelper>(this));
  CRAS_IGNORE_DEPRECATED_WARNING_END
}

void LogHelper::print(ros::console::Level level, const std::string& text) const
{
  CRAS_IGNORE_DEPRECATED_WARNING_BEGIN
  auto log = std::make_shared<WrapperLogHelper>(this);
  CRAS_IGNORE_DEPRECATED_WARNING_END
  switch (level)
  {
    case ros::console::Level::Debug:
      CRAS_LOG_DEBUG(log, text);
      break;
    case ros::console::Level::Info:
      CRAS_LOG_INFO(log, text);
      break;
    case ros::console::Level::Warn:
      CRAS_LOG_WARN(log, text);
      break;
    case ros::console::Level::Error:
      CRAS_LOG_ERROR(log, text);
      break;
    case ros::console::Level::Fatal:
      CRAS_LOG_FATAL(log, text);
      break;
    default:
      CRAS_LOG_ERROR(log, "Invalid log level %i. Printing as error level.", level);
      CRAS_LOG_ERROR(log, text);
      break;
  }
}

void LogHelper::log(ros::console::Level level, const std::string format, ...) const
{
  if (level < ROSCONSOLE_MIN_SEVERITY)
    return;

  va_list(args);
  va_start(args, format);
  const auto& text = cras::format(format, args);
  va_end(args);

  CRAS_IGNORE_DEPRECATED_WARNING_BEGIN
  this->print(level, text);
  CRAS_IGNORE_DEPRECATED_WARNING_END
}

void LogHelper::log(ros::console::Level level, const char* format, ...) const
{
  if (level < ROSCONSOLE_MIN_SEVERITY)
    return;

  va_list(args);
  va_start(args, format);
  const auto& text = cras::format(format, args);
  va_end(args);

  CRAS_IGNORE_DEPRECATED_WARNING_BEGIN
  this->print(level, text);
  CRAS_IGNORE_DEPRECATED_WARNING_END
}

void RosconsoleLogHelper::initialize() const
{
  this->initialized = true;
  if (ROS_UNLIKELY(!ros::console::g_initialized))
    ros::console::initialize();
}

void RosconsoleLogHelper::initializeLogLocation(
  ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const
{
  const auto goodLevel = (level < ros::console::Level::Count) ? level : ros::console::Level::Error;
  ros::console::initializeLogLocation(loc, name, goodLevel);
  if (level != goodLevel)
  {
    const auto str = cras::format("Invalid log level %i. Printing as error level.", level);
    this->logString(nullptr, loc->logger_, ros::console::Level::Error, str, __FILE__, __LINE__,
      __ROSCONSOLE_FUNCTION__);
  }
}

void RosconsoleLogHelper::setLogLocationLevel(ros::console::LogLocation* loc, ros::console::Level level) const
{
  ros::console::setLogLocationLevel(loc, (level < ros::console::Level::Count) ? level : ros::console::Level::Error);
}

void RosconsoleLogHelper::checkLogLocationEnabled(ros::console::LogLocation* loc) const
{
  ros::console::checkLogLocationEnabled(loc);
}

void RosconsoleLogHelper::logString(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const std::string& str, const char* file, int line, const char* function) const
{
  ros::console::print(filter, logger, level, file, line, function, "%s", str.c_str());
}

WrapperLogHelper::WrapperLogHelper(const ::cras::LogHelper* wrapped) : wrapped(wrapped)
{
  this->initialized = this->wrapped->initialized;
}

void WrapperLogHelper::initialize() const
{
  this->initialized = true;
  this->wrapped->initialize();
}

void WrapperLogHelper::initializeLogLocation(
  ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const
{
  this->wrapped->initializeLogLocation(loc, name, level);
}

void WrapperLogHelper::setLogLocationLevel(ros::console::LogLocation* loc, ros::console::Level level) const
{
  this->wrapped->setLogLocationLevel(loc, level);
}

void WrapperLogHelper::checkLogLocationEnabled(ros::console::LogLocation* loc) const
{
  this->wrapped->checkLogLocationEnabled(loc);
}

void WrapperLogHelper::logString(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const std::string& str, const char* file, int line, const char* function) const
{
  this->wrapped->logString(filter, logger, level, str, file, line, function);
}

ros::Time WrapperLogHelper::getTimeNow() const
{
  return this->wrapped->getTimeNow();
}

const void* WrapperLogHelper::getId() const
{
  return this->wrapped->getId();
}

HasLogger::HasLogger(const cras::LogHelperPtr& log) : log(log)
{
}

cras::LogHelperConstPtr HasLogger::getCrasLogger() const
{
  return this->log;
}

static cras::LogHelperConstPtr g_cras_logger;
static cras::LogHelperConstPtr g_prev_cras_logger;

}

cras::LogHelperConstPtr getCrasLogger()
{
  if (!cras::g_cras_logger)
    cras::g_cras_logger = std::make_shared<cras::NodeLogHelper>();
  return cras::g_cras_logger;
}

cras::LogHelperConstPtr setCrasLogger(const cras::LogHelperConstPtr& log)
{
  cras::g_prev_cras_logger = cras::g_cras_logger;
  cras::g_cras_logger = log;
  return cras::g_prev_cras_logger;
}

void restorePreviousCrasLogger()
{
  if (cras::g_prev_cras_logger)
    cras::g_cras_logger = cras::g_prev_cras_logger;
}
