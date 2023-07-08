// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS logging helpers.
 * \author Martin Pecka
 */

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <nodelet/nodelet.h>
#include <rosgraph_msgs/Log.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/suppress_warnings.h>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

static std::unordered_map<const void*, size_t> logHelperNumInstances;

LogHelper::LogHelper()
{
  // The only way this number can get over 1 is that the memory manager recycles an address already given to a previous
  // instance that got deleted in the meantime. This way, we know this number will stay equal for the whole lifetime
  // of an instance, and will be different when the memory address is recycled.
  logHelperNumInstances[this]++;
}

void LogHelper::print(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const char* file, int line, const char* function, const std::string fmt, ...) const
{
  va_list(args);
  va_start(args, fmt);
  const auto str = cras::format(fmt, args);
  va_end(args);
  this->print(filter, logger, level, str, file, line, function);
}

void LogHelper::print(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const char* file, int line, const char* function, const char* fmt, ...) const
{
  va_list(args);
  va_start(args, fmt);
  const auto str = cras::format(fmt, args);
  va_end(args);
  this->print(filter, logger, level, str, file, line, function);
}

void LogHelper::print(ros::console::FilterBase* filter, void* logger, ros::console::Level level,
  const std::stringstream& ss, const char* file, int line, const char* function) const
{
  this->print(filter, logger, level, ss.str(), file, line, function);
}

void LogHelper::print(ros::console::FilterBase* filter, void* logger, const ros::console::Level level,
  const std::string& str, const char* file, int line, const char* function) const
{
  auto outMessage = str;
  auto outLevel = level;

  if (filter)
  {
    ros::console::FilterParams params{file, line, function, outMessage.c_str(), logger, level, {}};
    if (!filter->isEnabled(params))
      return;

    outLevel = params.level;

    if (!params.out_message.empty())
      outMessage = params.out_message;
  }

  // line comes as int type from rosconsole.h, but in reality, it is created by the __LINE__ macro, which by definition
  // cannot be negative and cannot be larger than 2^31-1. uint32_t is also the type used in rosgraph_msgs/Log.
  this->logString(logger, outLevel, outMessage, file, static_cast<uint32_t>(line >= 0 ? line : 0), function);
}


void LogHelper::initialize() const
{
  if (ROS_LIKELY(this->initialized))
    return;
  this->initialized = true;
  this->initializeImpl();
}

void LogHelper::initializeLogLocation(ros::console::LogLocation* loc, const std::string& name,
  ros::console::Level level) const
{
  if (ROS_LIKELY(loc->initialized_))
    return;

  const auto goodLevel = ROS_LIKELY(level < ros::console::Level::Count) ? level : ros::console::Level::Error;
  this->initializeLogLocationImpl(loc, name, goodLevel);

  if (ROS_UNLIKELY(goodLevel != level))
  {
    const auto str = cras::format("Invalid log level %i. Printing as error level.", level);
    this->logString(loc->logger_, ros::console::Level::Error, str, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__);
  }
}

void LogHelper::setLogLocationLevel(ros::console::LogLocation* loc, ::ros::console::Level level) const
{
  loc->level_ = (level < ros::console::Level::Count) ? level : ros::console::Level::Error;
}

void LogHelper::checkLogLocationEnabled(ros::console::LogLocation* loc) const
{
  loc->logger_enabled_ = true;
}

ros::Time LogHelper::getTimeNow() const
{
  return cras::nowFallbackToWall();
}

const void* LogHelper::getId() const
{
  // We want to get an ID that is unique. As a base, we use the memory address of this logger. However, the address
  // might be a recycled address that belonged to some previous instance. logHelperNumInstances helps us differentiate
  // such instances. The reason why the memory address is hashed first is so that it is not as easy to collide with
  // other objects in the memory area of the process - in the worst case, when an array of loghelpers would be
  // constructed, simply adding +1 to the current one's address might easily point to the address of the next one and
  // their IDs might clash in case the first one has been constructed one times more than the following one.
  // Here, we hope that hashing will uniformly distribute even sequential addresses into non-overlapping intervals
  // (okay, unless somebody creates billions of instances).
  return reinterpret_cast<const void*>(std::hash<const void*>{}(this) + logHelperNumInstances[this]);
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

void RosconsoleLogHelper::initializeImpl() const
{
  if (ROS_UNLIKELY(!ros::console::g_initialized))
    ros::console::initialize();
}

void RosconsoleLogHelper::initializeLogLocationImpl(
  ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const
{
  ros::console::initializeLogLocation(loc, name, level);
}

void RosconsoleLogHelper::setLogLocationLevel(ros::console::LogLocation* loc, ros::console::Level level) const
{
  ros::console::setLogLocationLevel(
    loc, ROS_LIKELY(level < ros::console::Level::Count) ? level : ros::console::Level::Error);
}

void RosconsoleLogHelper::checkLogLocationEnabled(ros::console::LogLocation* loc) const
{
  ros::console::checkLogLocationEnabled(loc);
}

void RosconsoleLogHelper::logString(void* logger, ros::console::Level level, const std::string& str, const char* file,
  uint32_t line, const char* function) const
{
  ros::console::print(nullptr, logger, level, file, static_cast<int>(line), function, "%s", str.c_str());
}

WrapperLogHelper::WrapperLogHelper(const ::cras::LogHelper* wrapped) : wrapped(wrapped)
{
  this->initialized = this->wrapped->initialized;
}

void WrapperLogHelper::initializeImpl() const
{
  this->wrapped->initialize();
}

void WrapperLogHelper::initializeLogLocationImpl(
  ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const
{
  this->wrapped->initializeLogLocationImpl(loc, name, level);
}

void WrapperLogHelper::setLogLocationLevel(ros::console::LogLocation* loc, ros::console::Level level) const
{
  this->wrapped->setLogLocationLevel(loc, level);
}

void WrapperLogHelper::checkLogLocationEnabled(ros::console::LogLocation* loc) const
{
  this->wrapped->checkLogLocationEnabled(loc);
}

void WrapperLogHelper::logString(void* logger, ros::console::Level level, const std::string& str, const char* file,
  uint32_t line, const char* function) const
{
  this->wrapped->logString(logger, level, str, file, line, function);
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

void HasLogger::setCrasLogger(const ::cras::LogHelperPtr& log)
{
  this->log = log;
}

static cras::LogHelperConstPtr g_cras_logger;
static cras::LogHelperConstPtr g_prev_cras_logger;

int8_t logLevelToRosgraphMsgLevel(ros::console::Level rosLevel)
{
  switch (rosLevel)
  {
    case ros::console::Level::Debug:
      return rosgraph_msgs::Log::DEBUG;
    case ros::console::Level::Info:
      return rosgraph_msgs::Log::INFO;
    case ros::console::Level::Warn:
      return rosgraph_msgs::Log::WARN;
    case ros::console::Level::Error:
      return rosgraph_msgs::Log::ERROR;
    case ros::console::Level::Fatal:
      return rosgraph_msgs::Log::FATAL;
    default:
      return rosgraph_msgs::Log::FATAL;
  }
}

ros::console::Level rosgraphMsgLevelToLogLevel(uint8_t msgLevel)
{
  switch (msgLevel)
  {
    case rosgraph_msgs::Log::DEBUG:
      return ros::console::Level::Debug;
    case rosgraph_msgs::Log::INFO:
      return ros::console::Level::Info;
    case rosgraph_msgs::Log::WARN:
      return ros::console::Level::Warn;
    case rosgraph_msgs::Log::ERROR:
      return ros::console::Level::Error;
    case rosgraph_msgs::Log::FATAL:
      return ros::console::Level::Fatal;
    default:
      return ros::console::Level::Fatal;
  }
}

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
