#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief ROS logging helpers.
 * \author Martin Pecka
 *
 * The idea of CRAS logging system is that your code should not care whether it is running in a node or a nodelet.
 * So it provides unified `CRAS_*` logging macros similar to `ROS_*` macros, which automatically choose a suitable
 * logger. This choice is performed by evaluating expression `getCrasLogger()` in the current context. By default,
 * it picks up the `::getCrasLogger()` global functions which returns a logger that mimicks the `ROS_*` macros.
 * When writing code inside a nodelet, the base nodelet class provides an "override" of this method that returns a
 * logger that mimicks `NODELET_*` macros. Filters also have their specialized default logger. And each of your classes
 * can have, or you can even just define a local lambda called `getCrasLogger` that would return the logger to use.
 * If you want to pass a logger explicitly, use the `CRAS_LOG_*` variants that take logger as the first argument. You
 * can also write your own loggers that behave very differently.
 */

#include <cstdarg>
#include <memory>
#include <string>

#include <ros/console.h>

#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

/**
 * \brief Convert the given rosconsole logging level to `rosgraph_msgs::Log` level constant.
 * \param[in] rosLevel The rosconsole logging level.
 * \return The `rosgraph_msgs::Log` level constant.
 */
int8_t logLevelToRosgraphMsgLevel(::ros::console::Level rosLevel);

/**
 * \brief Convert the given `rosgraph_msgs::Log` level constant to rosconsole logging level.
 * \param[in] msgLevel A `rosgraph_msgs::Log` level constant.
 * \return The rosconsole logging level.
 */
::ros::console::Level rosgraphMsgLevelToLogLevel(uint8_t msgLevel);

/**
 * \brief This class (reps. its descendants) provides unified access to ROS logging functions, be it ROS_* or NODELET_*.
 *
 * Subclasses only need to implement the print* functions that print a pre-formatted string.
 */
class LogHelper
{
public:
  typedef ::std::shared_ptr<::cras::LogHelper> Ptr;  //!< \brief Pointer to `LogHelper`.
  typedef ::std::shared_ptr<const ::cras::LogHelper> ConstPtr;  //!< \brief Const pointer to `LogHelper`.

  // The class is not copyable/movable to prevent slicing

  LogHelper() = default;
  virtual ~LogHelper() = default;
  LogHelper(const LogHelper&) = delete;
  LogHelper& operator=(const LogHelper&) = delete;
  LogHelper(LogHelper &&) = delete;
  LogHelper& operator=(LogHelper &&) = delete;

  //! \brief Whether `initialize()` has been called at least once. Subclasses are required to handle this invariant.
  mutable bool initialized {false};

  /**
   * \brief Initialize the logger. This function does what it needs and then calls `initializeImpl()`.
   */
  void initialize() const;

protected:
  /**
   * \brief Initialize the logger. It is not needed to check `initialized` as this function is only called when it was
   *        false.
   */
  virtual void initializeImpl() const = 0;

public:
  /**
   * \brief Initialize the given log location with proper data for later use. This is the function called from macros.
   *        This function checks whether log level is within bounds, corrects it if needed, calls
   *        `initializeLogLocaionImpl()`, and, if the passed log level was wrong, logs an additional error (after the
   *        log location has been initialized by the impl function).
   * \param[out] loc The location to fill.
   * \param[in] name Name of the logger.
   * \param[in] level Logging level of the message at this location.
   */
  void initializeLogLocation(
    ::ros::console::LogLocation* loc, const std::string& name, ::ros::console::Level level) const;

protected:
  /**
   * \brief Initialize the given log location with proper data for later use. This function is only called if the log
   *        location was not already initialized.
   * \param[out] loc The location to fill.
   * \param[in] name Name of the logger.
   * \param[in] level Logging level of the message at this location. The level is already checked to be valid.
   */
  virtual void initializeLogLocationImpl(
    ::ros::console::LogLocation* loc, const std::string& name, ::ros::console::Level level) const = 0;

public:
  /**
   * \brief Set level of a log location. This is only called when global logging level changes.
   * \param[in,out] loc The location to update.
   * \param[in] level The new level.
   */
  virtual void setLogLocationLevel(::ros::console::LogLocation* loc, ::ros::console::Level level) const;

  /**
   * \brief Check whether the log location is enabled.
   * \param[in] loc The location to check.
   */
  virtual void checkLogLocationEnabled(::ros::console::LogLocation* loc) const;

  /**
   * \brief Write the given string to the log.
   * \param[in] logger Private logger data read from the relevant log location (set by `initializeLogLocation()`).
   * \param[in] level Level of the logged message.
   * \param[in] str Logged message.
   * \param[in] file File at which the logging macro has been called.
   * \param[in] line Line on which the logging macro has been called.
   * \param[in] function Name of function from which the logging macro has been called.
   */
  virtual void logString(void* logger, ::ros::console::Level level, const ::std::string& str, const char* file,
    uint32_t line, const char* function) const = 0;

  /**
   * \brief Get current time (used for throttling messages). By default, ROS time is returned, with fallback to wall
   *        time if ROS time is not initialized.
   * \return Current time.
   */
  virtual ::ros::Time getTimeNow() const;

  /**
   * \brief Return a unique identifier of this logger. These IDs are used for determining which logger has already
   *        logged something.
   * \return An ID.
   */
  virtual const void* getId() const;

  /**
   * \brief Print function used by the macros. It basically just relays its work to `logString()`.
   */
  void print(::ros::console::FilterBase* filter, void* logger, ::ros::console::Level level,
    const char* file, int line, const char* function, const char* fmt, ...) const ROSCONSOLE_PRINTF_ATTRIBUTE(8, 9);

  /**
   * \brief Print function used by the macros. It basically just relays its work to `logString()`.
   */
  void print(::ros::console::FilterBase* filter, void* logger, ::ros::console::Level level,
    const char* file, int line, const char* function, ::std::string fmt, ...) const;

  /**
   * \brief Print function used by the macros. It basically just relays its work to `logString()`.
   */
  void print(::ros::console::FilterBase* filter, void* logger, ::ros::console::Level level,
    const ::std::stringstream& ss, const char* file, int line, const char* function) const;

  /**
   * \brief Print function used by the macros. It basically just relays its work to `logString()`.
   */
  void print(::ros::console::FilterBase* filter, void* logger, ::ros::console::Level level, const ::std::string& str,
    const char* file, int line, const char* function) const;

  [[deprecated("This function will be removed in a future release.")]]
  void setGlobalLogger() const;

  /**
   * \brief Log a message using the given log severity.
   * \param[in] level Log severity level (one of the ros::console::Level enum constants).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  [[deprecated("Use CRAS_* logging macros instead.")]]
  void log(::ros::console::Level level, const char* format, ...) const;

  /**
   * \brief Log a message using the given log severity.
   * \param[in] level Log severity level (one of the ros::console::Level enum constants).
   * \param[in] format printf-like format string.
   * \param[in] ... Arguments to print.
   */
  [[deprecated("Use CRAS_* logging macros instead.")]]
  void log(::ros::console::Level level, ::std::string format, ...) const;

  /**
   * \brief Log a message using the given log severity.
   * \param[in] level Log severity level (one of the ros::console::Level enum constants).
   * \param[in] text The message to log.
   */
  [[deprecated("Use CRAS_* logging macros instead.")]]
  void print(::ros::console::Level level, const ::std::string& text) const;

  friend class WrapperLogHelper;
};

typedef ::cras::LogHelper::Ptr LogHelperPtr;  //!< \brief Pointer to `LogHelper`.
typedef ::cras::LogHelper::ConstPtr LogHelperConstPtr;  //!< \brief Const pointer to `LogHelper`.

/**
 * \brief Log helper relaying all of its work to the same mechanism used by `ROS_*` logging macros.
 */
class RosconsoleLogHelper : public ::cras::LogHelper
{
public:
  void setLogLocationLevel(::ros::console::LogLocation* loc, ::ros::console::Level level) const override;

  void checkLogLocationEnabled(::ros::console::LogLocation* loc) const override;

  void logString(void* logger, ::ros::console::Level level, const ::std::string& str, const char* file, uint32_t line,
    const char* function) const override;

protected:
  void initializeImpl() const override;

  void initializeLogLocationImpl(
    ::ros::console::LogLocation* loc, const ::std::string& name, ::ros::console::Level level) const override;
};

/**
 * \brief Convenience base class for providing `this->log` and `getCrasLogger()`. Just add it as a base to your class
 *        and all `CRAS_*` logging macros should work with the logger from this class.
 */
class HasLogger
{
public:
  /**
   * \brief Associate the logger with this interface.
   * \param[in] log The logger to use.
   */
  explicit HasLogger(const ::cras::LogHelperPtr& log);

  /**
   * \brief This is the function picked up by `CRAS_*` logging macros.
   * \return The internal logger.
   */
  ::cras::LogHelperConstPtr getCrasLogger() const;

protected:
  //! \brief Log helper.
  ::cras::LogHelperPtr log;
};

class [[deprecated("This wrapper should only be used to provide backward compatibility.")]]
  WrapperLogHelper : public ::cras::LogHelper
{
public:
  explicit WrapperLogHelper(const ::cras::LogHelper* wrapped);

  void setLogLocationLevel(::ros::console::LogLocation* loc, ::ros::console::Level level) const override;

  void checkLogLocationEnabled(::ros::console::LogLocation* loc) const override;

  void logString(void* logger, ::ros::console::Level level, const ::std::string& str, const char* file, uint32_t line,
    const char* function) const override;

  ::ros::Time getTimeNow() const override;

  const void* getId() const override;

protected:
  void initializeImpl() const override;

  void initializeLogLocationImpl(
    ::ros::console::LogLocation* loc, const ::std::string& name, ::ros::console::Level level) const override;

private:
  const ::cras::LogHelper* wrapped;
};

}

/**
 * \brief Get the default logger used for `CRAS_*` logging macros. It will never be null.
 * \return The logger.
 */
::cras::LogHelperConstPtr getCrasLogger();

/**
 * \brief Set the default logger used by `CRAS_*` macros.
 * \param[in] log The new logger.
 * \return The previous logger.
 */
::cras::LogHelperConstPtr setCrasLogger(const ::cras::LogHelperConstPtr& log);

[[deprecated("This function will be removed in a future release.")]]
void restorePreviousCrasLogger();

#include <cras_cpp_common/log_utils/macros.h>
#include <cras_cpp_common/log_utils/deprecated/macros.h>
