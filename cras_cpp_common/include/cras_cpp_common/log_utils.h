#pragma once

#include <cstdarg>
#include <string>

#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

/**
 * This class (reps. its descendants) provides unified access to ROS logging functions, be it ROS_* or NODELET_*.
 */
class LogHelper
{
public:

  inline void logDebug(const char* format, ...) const {
    va_list(args);
    va_start(args, format);
    this->printDebug(cras::format(format, args));
    va_end(args);
  };

  inline void logInfo(const char* format, ...) const {
    va_list(args);
    va_start(args, format);
    this->printInfo(cras::format(format, args));
    va_end(args);
  };

  inline void logWarn(const char* format, ...) const {
    va_list(args);
    va_start(args, format);
    this->printWarn(cras::format(format, args));
    va_end(args);
  };

  inline void logError(const char* format, ...) const {
    va_list(args);
    va_start(args, format);
    this->printError(cras::format(format, args));
    va_end(args);
  };

  inline void logFatal(const char* format, ...) const {
    va_list(args);
    va_start(args, format);
    this->printFatal(cras::format(format, args));
    va_end(args);
  };

protected:
  virtual void printDebug(const std::string& text) const = 0;
  virtual void printInfo(const std::string& text) const = 0;
  virtual void printWarn(const std::string& text) const = 0;
  virtual void printError(const std::string& text) const = 0;
  virtual void printFatal(const std::string& text) const = 0;
};

}