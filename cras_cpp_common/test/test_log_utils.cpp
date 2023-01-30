/**
 * \file
 * \brief Unit test for log_utils.h
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <list>
#include <memory>
#include <string>
#include <vector>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/log_utils/nodelet.h>
#include <cras_cpp_common/suppress_warnings.h>

#include "log_appender.inc"

class TestLogger : public cras::LogHelper
{
public:
  void initialize() const override
  {
    this->initialized = true;
  }

  void initializeLogLocation(
    ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const override
  {
    if (loc->initialized_)
      return;

    this->logLocationsNames.push_back(name);

    const auto goodLevel = (level < ros::console::Level::Count) ? level : ros::console::Level::Error;

    loc->logger_ = &this->logLocationsNames.back();
    loc->level_ = goodLevel;
    loc->initialized_ = true;
    loc->logger_enabled_ = true;

    if (level != goodLevel)
    {
      const auto str = cras::format("Invalid log level %i. Printing as error level.", level);
      this->logString(nullptr, loc->logger_, ros::console::Level::Error, str, __FILE__, __LINE__,
                      __ROSCONSOLE_FUNCTION__);
    }
  };

  void setLogLocationLevel(::ros::console::LogLocation* loc, ::ros::console::Level level) const override
  {
    loc->level_ = (level < ros::console::Level::Count) ? level : ros::console::Level::Error;
  }

  void checkLogLocationEnabled(::ros::console::LogLocation*) const override
  {
  }

  void logString(::ros::console::FilterBase* filter, void* logger, ::ros::console::Level level,
    const ::std::string& str, const char* file, int line, const char* function) const override
  {
    switch (level)
    {
      case ros::console::levels::Debug:
        this->debugMsg = str;
        break;
      case ros::console::levels::Info:
        this->infoMsg = str;
        break;
      case ros::console::levels::Warn:
        this->warnMsg = str;
        break;
      case ros::console::levels::Error:
        this->errorMsg = str;
        this->errors.push_back(str);
        break;
      case ros::console::levels::Fatal:
        this->fatalMsg = str;
        break;
    }
    this->filter = filter;
    this->name = cras::removePrefix(*reinterpret_cast<std::string*>(logger), ROSCONSOLE_DEFAULT_NAME ".");
    this->name = cras::removePrefix(this->name, ROSCONSOLE_DEFAULT_NAME);
  }

public:
  void reset()
  {
    this->debugMsg = this->infoMsg = this->warnMsg = this->errorMsg = this->fatalMsg = this->name = "";
    this->filter = nullptr;
    this->errors.clear();
  }

  mutable std::string debugMsg {};
  mutable std::string infoMsg {};
  mutable std::string warnMsg {};
  mutable std::string errorMsg {};
  mutable std::string fatalMsg {};

  mutable std::string name {};
  mutable ::ros::console::FilterBase* filter {};

  mutable std::vector<std::string> errors;
  mutable std::list<std::string> logLocationsNames;
};

/**
 * \brief Test that the logger logs using the correct logging severity.
 */
TEST(LogUtils, CorrectLogLevel)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};

  log->reset(); CRAS_DEBUG("a"); EXPECT_EQ("a", log->debugMsg);
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_INFO("a"); EXPECT_EQ("a", log->infoMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_WARN("a"); EXPECT_EQ("a", log->warnMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_ERROR("a"); EXPECT_EQ("a", log->errorMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_FATAL("a"); EXPECT_EQ("a", log->fatalMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg);


  log->reset(); CRAS_DEBUG_STREAM("a" << "b"); EXPECT_EQ("ab", log->debugMsg);
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_INFO_STREAM("a" << "b"); EXPECT_EQ("ab", log->infoMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_WARN_STREAM("a" << "b"); EXPECT_EQ("ab", log->warnMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_ERROR_STREAM("a" << "b"); EXPECT_EQ("ab", log->errorMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_FATAL_STREAM("a" << "b"); EXPECT_EQ("ab", log->fatalMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg);


  log->reset(); CRAS_LOG(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, "a"); EXPECT_EQ("a", log->debugMsg);
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "a"); EXPECT_EQ("a", log->infoMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "a"); EXPECT_EQ("a", log->warnMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, "a"); EXPECT_EQ("a", log->errorMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "a"); EXPECT_EQ("a", log->fatalMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg);


  log->reset(); CRAS_LOG_STREAM(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, "a" << "b");
  EXPECT_EQ("ab", log->debugMsg);
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG_STREAM(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "a" << "b");
  EXPECT_EQ("ab", log->infoMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG_STREAM(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "a" << "b");
  EXPECT_EQ("ab", log->warnMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG_STREAM(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, "a" << "b");
  EXPECT_EQ("ab", log->errorMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->fatalMsg);

  log->reset(); CRAS_LOG_STREAM(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg);
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->errorMsg);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedDebug)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_DEBUG_NAMED("test", "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_COND(true, "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_ONCE("a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_THROTTLE(1.0, "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_DELAYED_THROTTLE(1.0, "a");
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_DELAYED_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_FILTER(&f, "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_DEBUG_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedDebugStream)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_DEBUG_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_DELAYED_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_DEBUG_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", log->debugMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedInfo)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_INFO_NAMED("test", "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_COND(true, "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_ONCE("a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_THROTTLE(1.0, "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_DELAYED_THROTTLE(1.0, "a");
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_DELAYED_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_FILTER(&f, "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_INFO_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedInfoStream)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_INFO_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_DELAYED_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_INFO_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_INFO_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", log->infoMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedWarn)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_WARN_NAMED("test", "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_COND(true, "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_ONCE("a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_THROTTLE(1.0, "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_DELAYED_THROTTLE(1.0, "a");
  EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_DELAYED_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_FILTER(&f, "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_WARN_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedWarnStream)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_WARN_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_DELAYED_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_WARN_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_WARN_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", log->warnMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedError)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_ERROR_NAMED("test", "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_COND(true, "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_ONCE("a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_THROTTLE(1.0, "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_DELAYED_THROTTLE(1.0, "a");
  EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_DELAYED_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_FILTER(&f, "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_ERROR_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedErrorStream)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_ERROR_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_DELAYED_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_ERROR_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_ERROR_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", log->errorMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedFatal)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_FATAL_NAMED("test", "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_COND(true, "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_ONCE("a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_THROTTLE(1.0, "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_DELAYED_THROTTLE(1.0, "a");
  EXPECT_EQ("", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_DELAYED_THROTTLE_NAMED(1.0, "test", "a");
  EXPECT_EQ("", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_FILTER(&f, "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_FATAL_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedFatalStream)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;

  log->reset(); CRAS_FATAL_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_DELAYED_THROTTLE(1.0, "a" << "b");
  EXPECT_EQ("", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b");
  EXPECT_EQ("", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(nullptr, log->filter);

  log->reset(); CRAS_FATAL_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("", log->name); EXPECT_EQ(&f, log->filter);

  log->reset(); CRAS_FATAL_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", log->fatalMsg); EXPECT_EQ("test", log->name); EXPECT_EQ(&f, log->filter);
}

/**
 * \brief Test logging in printf-style.
 */
TEST(LogUtils, PrintfArgs)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};

  log->reset(); CRAS_INFO("%s", std::string("cras").c_str()); EXPECT_EQ("cras", log->infoMsg);
  log->reset(); CRAS_INFO("%i", -42); EXPECT_EQ("-42", log->infoMsg);
  log->reset(); CRAS_INFO("%u", 42); EXPECT_EQ("42", log->infoMsg);
  log->reset(); CRAS_INFO("%f", 42.0); EXPECT_EQ("42.000000", log->infoMsg);
  log->reset(); CRAS_INFO("%f", 3.14); EXPECT_EQ("3.140000", log->infoMsg);
  log->reset(); CRAS_INFO("%s %i %f", "cras", -42, 3.14); EXPECT_EQ("cras -42 3.140000", log->infoMsg);
}

/**
 * \brief Test logging in printf-style.
 */
TEST(LogUtils, PrintfStringArgs)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};
  using s = std::string;

  log->reset(); CRAS_INFO(s("%s"), std::string("cras").c_str()); EXPECT_EQ("cras", log->infoMsg);
  log->reset(); CRAS_INFO(s("%i"), -42); EXPECT_EQ("-42", log->infoMsg);
  log->reset(); CRAS_INFO(s("%u"), 42); EXPECT_EQ("42", log->infoMsg);
  log->reset(); CRAS_INFO(s("%f"), 42.0); EXPECT_EQ("42.000000", log->infoMsg);
  log->reset(); CRAS_INFO(s("%f"), 3.14); EXPECT_EQ("3.140000", log->infoMsg);
  log->reset(); CRAS_INFO(s("%s %i %f"), "cras", -42, 3.14); EXPECT_EQ("cras -42 3.140000", log->infoMsg);
}

/**
 * \brief Test logging of very long strings.
 */
TEST(LogUtils, LongStrings)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};

  std::string longString(300000, '*');  // generates a string of length 300.000 asterisks

  log->reset(); CRAS_INFO("%s", longString.c_str()); EXPECT_EQ(longString, log->infoMsg);

  CRAS_IGNORE_PRINTF_SECURITY_WARNING_BEGIN
  log->reset(); CRAS_INFO(longString.c_str()); EXPECT_EQ(longString, log->infoMsg);
  CRAS_IGNORE_PRINTF_SECURITY_WARNING_END
}

/**
 * \brief Test logging at wrong logging level.
 */
TEST(LogUtils, WrongLevel)  // NOLINT
{
  {
    auto log = std::make_shared<cras::NodeLogHelper>();
    auto getCrasLogger = [log]() { return log; };

    logger.reset();
    CRAS_LOG(log, static_cast<ros::console::Level>(1000), ROSCONSOLE_DEFAULT_NAME, "cras");
    ASSERT_EQ(2u, logger.strs.size());
    EXPECT_TRUE(cras::startsWith(logger.strs[0], "Invalid log level "));
    EXPECT_EQ("cras", logger.strs[1]);
  }

  {
    auto log = std::make_shared<TestLogger>();
    auto getCrasLogger = [log]() { return log; };

    log->reset(); CRAS_LOG(log, static_cast<ros::console::Level>(1000), ROSCONSOLE_DEFAULT_NAME, "cras");
    ASSERT_EQ(2u, log->errors.size());
    EXPECT_TRUE(cras::startsWith(log->errors[0], "Invalid log level "));
    EXPECT_EQ("cras", log->errors[1]);
    EXPECT_EQ("", log->debugMsg); EXPECT_EQ("", log->infoMsg); EXPECT_EQ("", log->warnMsg);
    EXPECT_EQ("", log->fatalMsg);
  }
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, Log)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};

  logger.reset(); CRAS_DEBUG("cras"); logger.afterLog();
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_INFO("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_WARN("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_ERROR("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_LOG(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};

  logger.reset(); CRAS_DEBUG_STREAM("cr" << "as"); logger.afterLog();
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_INFO_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_WARN_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_ERROR_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

void t(const double time)
{
  ros::Time::setNow(ros::Time(time));
}

class DisableLog_FILTER : public ros::console::FilterBase
{
public:
  bool isEnabled() override
  {
    return false;
  }
};

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedDebug)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_DEBUG_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedDebugStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_DEBUG_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedInfo)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_INFO_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_INFO_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_INFO_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_INFO_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedInfoStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_INFO_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_INFO_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_INFO_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedWarn)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_WARN_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_WARN_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_WARN_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_WARN_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedWarnStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_WARN_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_WARN_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_WARN_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedError)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_ERROR_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_ERROR_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_ERROR_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedErrorStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_ERROR_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_ERROR_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedFatal)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_FATAL_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_FATAL_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_FATAL_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedFatalStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  logger.reset(); CRAS_FATAL_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_FATAL_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros when passing a std::string format.
 */
TEST(NodeLogUtils, LogString)  // NOLINT
{
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto getCrasLogger = [log](){return log;};

  using s = std::string;

  logger.reset(); CRAS_DEBUG(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_INFO(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_WARN(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_ERROR(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_LOG(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

const inline std::string name {"test"};  // NOLINT

const std::string& testGetName()
{
  return name;
}

/**
 * \brief Test that nodelet logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, Log)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_DEBUG("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_INFO("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_WARN("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_ERROR("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_INFO_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_WARN_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_ERROR_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG_STREAM(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, "cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  // Test that setting the logger level of the nodelet's named logger actually affects the logging.
  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Error))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_DEBUG_STREAM("cr" << "as");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); CRAS_INFO_STREAM("cr" << "as");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); CRAS_WARN_STREAM("cr" << "as");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); CRAS_ERROR_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL_STREAM("cr" << "as");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

/**
 * \brief Test that nodelet logger logs using NODELET_* macros when passing std::string format.
 */
TEST(NodeletLogUtils, LogString)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  using s = std::string;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_DEBUG(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_INFO(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_WARN(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_ERROR(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_LOG(log, ros::console::Level::Debug, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Info, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Warn, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_LOG(log, ros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  // Test that setting the logger level of the nodelet's named logger actually affects the logging.
  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Error))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_DEBUG(s("cras"));
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); CRAS_INFO(s("cras"));
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); CRAS_WARN(s("cras"));
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); CRAS_ERROR(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); CRAS_FATAL(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedDebug)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_DEBUG_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedDebugStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_DEBUG_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_DEBUG_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_DEBUG_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_DEBUG_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedInfo)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_INFO_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_INFO_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_INFO_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_INFO_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedInfoStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_INFO_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_INFO_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_INFO_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_INFO_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_INFO_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_INFO_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedWarn)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_WARN_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_WARN_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_WARN_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_WARN_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedWarnStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Warn))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_WARN_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_WARN_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_WARN_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_WARN_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_WARN_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_WARN_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedError)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_ERROR_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_ERROR_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_ERROR_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedErrorStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Error))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_ERROR_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_ERROR_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_ERROR_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_ERROR_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_ERROR_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedFatal)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_FATAL_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_COND(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_COND_NAMED(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_COND(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_COND_NAMED(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  // repeat to know _ONCE can print when used in a different statement
  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_ONCE("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_ONCE_NAMED("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_FATAL_DELAYED_THROTTLE(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_FATAL_DELAYED_THROTTLE_NAMED(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER_NAMED(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_FILTER_NAMED(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedFatalStream)  // NOLINT
{
  auto log = std::make_shared<cras::NodeletLogHelper>(&testGetName);
  auto getCrasLogger = [log](){return log;};
  ros::console::FilterBase f;
  DisableLog_FILTER fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Fatal))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); CRAS_FATAL_STREAM_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND(false, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND_NAMED(false, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND(true, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_COND_NAMED(true, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_STREAM_ONCE("a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) CRAS_FATAL_STREAM_ONCE_NAMED("test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_STREAM_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); CRAS_FATAL_STREAM_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); CRAS_FATAL_STREAM_DELAYED_THROTTLE(1.0, "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) {
    t((i + 1) / 2.0); CRAS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(1.0, "test", "a" << "b"); }
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER(&f, "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER(&fd, "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER_NAMED(&f, "test", "a" << "b");
  EXPECT_EQ("ab", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); CRAS_FATAL_STREAM_FILTER_NAMED(&fd, "test", "a" << "b");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

#include <cras_cpp_common/log_utils/inject_rosconsole.h>

/**
 * \brief Test injecting CRAS_ macros behavior into normal ROS_ macros
 */
TEST(NodeLogUtils, InjectRosconsole)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};

  using s = std::string;

  logger.reset(); log->reset(); ROS_DEBUG(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", log->debugMsg);
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log->reset(); ROS_INFO(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", log->infoMsg);
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log->reset(); ROS_WARN(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", log->warnMsg);
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log->reset(); ROS_ERROR(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", log->errorMsg);
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log->reset(); ROS_FATAL(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", log->fatalMsg);
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

CRAS_RESTORE_ROS_LOG

/**
 * \brief Test that calling CRAS_RESTORE_ROS_LOG returns back the normal ROS_ macros behavior
 */
TEST(NodeLogUtils, RestoreRosconsole)  // NOLINT
{
  auto log = std::make_shared<TestLogger>();
  auto getCrasLogger = [log](){return log;};

  logger.reset(); ROS_DEBUG("cras"); logger.afterLog();
  EXPECT_EQ("", log->debugMsg);
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); ROS_INFO("cras");
  EXPECT_EQ("", log->infoMsg);
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); ROS_WARN("cras");
  EXPECT_EQ("", log->warnMsg);
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); ROS_ERROR("cras");
  EXPECT_EQ("", log->errorMsg);
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); ROS_FATAL("cras");
  EXPECT_EQ("", log->fatalMsg);
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::console::register_appender(&logger);
  // Allow logging Debug messages.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::Time::init();

  return RUN_ALL_TESTS();
}
