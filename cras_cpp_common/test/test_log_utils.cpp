/**
 * \file
 * \brief Unit test for log_utils.h
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/log_utils/nodelet.h>

#include "log_appender.inc"

class TestLogger : public cras::LogHelper
{
protected:
  void printDebug(const std::string& text) const override
  {
    this->debugMsg = text;
  }
  void printDebugNamed(const std::string& name, const std::string& text) const override
  {
    this->debugMsg = text;
    this->name = name;
  }
  void printDebugCond(bool condition, const std::string& text) const override
  {
    this->debugMsg = text;
    this->condition = condition;
  }
  void printDebugCondNamed(bool condition, const std::string& name, const std::string& text) const override
  {
    this->debugMsg = text;
    this->condition = condition;
    this->name = name;
  }
  void printDebugOnce(const std::string& text) const override
  {
    this->debugMsg = text;
    this->once = true;
  }
  void printDebugOnceNamed(const std::string& name, const std::string& text) const override
  {
    this->debugMsg = text;
    this->once = true;
    this->name = name;
  }
  void printDebugThrottle(double period, const std::string& text) const override
  {
    this->debugMsg = text;
    this->period = period;
  }
  void printDebugThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->debugMsg = text;
    this->period = period;
    this->name = name;
  }
  void printDebugDelayedThrottle(double period, const std::string& text) const override
  {
    this->debugMsg = text;
    this->delayed = true;
    this->period = period;
  }
  void printDebugDelayedThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->debugMsg = text;
    this->delayed = true;
    this->period = period;
    this->name = name;
  }
  void printDebugFilter(::ros::console::FilterBase* filter, const std::string& text) const override
  {
    this->debugMsg = text;
    this->filter = filter;
  }
  void printDebugFilterNamed(
    ::ros::console::FilterBase* filter, const std::string& name, const std::string& text) const override
  {
    this->debugMsg = text;
    this->filter = filter;
    this->name = name;
  }

  void printInfo(const std::string& text) const override
  {
    this->infoMsg = text;
  }
  void printInfoNamed(const std::string& name, const std::string& text) const override
  {
    this->infoMsg = text;
    this->name = name;
  }
  void printInfoCond(bool condition, const std::string& text) const override
  {
    this->infoMsg = text;
    this->condition = condition;
  }
  void printInfoCondNamed(bool condition, const std::string& name, const std::string& text) const override
  {
    this->infoMsg = text;
    this->condition = condition;
    this->name = name;
  }
  void printInfoOnce(const std::string& text) const override
  {
    this->infoMsg = text;
    this->once = true;
  }
  void printInfoOnceNamed(const std::string& name, const std::string& text) const override
  {
    this->infoMsg = text;
    this->once = true;
    this->name = name;
  }
  void printInfoThrottle(double period, const std::string& text) const override
  {
    this->infoMsg = text;
    this->period = period;
  }
  void printInfoThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->infoMsg = text;
    this->period = period;
    this->name = name;
  }
  void printInfoDelayedThrottle(double period, const std::string& text) const override
  {
    this->infoMsg = text;
    this->delayed = true;
    this->period = period;
  }
  void printInfoDelayedThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->infoMsg = text;
    this->delayed = true;
    this->period = period;
    this->name = name;
  }
  void printInfoFilter(::ros::console::FilterBase* filter, const std::string& text) const override
  {
    this->infoMsg = text;
    this->filter = filter;
  }
  void printInfoFilterNamed(
    ::ros::console::FilterBase* filter, const std::string& name, const std::string& text) const override
  {
    this->infoMsg = text;
    this->filter = filter;
    this->name = name;
  }

  void printWarn(const std::string& text) const override
  {
    this->warnMsg = text;
  }
  void printWarnNamed(const std::string& name, const std::string& text) const override
  {
    this->warnMsg = text;
    this->name = name;
  }
  void printWarnCond(bool condition, const std::string& text) const override
  {
    this->warnMsg = text;
    this->condition = condition;
  }
  void printWarnCondNamed(bool condition, const std::string& name, const std::string& text) const override
  {
    this->warnMsg = text;
    this->condition = condition;
    this->name = name;
  }
  void printWarnOnce(const std::string& text) const override
  {
    this->warnMsg = text;
    this->once = true;
  }
  void printWarnOnceNamed(const std::string& name, const std::string& text) const override
  {
    this->warnMsg = text;
    this->once = true;
    this->name = name;
  }
  void printWarnThrottle(double period, const std::string& text) const override
  {
    this->warnMsg = text;
    this->period = period;
  }
  void printWarnThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->warnMsg = text;
    this->period = period;
    this->name = name;
  }
  void printWarnDelayedThrottle(double period, const std::string& text) const override
  {
    this->warnMsg = text;
    this->delayed = true;
    this->period = period;
  }
  void printWarnDelayedThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->warnMsg = text;
    this->delayed = true;
    this->period = period;
    this->name = name;
  }
  void printWarnFilter(::ros::console::FilterBase* filter, const std::string& text) const override
  {
    this->warnMsg = text;
    this->filter = filter;
  }
  void printWarnFilterNamed(
    ::ros::console::FilterBase* filter, const std::string& name, const std::string& text) const override
  {
    this->warnMsg = text;
    this->filter = filter;
    this->name = name;
  }

  void printError(const std::string& text) const override
  {
    this->errorMsg = text;
    this->errors.push_back(text);
  }
  void printErrorNamed(const std::string& name, const std::string& text) const override
  {
    this->errorMsg = text;
    this->name = name;
  }
  void printErrorCond(bool condition, const std::string& text) const override
  {
    this->errorMsg = text;
    this->condition = condition;
  }
  void printErrorCondNamed(bool condition, const std::string& name, const std::string& text) const override
  {
    this->errorMsg = text;
    this->condition = condition;
    this->name = name;
  }
  void printErrorOnce(const std::string& text) const override
  {
    this->errorMsg = text;
    this->once = true;
  }
  void printErrorOnceNamed(const std::string& name, const std::string& text) const override
  {
    this->errorMsg = text;
    this->once = true;
    this->name = name;
  }
  void printErrorThrottle(double period, const std::string& text) const override
  {
    this->errorMsg = text;
    this->period = period;
  }
  void printErrorThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->errorMsg = text;
    this->period = period;
    this->name = name;
  }
  void printErrorDelayedThrottle(double period, const std::string& text) const override
  {
    this->errorMsg = text;
    this->delayed = true;
    this->period = period;
  }
  void printErrorDelayedThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->errorMsg = text;
    this->delayed = true;
    this->period = period;
    this->name = name;
  }
  void printErrorFilter(::ros::console::FilterBase* filter, const std::string& text) const override
  {
    this->errorMsg = text;
    this->filter = filter;
  }
  void printErrorFilterNamed(
    ::ros::console::FilterBase* filter, const std::string& name, const std::string& text) const override
  {
    this->errorMsg = text;
    this->filter = filter;
    this->name = name;
  }

  void printFatal(const std::string& text) const override
  {
    this->fatalMsg = text;
  }
  void printFatalNamed(const std::string& name, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->name = name;
  }
  void printFatalCond(bool condition, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->condition = condition;
  }
  void printFatalCondNamed(bool condition, const std::string& name, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->condition = condition;
    this->name = name;
  }
  void printFatalOnce(const std::string& text) const override
  {
    this->fatalMsg = text;
    this->once = true;
  }
  void printFatalOnceNamed(const std::string& name, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->once = true;
    this->name = name;
  }
  void printFatalThrottle(double period, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->period = period;
  }
  void printFatalThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->period = period;
    this->name = name;
  }
  void printFatalDelayedThrottle(double period, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->delayed = true;
    this->period = period;
  }
  void printFatalDelayedThrottleNamed(double period, const std::string& name, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->delayed = true;
    this->period = period;
    this->name = name;
  }
  void printFatalFilter(::ros::console::FilterBase* filter, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->filter = filter;
  }
  void printFatalFilterNamed(
    ::ros::console::FilterBase* filter, const std::string& name, const std::string& text) const override
  {
    this->fatalMsg = text;
    this->filter = filter;
    this->name = name;
  }

public:
  void reset()
  {
    this->debugMsg = this->infoMsg = this->warnMsg = this->errorMsg = this->fatalMsg = this->name = "";
    this->period = 0.0;
    this->filter = nullptr;
    this->condition = this->once = this->delayed = false;
    this->errors.clear();
  }

  mutable std::string debugMsg {};
  mutable std::string infoMsg {};
  mutable std::string warnMsg {};
  mutable std::string errorMsg {};
  mutable std::string fatalMsg {};

  mutable std::string name {};
  mutable double period {};
  mutable ::ros::console::FilterBase* filter {};
  mutable bool condition {};
  mutable bool once {};
  mutable bool delayed {};

  mutable std::vector<std::string> errors;
};

/**
 * \brief Test that the logger logs using the correct logging severity.
 */
TEST(LogUtils, CorrectLogLevel)  // NOLINT
{
  TestLogger log;

  log.reset(); log.logDebug("a"); EXPECT_EQ("a", log.debugMsg);
  EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.logInfo("a"); EXPECT_EQ("a", log.infoMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.logWarn("a"); EXPECT_EQ("a", log.warnMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.logError("a"); EXPECT_EQ("a", log.errorMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.logFatal("a"); EXPECT_EQ("a", log.fatalMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg);


  log.reset(); log.log(ros::console::Level::Debug, "a"); EXPECT_EQ("a", log.debugMsg);
  EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.log(ros::console::Level::Info, "a"); EXPECT_EQ("a", log.infoMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.log(ros::console::Level::Warn, "a"); EXPECT_EQ("a", log.warnMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.log(ros::console::Level::Error, "a"); EXPECT_EQ("a", log.errorMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.log(ros::console::Level::Fatal, "a"); EXPECT_EQ("a", log.fatalMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg);


  log.reset(); log.print(ros::console::Level::Debug, "a"); EXPECT_EQ("a", log.debugMsg);
  EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.print(ros::console::Level::Info, "a"); EXPECT_EQ("a", log.infoMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.print(ros::console::Level::Warn, "a"); EXPECT_EQ("a", log.warnMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.errorMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.print(ros::console::Level::Error, "a"); EXPECT_EQ("a", log.errorMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.print(ros::console::Level::Fatal, "a"); EXPECT_EQ("a", log.fatalMsg);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.errorMsg);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedDebug)  // NOLINT
{
  TestLogger log;
  ros::console::FilterBase f;

  log.reset(); log.logDebugNamed("test", "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugCond(true, "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugCondNamed(true, "test", "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugOnce("a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugOnceNamed("test", "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugThrottle(1.0, "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugDelayedThrottle(1.0, "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logDebugDelayedThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logDebugFilter(&f, "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logDebugFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", log.debugMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedInfo)  // NOLINT
{
  TestLogger log;
  ros::console::FilterBase f;

  log.reset(); log.logInfoNamed("test", "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoCond(true, "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoCondNamed(true, "test", "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoOnce("a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoOnceNamed("test", "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoThrottle(1.0, "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoDelayedThrottle(1.0, "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logInfoDelayedThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logInfoFilter(&f, "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logInfoFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", log.infoMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedWarn)  // NOLINT
{
  TestLogger log;
  ros::console::FilterBase f;

  log.reset(); log.logWarnNamed("test", "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnCond(true, "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnCondNamed(true, "test", "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnOnce("a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnOnceNamed("test", "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnThrottle(1.0, "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnDelayedThrottle(1.0, "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logWarnDelayedThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logWarnFilter(&f, "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logWarnFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", log.warnMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedError)  // NOLINT
{
  TestLogger log;
  ros::console::FilterBase f;

  log.reset(); log.logErrorNamed("test", "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorCond(true, "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorCondNamed(true, "test", "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorOnce("a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorOnceNamed("test", "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorThrottle(1.0, "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorDelayedThrottle(1.0, "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logErrorDelayedThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logErrorFilter(&f, "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logErrorFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", log.errorMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);
}

/**
 * \brief Test the extended logging functions.
 */
TEST(LogUtils, ExtendedFatal)  // NOLINT
{
  TestLogger log;
  ros::console::FilterBase f;

  log.reset(); log.logFatalNamed("test", "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalCond(true, "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalCondNamed(true, "test", "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(true, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalOnce("a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalOnceNamed("test", "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(true, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalThrottle(1.0, "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalDelayedThrottle(1.0, "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logFatalDelayedThrottleNamed(1.0, "test", "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(nullptr, log.filter); EXPECT_EQ(1.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(true, log.delayed);

  log.reset(); log.logFatalFilter(&f, "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);

  log.reset(); log.logFatalFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", log.fatalMsg); EXPECT_EQ("test", log.name); EXPECT_EQ(&f, log.filter); EXPECT_EQ(0.0, log.period);
  EXPECT_EQ(false, log.condition); EXPECT_EQ(false, log.once); EXPECT_EQ(false, log.delayed);
}

/**
 * \brief Test logging in printf-style.
 */
TEST(LogUtils, PrintfArgs)  // NOLINT
{
  TestLogger log;

  log.reset(); log.logInfo("%s", std::string("cras").c_str()); EXPECT_EQ("cras", log.infoMsg);
  log.reset(); log.logInfo("%i", -42); EXPECT_EQ("-42", log.infoMsg);
  log.reset(); log.logInfo("%u", 42); EXPECT_EQ("42", log.infoMsg);
  log.reset(); log.logInfo("%f", 42.0); EXPECT_EQ("42.000000", log.infoMsg);
  log.reset(); log.logInfo("%f", 3.14); EXPECT_EQ("3.140000", log.infoMsg);
  log.reset(); log.logInfo("%s %i %f", "cras", -42, 3.14); EXPECT_EQ("cras -42 3.140000", log.infoMsg);
}

/**
 * \brief Test logging in printf-style.
 */
TEST(LogUtils, PrintfStringArgs)  // NOLINT
{
  TestLogger log;
  using s = std::string;

  log.reset(); log.logInfo(s("%s"), std::string("cras").c_str()); EXPECT_EQ("cras", log.infoMsg);
  log.reset(); log.logInfo(s("%i"), -42); EXPECT_EQ("-42", log.infoMsg);
  log.reset(); log.logInfo(s("%u"), 42); EXPECT_EQ("42", log.infoMsg);
  log.reset(); log.logInfo(s("%f"), 42.0); EXPECT_EQ("42.000000", log.infoMsg);
  log.reset(); log.logInfo(s("%f"), 3.14); EXPECT_EQ("3.140000", log.infoMsg);
  log.reset(); log.logInfo(s("%s %i %f"), "cras", -42, 3.14); EXPECT_EQ("cras -42 3.140000", log.infoMsg);
}

/**
 * \brief Test logging of very long strings.
 */
TEST(LogUtils, LongStrings)  // NOLINT
{
  TestLogger log;

  std::string longString(300000, '*');  // generates a string of length 300.000 asterisks
  log.reset(); log.logInfo("%s", longString.c_str()); EXPECT_EQ(longString, log.infoMsg);
  log.reset(); log.logInfo(longString.c_str()); EXPECT_EQ(longString, log.infoMsg);
}

/**
 * \brief Test logging at wrong logging level.
 */
TEST(LogUtils, WrongLevel)  // NOLINT
{
  TestLogger log;

  log.reset(); log.log(static_cast<ros::console::Level>(1000), "cras");
  ASSERT_EQ(2u, log.errors.size());
  EXPECT_TRUE(cras::startsWith(log.errors[0], "Invalid log level "));
  EXPECT_EQ("cras", log.errors[1]);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.fatalMsg);

  log.reset(); log.print(static_cast<ros::console::Level>(1000), "cras");
  ASSERT_EQ(2u, log.errors.size());
  EXPECT_TRUE(cras::startsWith(log.errors[0], "Invalid log level "));
  EXPECT_EQ("cras", log.errors[1]);
  EXPECT_EQ("", log.debugMsg); EXPECT_EQ("", log.infoMsg); EXPECT_EQ("", log.warnMsg); EXPECT_EQ("", log.fatalMsg);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, Log)  // NOLINT
{
  cras::NodeLogHelper log;

  logger.reset(); log.logDebug("cras"); logger.afterLog();
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logInfo("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logWarn("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logError("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logFatal("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.log(ros::console::Level::Debug, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Info, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Warn, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Error, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Fatal, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.print(ros::console::Level::Debug, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Info, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Warn, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Error, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Fatal, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

void t(const double time)
{
  ros::Time::setNow(ros::Time(time));
}

class DisableLogFilter : public ros::console::FilterBase
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
  cras::NodeLogHelper log;
  ros::console::FilterBase f;
  DisableLogFilter fd;

  logger.reset(); log.logDebugNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logDebugCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logDebugCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logDebugOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logDebugOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logDebugThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logDebugThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logDebugDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logDebugDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logDebugFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logDebugFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedInfo)  // NOLINT
{
  cras::NodeLogHelper log;
  ros::console::FilterBase f;
  DisableLogFilter fd;

  logger.reset(); log.logInfoNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logInfoCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logInfoCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logInfoOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logInfoOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logInfoThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logInfoThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logInfoDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logInfoDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logInfoFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logInfoFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedWarn)  // NOLINT
{
  cras::NodeLogHelper log;
  ros::console::FilterBase f;
  DisableLogFilter fd;

  logger.reset(); log.logWarnNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logWarnCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logWarnCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logWarnOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logWarnOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logWarnThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logWarnThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logWarnDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logWarnDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logWarnFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logWarnFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedError)  // NOLINT
{
  cras::NodeLogHelper log;
  ros::console::FilterBase f;
  DisableLogFilter fd;

  logger.reset(); log.logErrorNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logErrorCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logErrorCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logErrorOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logErrorOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logErrorThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logErrorThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logErrorDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logErrorDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logErrorFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logErrorFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros.
 */
TEST(NodeLogUtils, LogExtendedFatal)  // NOLINT
{
  cras::NodeLogHelper log;
  ros::console::FilterBase f;
  DisableLogFilter fd;

  logger.reset(); log.logFatalNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logFatalCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logFatalCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logFatalOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logFatalOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logFatalThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logFatalThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logFatalDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logFatalDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logFatalFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logFatalFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using ROS_* macros when passing a std::string format.
 */
TEST(NodeLogUtils, LogString)  // NOLINT
{
  cras::NodeLogHelper log;

  using s = std::string;

  logger.reset(); log.logDebug(s("cras")); logger.afterLog();
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logInfo(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logWarn(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logError(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logFatal(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.log(ros::console::Level::Debug, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Info, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Warn, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Error, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Fatal, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.print(ros::console::Level::Debug, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Info, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Warn, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Error, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Fatal, s("cras"));
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
  cras::NodeletLogHelper log(&testGetName);

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logDebug("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logInfo("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logWarn("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logError("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logFatal("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.log(ros::console::Level::Debug, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Info, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Warn, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Error, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Fatal, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.print(ros::console::Level::Debug, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Info, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Warn, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Error, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Fatal, "cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  // Test that setting the logger level of the nodelet's named logger actually affects the logging.
  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Error))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logDebug("cras");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); log.logInfo("cras");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); log.logWarn("cras");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); log.logError("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logFatal("cras");
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

/**
 * \brief Test that nodelet logger logs using NODELET_* macros when passing std::string format.
 */
TEST(NodeletLogUtils, LogString)  // NOLINT
{
  cras::NodeletLogHelper log(&testGetName);
  using s = std::string;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logDebug(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logInfo(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logWarn(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logError(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logFatal(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.log(ros::console::Level::Debug, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Info, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Warn, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Error, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.log(ros::console::Level::Fatal, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.print(ros::console::Level::Debug, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Info, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Warn, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Error, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.print(ros::console::Level::Fatal, s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  // Test that setting the logger level of the nodelet's named logger actually affects the logging.
  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Error))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logDebug(s("cras"));
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); log.logInfo(s("cras"));
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); log.logWarn(s("cras"));
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
  logger.reset(); log.logError(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);
  logger.reset(); log.logFatal(s("cras"));
  EXPECT_EQ("cras", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedDebug)  // NOLINT
{
  cras::NodeletLogHelper log(&testGetName);
  ros::console::FilterBase f;
  DisableLogFilter fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logDebugNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logDebugCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logDebugCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logDebugOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logDebugOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logDebugThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logDebugThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logDebugDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logDebugDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logDebugFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logDebugFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Debug, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logDebugFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedInfo)  // NOLINT
{
  cras::NodeletLogHelper log(&testGetName);
  ros::console::FilterBase f;
  DisableLogFilter fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logInfoNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logInfoCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logInfoCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logInfoOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logInfoOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logInfoThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logInfoThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logInfoDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logInfoDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logInfoFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logInfoFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Info, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logInfoFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedWarn)  // NOLINT
{
  cras::NodeletLogHelper log(&testGetName);
  ros::console::FilterBase f;
  DisableLogFilter fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logWarnNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logWarnCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logWarnCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logWarnOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logWarnOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logWarnThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logWarnThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logWarnDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logWarnDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logWarnFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logWarnFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Warn, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logWarnFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedError)  // NOLINT
{
  cras::NodeletLogHelper log(&testGetName);
  ros::console::FilterBase f;
  DisableLogFilter fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logErrorNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logErrorCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logErrorCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logErrorOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logErrorOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logErrorThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logErrorThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logErrorDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logErrorDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logErrorFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logErrorFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Error, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logErrorFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
}

/**
 * \brief Test that node logger logs using NODELET_* macros.
 */
TEST(NodeletLogUtils, LogExtendedFatal)  // NOLINT
{
  cras::NodeletLogHelper log(&testGetName);
  ros::console::FilterBase f;
  DisableLogFilter fd;

  if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + ".test", ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  logger.reset(); log.logFatalNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalCond(false, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logFatalCondNamed(false, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logFatalCond(true, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalCondNamed(true, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logFatalOnce("a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 3; ++i) log.logFatalOnceNamed("test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logFatalThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 4; ++i) { t((i + 1) / 2.0); log.logFatalThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logFatalDelayedThrottle(1.0, "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset();
  for (size_t i = 0; i < 6; ++i) { t((i + 1) / 2.0); log.logFatalDelayedThrottleNamed(1.0, "test", "a"); }
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(2u, logger.num);

  logger.reset(); log.logFatalFilter(&f, "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalFilter(&fd, "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);

  logger.reset(); log.logFatalFilterNamed(&f, "test", "a");
  EXPECT_EQ("a", logger.str); EXPECT_EQ(ros::console::Level::Fatal, logger.level); EXPECT_EQ(1u, logger.num);

  logger.reset(); log.logFatalFilterNamed(&fd, "test", "a");
  EXPECT_EQ("", logger.str); EXPECT_EQ(ros::console::Level::Count, logger.level); EXPECT_EQ(0u, logger.num);
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
