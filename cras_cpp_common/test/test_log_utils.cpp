/**
 * \file
 * \brief Unit test for log_utils.h
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>

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
  
  void printInfo(const std::string& text) const override
  {
    this->infoMsg = text;
  }
  
  void printWarn(const std::string& text) const override
  {
    this->warnMsg = text;
  }
  
  void printError(const std::string& text) const override
  {
    this->errorMsg = text;
    this->errors.push_back(text);
  }
  
  void printFatal(const std::string& text) const override
  {
    this->fatalMsg = text;
  }

public:
  void reset()
  {
    this->debugMsg = this->infoMsg = this->warnMsg = this->errorMsg = this->fatalMsg = "";
    this->errors.clear();
  }
  
  mutable std::string debugMsg {};
  mutable std::string infoMsg {};
  mutable std::string warnMsg {};
  mutable std::string errorMsg {};
  mutable std::string fatalMsg {};
  
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::console::register_appender(&logger);
  // Allow logging Debug messages.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();
  
  return RUN_ALL_TESTS();
}