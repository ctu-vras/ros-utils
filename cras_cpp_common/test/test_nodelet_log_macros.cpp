/**
 * \file
 * \brief Unit test for nodelet_utils/log_macros.h
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <ros/ros.h>

#include <cras_cpp_common/nodelet_utils/log_macros.h>

#include "log_appender.inc"

/**
 * \brief Fake being inside a nodelet by providing `getName()` free function which is used by the NODELET_* macros.
 * \return
 */
std::string getName()
{
  return "my_nodelet";
}

/**
 * \brief Advance ros::Time by the given amount.
 * \param[in] duration Seconds to advance.
 */
void advanceTime(const double duration)
{
  ros::Time::setNow(ros::Time::now() + ros::Duration(duration));
}

/**
 * \brief Test NODELET_DEBUG_DELAYED_THROTTLE and NODELET_DEBUG_STREAM_DELAYED_THROTTLE macros.
 */
TEST(NodeletLogMacros, Debug)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  std::vector<std::string> expStrs;
  std::vector<ros::console::Level> expLevels;
  std::vector<size_t> expNums;
  ros::console::Level lf = ros::console::Level::Count;
  ros::console::Level lt = ros::console::Level::Debug;

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_DEBUG_DELAYED_THROTTLE(1.0, "test%i", 1);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test1", "", "", "", "", "", "", "", "", "", "test1", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_DEBUG_STREAM_DELAYED_THROTTLE(1.0, "test" << 2);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test2", "", "", "", "", "", "", "", "", "", "test2", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);
}

/**
 * \brief Test NODELET_INFO_DELAYED_THROTTLE and NODELET_INFO_STREAM_DELAYED_THROTTLE macros.
 */
TEST(NodeletLogMacros, Info)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  std::vector<std::string> expStrs;
  std::vector<ros::console::Level> expLevels;
  std::vector<size_t> expNums;
  ros::console::Level lf = ros::console::Level::Count;
  ros::console::Level lt = ros::console::Level::Info;

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_INFO_DELAYED_THROTTLE(1.0, "test%i", 1);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test1", "", "", "", "", "", "", "", "", "", "test1", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_INFO_STREAM_DELAYED_THROTTLE(1.0, "test" << 2);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test2", "", "", "", "", "", "", "", "", "", "test2", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);
}

/**
 * \brief Test NODELET_WARN_DELAYED_THROTTLE and NODELET_WARN_STREAM_DELAYED_THROTTLE macros.
 */
TEST(NodeletLogMacros, Warn)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  std::vector<std::string> expStrs;
  std::vector<ros::console::Level> expLevels;
  std::vector<size_t> expNums;
  ros::console::Level lf = ros::console::Level::Count;
  ros::console::Level lt = ros::console::Level::Warn;

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_WARN_DELAYED_THROTTLE(1.0, "test%i", 1);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test1", "", "", "", "", "", "", "", "", "", "test1", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_WARN_STREAM_DELAYED_THROTTLE(1.0, "test" << 2);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test2", "", "", "", "", "", "", "", "", "", "test2", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);
}

/**
 * \brief Test NODELET_ERROR_DELAYED_THROTTLE and NODELET_ERROR_STREAM_DELAYED_THROTTLE macros.
 */
TEST(NodeletLogMacros, Error)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  std::vector<std::string> expStrs;
  std::vector<ros::console::Level> expLevels;
  std::vector<size_t> expNums;
  ros::console::Level lf = ros::console::Level::Count;
  ros::console::Level lt = ros::console::Level::Error;

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_ERROR_DELAYED_THROTTLE(1.0, "test%i", 1);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test1", "", "", "", "", "", "", "", "", "", "test1", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_ERROR_STREAM_DELAYED_THROTTLE(1.0, "test" << 2);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test2", "", "", "", "", "", "", "", "", "", "test2", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);
}

/**
 * \brief Test NODELET_FATAL_DELAYED_THROTTLE and NODELET_FATAL_STREAM_DELAYED_THROTTLE macros.
 */
TEST(NodeletLogMacros, Fatal)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  std::vector<std::string> expStrs;
  std::vector<ros::console::Level> expLevels;
  std::vector<size_t> expNums;
  ros::console::Level lf = ros::console::Level::Count;
  ros::console::Level lt = ros::console::Level::Fatal;

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_FATAL_DELAYED_THROTTLE(1.0, "test%i", 1);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test1", "", "", "", "", "", "", "", "", "", "test1", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);

  logger.reset();
  for (size_t i = 0; i < 22; ++i)
  {
    logger.resetMsg();
    NODELET_FATAL_STREAM_DELAYED_THROTTLE(1.0, "test" << 2);
    logger.afterLog();
    advanceTime(0.1);
  }
  expStrs = {"", "", "", "", "", "", "", "", "", "", "test2", "", "", "", "", "", "", "", "", "", "test2", ""};
  expLevels = {lf, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf, lf, lf, lf, lf, lf, lf, lf, lf, lt, lf};
  expNums = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2};
  EXPECT_EQ(expStrs, logger.strs);
  EXPECT_EQ(expLevels, logger.levels);
  EXPECT_EQ(expNums, logger.nums);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // ROS init.
  ros::init(argc, argv, "test_nodelet_log_macros");
  ros::Time::init();

  // Allow logging Debug messages.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  // Set the custom log appender that records messages.
  ros::console::register_appender(&logger);

  return RUN_ALL_TESTS();
}
