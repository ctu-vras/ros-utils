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
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/suppress_warnings.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include "cras_cpp_common/type_utils.hpp"

/**
 * \brief Test that the logger logs using the correct logging severity.
 */
TEST(MemoryLogUtils, Basic)  // NOLINT
{
  auto logIf = cras::MemoryLoggingInterface::make_shared("test");
  auto log = logIf->get_logger();

  const auto clk = rclcpp::Clock();

  rcl_interfaces::msg::Log msg;

  logIf->clear(); RCLCPP_DEBUG(log, "a"); ASSERT_EQ(1, logIf->getMessages().size()); msg = logIf->getMessages().front();
  EXPECT_EQ(__FILE__, msg.file); EXPECT_EQ(__LINE__ - 1, msg.line);  // This line has to be right after the log call
  EXPECT_EQ("a", msg.msg); EXPECT_EQ(rcl_interfaces::msg::Log::DEBUG, msg.level);
  EXPECT_NEAR(clk.now().seconds(), cras::float_secs(msg.stamp), 1.0); EXPECT_EQ("test", msg.name);
  EXPECT_EQ(__func__, msg.function);

  logIf->clear(); RCLCPP_INFO(log, "a"); ASSERT_EQ(1u, logIf->getMessages().size()); msg = logIf->getMessages().front();
  EXPECT_EQ(__FILE__, msg.file); EXPECT_EQ(__LINE__ - 1, msg.line);  // This line has to be right after the log call
  EXPECT_EQ("a", msg.msg); EXPECT_EQ(rcl_interfaces::msg::Log::INFO, msg.level);
  EXPECT_NEAR(clk.now().seconds(), cras::float_secs(msg.stamp), 1.0); EXPECT_EQ("test", msg.name);
  EXPECT_EQ(__func__, msg.function);

  logIf->clear(); RCLCPP_WARN(log, "a"); ASSERT_EQ(1u, logIf->getMessages().size()); msg = logIf->getMessages().front();
  EXPECT_EQ(__FILE__, msg.file); EXPECT_EQ(__LINE__ - 1, msg.line);  // This line has to be right after the log call
  EXPECT_EQ("a", msg.msg); EXPECT_EQ(rcl_interfaces::msg::Log::WARN, msg.level);
  EXPECT_NEAR(clk.now().seconds(), cras::float_secs(msg.stamp), 1.0); EXPECT_EQ("test", msg.name);
  EXPECT_EQ(__func__, msg.function);

  logIf->clear(); RCLCPP_ERROR(log, "a"); ASSERT_EQ(1u, logIf->getMessages().size()); msg = logIf->getMessages().front();
  EXPECT_EQ(__FILE__, msg.file); EXPECT_EQ(__LINE__ - 1, msg.line);  // This line has to be right after the log call
  EXPECT_EQ("a", msg.msg); EXPECT_EQ(rcl_interfaces::msg::Log::ERROR, msg.level);
  EXPECT_NEAR(clk.now().seconds(), cras::float_secs(msg.stamp), 1.0); EXPECT_EQ("test", msg.name);
  EXPECT_EQ(__func__, msg.function);

  logIf->clear(); RCLCPP_FATAL(log, "a"); ASSERT_EQ(1u, logIf->getMessages().size()); msg = logIf->getMessages().front();
  EXPECT_EQ(__FILE__, msg.file); EXPECT_EQ(__LINE__ - 1, msg.line);  // This line has to be right after the log call
  EXPECT_EQ("a", msg.msg); EXPECT_EQ(rcl_interfaces::msg::Log::FATAL, msg.level);
  EXPECT_NEAR(clk.now().seconds(), cras::float_secs(msg.stamp), 1.0); EXPECT_EQ("test", msg.name);
  EXPECT_EQ(__func__, msg.function);

  logIf->clear(); RCLCPP_INFO_ONCE(log, "a"); RCLCPP_INFO_ONCE(log, "b"); ASSERT_EQ(2u, logIf->getMessages().size());
  const auto& msg1 = logIf->getMessages().front(); const auto& msg2 = logIf->getMessages().back();
  EXPECT_EQ("a", msg1.msg); EXPECT_EQ("b", msg2.msg);

  logIf->clear(); for (size_t i = 0; i < 3; ++i) RCLCPP_INFO_ONCE(log, "%zu", i);
  ASSERT_EQ(1u, logIf->getMessages().size());
  msg = logIf->getMessages().front(); EXPECT_EQ("0", msg.msg);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
