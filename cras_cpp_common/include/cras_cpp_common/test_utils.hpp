#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test utilities for gtest.
 * \author Martin Pecka
 */

#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>

namespace cras
{

/**
 * Create a descendant of this class and use it as a fixture for Gtest tests using TEST_F macro. It will automatically
 * start and stop the rclcpp backend.
 */
class RclcppTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

}
