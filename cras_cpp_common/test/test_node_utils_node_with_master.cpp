// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for node_utils/node_with_optional_master (the case with master available).
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>
#include <vector>

#include <ros/param.h>

#include <cras_cpp_common/node_utils/node_with_optional_master.h>
#include <cras_cpp_common/log_utils/memory.h>

using namespace cras;

std::string ns;  // NOLINT

#ifndef GTEST_SKIP
#define GTEST_SKIP() return;
#endif

TEST(NodeWithoutMaster, NoNamespace)
{
  if (!ns.empty())
    GTEST_SKIP();

  LogHelperPtr log(new MemoryLogHelper{});
  NodeWithOptionalMaster node(log);

  char arg0[] = "test";
  char arg1[] = "_param1:=3";
  char arg2[] = "_param2:=test";
  char arg3[] = "remap1:=remap";
  char arg4[] = "normal_arg";
  std::array<char*, 6> argv = {&arg0[0], &arg1[0], &arg2[0], &arg3[0], &arg4[0], nullptr};
  auto argc = static_cast<int>(argv.size()) - 1;

  node.init(argc, argv.data(), "no_master", 0);

  EXPECT_TRUE(node.usesMaster());
  EXPECT_TRUE(node.isInitialized());
  EXPECT_TRUE(node.ok());
  EXPECT_EQ(2, argc);

  const auto params = node.getPrivateParams();
  EXPECT_EQ(3, params->getParam("param1", 0));
  EXPECT_EQ(std::string("test"), params->getParam("param2", "fail"));

  EXPECT_EQ(std::string("/remap"), node.resolveName("remap1"));
  EXPECT_EQ(std::string("/no_remap"), node.resolveName("no_remap"));

  node.shutdown();

  EXPECT_FALSE(node.ok());
  EXPECT_FALSE(node.isInitialized());
}

TEST(NodeWithoutMaster, Namespace)
{
  if (ns.empty())
    GTEST_SKIP();

  LogHelperPtr log(new MemoryLogHelper{});
  NodeWithOptionalMaster node(log);

  char arg0[] = "test";
  char arg1[] = "_param1:=3";
  char arg2[] = "_param2:=test";
  char arg3[] = "remap1:=remap";
  char arg4[] = "normal_arg";
  char arg5[] = "__ns:=ns";
  std::array<char*, 7> argv = {&arg0[0], &arg1[0], &arg2[0], &arg3[0], &arg4[0], &arg5[0], nullptr};
  auto argc = static_cast<int>(argv.size()) - 1;

  node.init(argc, argv.data(), "no_master", 0);

  EXPECT_TRUE(node.usesMaster());
  EXPECT_TRUE(node.isInitialized());
  EXPECT_TRUE(node.ok());
  EXPECT_EQ(2, argc);

  const auto params = node.getPrivateParams();
  EXPECT_EQ(3, params->getParam("param1", 0));
  EXPECT_EQ(std::string("test"), params->getParam("param2", "fail"));

  EXPECT_EQ(std::string("/ns/remap"), node.resolveName("remap1"));
  EXPECT_EQ(std::string("/ns/no_remap"), node.resolveName("no_remap"));

  node.shutdown();

  EXPECT_FALSE(node.ok());
  EXPECT_FALSE(node.isInitialized());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  std::vector<std::string> args(argv, argv + argc);
  if (std::find(args.begin(), args.end(), "ns") != args.end())
    ns = "ns";

  return RUN_ALL_TESTS();
}
