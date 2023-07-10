// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test for PriorityMux.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>
#include <unordered_map>

#include <cras_cpp_common/log_utils/memory.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <cras_topic_tools/priority_mux_base.h>

using namespace cras;

TEST(PriorityMuxBase, OneOutput)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();

  std::unordered_map<std::string, cras::priority_mux::TopicConfig> topicConfigs = {
    {"o1p10", {"o1p10", "o1p10", "o1", 10, {1, 0}}},
    {"o1p20", {"o1p20", "o1p20", "o1", 20, {1, 0}}},
    {"o1p30", {"o1p30", "o1p30", "o1", 30, {1, 0}}},
  };

  std::string timerName;
  ros::Duration timerTimeout;
  bool timerCalled {false};
  auto setTimer = [&](const std::string& name, const ros::Duration& timeout)
  {
    timerName = name;
    timerTimeout = timeout;
    timerCalled = true;
  };

  PriorityMux mux(topicConfigs, {}, setTimer, ros::Time(0), log);

  EXPECT_EQ(0, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));

  // prio 10, none is active => prio 10 selected
  EXPECT_TRUE(mux.cb("o1p10", ros::Time(1), ros::Time(1)));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 10 before timeout, should still be selected
  mux.update(ros::Time(1.5));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // prio 10 timed out => none selected
  mux.update(ros::Time(2.0));

  EXPECT_EQ(0, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // prio 10, none is active => prio 10 selected
  EXPECT_TRUE(mux.cb("o1p10", ros::Time(3), ros::Time(3)));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 10, 10 active => prio 10 selected
  EXPECT_TRUE(mux.cb("o1p10", ros::Time(3.1), ros::Time(3.1)));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 30, 10 active => 30 selected
  EXPECT_TRUE(mux.cb("o1p30", ros::Time(3.5), ros::Time(3.5)));

  EXPECT_EQ(30, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p30", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p30", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 10, 30 active => ignored
  EXPECT_FALSE(mux.cb("o1p10", ros::Time(3.5), ros::Time(3.5)));

  EXPECT_EQ(30, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p30", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {}; timerTimeout = {}; timerCalled = false;

  // prio 20, 30 active => ignored
  EXPECT_FALSE(mux.cb("o1p20", ros::Time(4), ros::Time(4)));

  EXPECT_EQ(30, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p30", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p20", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 30 timed out, 20 active => 20 selected
  mux.update(ros::Time(4.6));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // prio 20, 20 active => 20 selected
  EXPECT_TRUE(mux.cb("o1p20", ros::Time(5), ros::Time(5)));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p20", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 20 timed out, none active => none selected
  mux.update(ros::Time(6));

  EXPECT_EQ(0, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);
}

TEST(PriorityMuxBase, OneOutputLongTimeout)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();

  std::unordered_map<std::string, cras::priority_mux::TopicConfig> topicConfigs = {
    {"o1p10", {"o1p10", "o1p10", "o1", 10, {10, 0}}},
    {"o1p20", {"o1p20", "o1p20", "o1", 20, {1,  0}}},
  };

  std::string timerName;
  ros::Duration timerTimeout;
  bool timerCalled {false};
  auto setTimer = [&](const std::string& name, const ros::Duration& timeout)
  {
    timerName = name;
    timerTimeout = timeout;
    timerCalled = true;
  };

  PriorityMux mux(topicConfigs, {}, setTimer, ros::Time(0), log);

  // prio 10, none is active => prio 10 selected
  EXPECT_TRUE(mux.cb("o1p10", ros::Time(1), ros::Time(1)));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(10, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 20, 10 active => prio 20 selected
  EXPECT_TRUE(mux.cb("o1p20", ros::Time(2), ros::Time(2)));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p20", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // 20 active => 20 selected
  mux.update(ros::Time(2.5));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // 20 timed out, 10 still active => 10 selected
  mux.update(ros::Time(3.5));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // prio 20, 20 active => 20 selected
  EXPECT_TRUE(mux.cb("o1p20", ros::Time(5), ros::Time(5)));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p20", timerName);
  EXPECT_EQ(ros::Duration(1, 0), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // prio 20 timed out, 10 still active => 10 selected
  mux.update(ros::Time(6));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // prio 10 is explicitly released => none selected
  mux.disableCb("o1p10", ros::Time(7), true, ros::Time(7));

  EXPECT_EQ(0, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__disable_o1p10", timerName);
  EXPECT_EQ(ros::Duration(10, 0), timerTimeout);

  timerName = {}; timerTimeout = {}; timerCalled = false;

  // prio 10 timed out, none other active => none selected
  mux.update(ros::Time(11));

  EXPECT_EQ(0, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);
}

TEST(PriorityMuxBase, DelayedMessages)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();

  std::unordered_map<std::string, cras::priority_mux::TopicConfig> topicConfigs = {
    {"o1p10", {"o1p10", "o1p10", "o1", 10, {10, 0}}},
    {"o1p20", {"o1p20", "o1p20", "o1", 20, {1,  0}}},
  };

  std::string timerName;
  ros::Duration timerTimeout;
  bool timerCalled {false};
  auto setTimer = [&](const std::string& name, const ros::Duration& timeout)
  {
    timerName = name;
    timerTimeout = timeout;
    timerCalled = true;
  };

  PriorityMux mux(topicConfigs, {}, setTimer, ros::Time(0), log);

  const ros::Duration halfSec(0, 500000000);

  // message came before timeout, timer will be set
  EXPECT_TRUE(mux.cb("o1p10", ros::Time(1), ros::Time(1) + halfSec));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(9) + halfSec, timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // message came too late, no timer will be set and priority is unchanged
  EXPECT_TRUE(mux.cb("o1p20", ros::Time(1) + halfSec, ros::Time(2) + halfSec));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);
}

TEST(PriorityMuxBase, OneOutputAndLock)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();

  std::unordered_map<std::string, cras::priority_mux::TopicConfig> topicConfigs = {
    {"o1p10", {"o1p10", "o1p10", "o1", 10, {10, 0}}},
    {"o1p20", {"o1p20", "o1p20", "o1", 20, {1,  0}}},
  };

  std::unordered_map<std::string, cras::priority_mux::LockConfig> lockConfigs = {
    {"l15", {"l15", "l15", 15, {1, 0}}},
    {"l17", {"l17", "l17", 17, {0, 0}}},
  };

  std::string timerName;
  ros::Duration timerTimeout;
  bool timerCalled {false};
  auto setTimer = [&](const std::string& name, const ros::Duration& timeout)
  {
    timerName = name;
    timerTimeout = timeout;
    timerCalled = true;
  };

  PriorityMux mux(topicConfigs, lockConfigs, setTimer, ros::Time(0), log);

  // prio 10, lock 15 locked (because never published)
  EXPECT_FALSE(mux.cb("o1p10", ros::Time(2), ros::Time(2)));

  EXPECT_EQ(15, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p10", timerName);
  EXPECT_EQ(ros::Duration(10), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // unlock lock 15 => prio 10 selected
  mux.lockCb("l15", ros::Time(2), false, ros::Time(2));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("l15", timerName);
  EXPECT_EQ(ros::Duration(1), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // lock 15
  mux.lockCb("l15", ros::Time(2.1), true, ros::Time(2.1));

  EXPECT_EQ(15, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);  // timer not called because the lock is explicitly locked

  // prio 20, lock 15 locked => 20 selected
  EXPECT_TRUE(mux.cb("o1p20", ros::Time(2.2), ros::Time(2.2)));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o1p20", timerName);
  EXPECT_EQ(ros::Duration(1), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // unlock lock 15 => prio 20 still selected
  mux.lockCb("l15", ros::Time(2.7), false, ros::Time(2.7));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("l15", timerName);
  EXPECT_EQ(ros::Duration(1), timerTimeout);

  timerName = {};
  timerTimeout = {};
  timerCalled = false;

  // lock 17 => prio 20 still selected
  mux.lockCb("l17", ros::Time(2.8), true, ros::Time(2.8));

  EXPECT_EQ(20, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);  // timer not called because lock 17 has no timeout

  // prio 20 timed out, lock 17 locked
  mux.update(ros::Time(3.3));

  EXPECT_EQ(17, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);

  // unlock 17 => lock 15 still unlocked, prio 10 still selected
  mux.lockCb("l17", ros::Time(3.4), false, ros::Time(3.4));

  EXPECT_EQ(10, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);  // timer not called because lock 17 has no timeout

  // lock 15 times out
  mux.update(ros::Time(5.0));

  EXPECT_EQ(15, mux.getActivePriority());
  ASSERT_EQ(1u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_FALSE(timerCalled);
}

TEST(PriorityMuxBase, MultiOutputs)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();

  std::unordered_map<std::string, cras::priority_mux::TopicConfig> topicConfigs = {
    {"o1p10", {"o1p10", "o1p10", "o1", 10, {10, 0}}},
    {"o1p20", {"o1p20", "o1p20", "o1", 20, {1,  0}}},
    {"o2p10", {"o2p10", "o2p10", "o2", 10, {1,  0}}},
    {"o2p20", {"o2p20", "o2p20", "o2", 20, {1,  0}}},
    {"o2p30", {"o2p30", "o2p30", "o2", 30, {1,  0}}},
    {"o3p10", {"o3p10", "o3p10", "o3", 10, {10, 0}}},
    {"o3p20", {"o3p20", "o3p20", "o3", 20, {10, 0}}},
    {"o3p30", {"o3p30", "o3p30", "o3", 30, {10, 0}}},
  };

  PriorityMux mux(topicConfigs, {}, [](const std::string&, const ros::Duration&) {}, ros::Time(0), log);
  ASSERT_EQ(3u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o2"));
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o3"));

  // prio 10
  EXPECT_TRUE(mux.cb("o1p10", ros::Time(1), ros::Time(1)));
  EXPECT_EQ(10, mux.getActivePriority());
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p10", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p10", mux.getLastSelectedTopics().at("o3"));

  // prio 30
  EXPECT_TRUE(mux.cb("o2p30", ros::Time(2), ros::Time(2)));
  EXPECT_EQ(30, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p30", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p30", mux.getLastSelectedTopics().at("o3"));

  // prio 20, but 30 is active
  EXPECT_FALSE(mux.cb("o2p20", ros::Time(2.1), ros::Time(2.1)));
  EXPECT_EQ(30, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p30", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p30", mux.getLastSelectedTopics().at("o3"));

  // prio 30 times out, but 20 not yet
  mux.update(ros::Time(3));
  EXPECT_EQ(20, mux.getActivePriority());
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p20", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p20", mux.getLastSelectedTopics().at("o3"));

  // prio 20 times out, but 10 not yet
  mux.update(ros::Time(3.2));
  EXPECT_EQ(10, mux.getActivePriority());
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p10", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p10", mux.getLastSelectedTopics().at("o3"));

  // prio 10 times out
  mux.update(ros::Time(12));
  EXPECT_EQ(0, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));
}


TEST(PriorityMuxBase, MultiOutputsWithLocks)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();

  std::unordered_map<std::string, cras::priority_mux::TopicConfig> topicConfigs = {
    {"o1p10", {"o1p10", "o1p10", "o1", 10, {10, 0}}},
    {"o1p20", {"o1p20", "o1p20", "o1", 20, {1,  0}}},
    {"o2p10", {"o2p10", "o2p10", "o2", 10, {1,  0}}},
    {"o2p20", {"o2p20", "o2p20", "o2", 20, {1,  0}}},
    {"o2p30", {"o2p30", "o2p30", "o2", 30, {1,  0}}},
    {"o3p10", {"o3p10", "o3p10", "o3", 10, {10, 0}}},
    {"o3p20", {"o3p20", "o3p20", "o3", 20, {10, 0}}},
    {"o3p30", {"o3p30", "o3p30", "o3", 30, {10, 0}}},
  };

  std::unordered_map<std::string, cras::priority_mux::LockConfig> lockConfigs = {
    {"l15", {"l15", "l15", 15, {1, 0}}},
    {"l50", {"l50", "l50", 50, {0, 0}}},
  };

  PriorityMux mux(topicConfigs, lockConfigs, [](const std::string&, const ros::Duration&) {}, ros::Time(0), log);
  ASSERT_EQ(3u, mux.getLastSelectedTopics().size());
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o1"));
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o2"));
  ASSERT_NE(mux.getLastSelectedTopics().end(), mux.getLastSelectedTopics().find("o3"));

  // prio 10, lock 15 locked (because never published)
  EXPECT_FALSE(mux.cb("o1p10", ros::Time(2), ros::Time(2)));

  EXPECT_EQ(15, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));

  // lock 15 unlocked, prio 10 becomes active
  mux.lockCb("l15", ros::Time(2), false, ros::Time(2));
  EXPECT_EQ(10, mux.getActivePriority());
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p10", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p10", mux.getLastSelectedTopics().at("o3"));

  // prio 30, lock 15 locks
  EXPECT_TRUE(mux.cb("o2p30", ros::Time(3), ros::Time(3)));
  EXPECT_EQ(30, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p30", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p30", mux.getLastSelectedTopics().at("o3"));

  // prio 20, but 30 is active
  EXPECT_FALSE(mux.cb("o2p20", ros::Time(3.1), ros::Time(3.1)));
  EXPECT_EQ(30, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p30", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p30", mux.getLastSelectedTopics().at("o3"));

  // lock 50 locked
  mux.lockCb("l50", ros::Time(3), true, ros::Time(3));
  EXPECT_EQ(50, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));

  // prio 30 times out, but 20 not yet; lock 50 still locked
  mux.update(ros::Time(4));
  EXPECT_EQ(50, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));

  // lock 50 unlocked; prio 30 times out, but 20 not yet
  mux.lockCb("l50", ros::Time(4), false, ros::Time(4));
  EXPECT_EQ(20, mux.getActivePriority());
  EXPECT_EQ("o1p20", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p20", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p20", mux.getLastSelectedTopics().at("o3"));

  // prio 20 times out, lock 15 is locked
  mux.update(ros::Time(4.2));
  EXPECT_EQ(15, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));

  // lock 15 is unlocked, prio 10 still not timed out
  mux.lockCb("l15", ros::Time(5), false, ros::Time(5));
  EXPECT_EQ(10, mux.getActivePriority());
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p10", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p10", mux.getLastSelectedTopics().at("o3"));

  // try releasing a topic that is active but is not responsible for it
  mux.disableCb("o2p10", ros::Time(5.5), true, ros::Time(5.5));
  EXPECT_EQ(10, mux.getActivePriority());
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p10", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p10", mux.getLastSelectedTopics().at("o3"));

  // release o1p10, explicitly releasing prio 10
  mux.disableCb("o1p10", ros::Time(5.5), true, ros::Time(5.5));
  EXPECT_EQ(0, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));

  // enable o1p10 again, allowing prio 10 again
  mux.disableCb("o1p10", ros::Time(5.6), false, ros::Time(5.6));
  EXPECT_EQ(10, mux.getActivePriority());
  EXPECT_EQ("o1p10", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("o2p10", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("o3p10", mux.getLastSelectedTopics().at("o3"));

  // prio 10 times out, lock 15 is locked
  mux.update(ros::Time(12));
  EXPECT_EQ(15, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));

  // prio 10 times out, lock 15 unlocked
  mux.lockCb("l15", ros::Time(12), false, ros::Time(12));
  EXPECT_EQ(0, mux.getActivePriority());
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o1"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o2"));
  EXPECT_EQ("__none", mux.getLastSelectedTopics().at("o3"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
