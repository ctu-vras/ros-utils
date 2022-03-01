/**
 * \file
 * \brief Unit test for nodelet_with_diagnostics.h.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"
#include <cras_cpp_common/nodelet_utils/nodelet_with_diagnostics.hpp>

#include "subscription_callbacks.inc"

using namespace cras;

#define OPTS {10.0, 10.0, 0.1, 5}
#define OPTS_L {20.0, 20.0, 0.1, 5}
#define OPTS_H {5.0, 5.0, 0.1, 5}
#define OPTS2 {10.0, 10.0, 0.1, 5, 0.0, 1.0}
#define OPTS2_LO {20.0, 20.0, 0.1, 5, 0.0, 1.0}
#define OPTS2_HO {5.0, 5.0, 0.1, 5, 0.0, 1.0}
#define OPTS2_LP {20.0, 20.0, 0.1, 5, -2.0, -1.0}
#define OPTS2_LF {20.0, 20.0, 0.1, 5, 1.0, 2.0}
#define OPTS2_HP {5.0, 5.0, 0.1, 5, -2.0, -1.0}
#define OPTS2_HF {5.0, 5.0, 0.1, 5, 1.0, 2.0}
#define OPTS2_OP {10.0, 10.0, 0.1, 5, -2.0, -1.0}
#define OPTS2_OF {10.0, 10.0, 0.1, 5, 1.0, 2.0}

struct Stat
{
  std::string message;
  diagnostic_msgs::DiagnosticStatus::_level_type level;
};

const inline Stat O({"", diagnostic_msgs::DiagnosticStatus::OK});
const inline Stat L({"Frequency too low.", diagnostic_msgs::DiagnosticStatus::WARN});
const inline Stat H({"Frequency too high.", diagnostic_msgs::DiagnosticStatus::WARN});

const inline Stat LO ({"Frequency too low.", diagnostic_msgs::DiagnosticStatus::WARN});
const inline Stat LF ({"Frequency too low.; Timestamps too far in future seen.",
  diagnostic_msgs::DiagnosticStatus::ERROR});
const inline Stat LP ({"Frequency too low.; Timestamps too far in past seen.",
  diagnostic_msgs::DiagnosticStatus::ERROR});
const inline Stat HO ({"Frequency too high.", diagnostic_msgs::DiagnosticStatus::WARN});
const inline Stat HF ({"Frequency too high.; Timestamps too far in future seen.",
  diagnostic_msgs::DiagnosticStatus::ERROR});
const inline Stat HP ({"Frequency too high.; Timestamps too far in past seen.",
  diagnostic_msgs::DiagnosticStatus::ERROR});
const inline Stat OF ({"Timestamps too far in future seen.", diagnostic_msgs::DiagnosticStatus::ERROR});
const inline Stat OP ({"Timestamps too far in past seen.", diagnostic_msgs::DiagnosticStatus::ERROR});
const inline Stat OO ({"", diagnostic_msgs::DiagnosticStatus::OK});

class NodeletDiagnosticsTest : public NodeletWithDiagnostics<nodelet::Nodelet>
{
protected:
  void onInit() override
  {
    auto pnh = this->getPrivateNodeHandle();
    pnh.setParam("topic/rate/min", 5.0);
    pnh.setParam("topic/rate/max", 5.0);
    pnh.setParam("topic/rate/tolerance", 0.1);
    pnh.setParam("topic/rate/window_size", 10);
    pnh.setParam("topic/delay/min", 2.0);
    pnh.setParam("topic/delay/max", 3.0);
    pnh.setParam("a/rate/min", 20.0);
    pnh.setParam("a/rate/max", 20.0);
    pnh.setParam("a/rate/tolerance", 0.1);
    pnh.setParam("a/rate/window_size", 10);
    pnh.setParam("a/delay/min", -2.0);
    pnh.setParam("a/delay/max", -1.0);
    pnh.setParam("b/rate/min", 20.0);
    pnh.setParam("b/rate/max", 20.0);
    pnh.setParam("b/rate/tolerance", 0.1);
    pnh.setParam("b/rate/window_size", 10);
    pnh.setParam("b/delay/min", -2.0);
    pnh.setParam("b/delay/max", -1.0);
    pnh.setParam("c/rate/min", 5.0);
    pnh.setParam("c/rate/max", 5.0);
    pnh.setParam("c/rate/tolerance", 0.1);
    pnh.setParam("c/rate/window_size", 10);
    pnh.setParam("c/delay/min", -2.0);
    pnh.setParam("c/delay/max", -1.0);
    pnh.setParam("/topic/rate/min", 20.0);
    pnh.setParam("/topic/rate/max", 20.0);
    pnh.setParam("/topic/rate/tolerance", 0.1);
    pnh.setParam("/topic/rate/window_size", 10);
    pnh.setParam("/topic/delay/min", 2.0);
    pnh.setParam("/topic/delay/max", 3.0);
    pnh.setParam("/test/c/rate/min", 20.0);
    pnh.setParam("/test/c/rate/max", 20.0);
    pnh.setParam("/test/c/rate/tolerance", 0.1);
    pnh.setParam("/test/c/rate/window_size", 10);
    pnh.setParam("/test/c/delay/min", -2.0);
    pnh.setParam("/test/c/delay/max", -1.0);
    pnh.setParam("/test/topic/rate/min", 5.0);
    pnh.setParam("/test/topic/rate/max", 5.0);
    pnh.setParam("/test/topic/rate/tolerance", 0.1);
    pnh.setParam("/test/topic/rate/window_size", 10);
    pnh.setParam("/test/topic/delay/min", 2.0);
    pnh.setParam("/test/topic/delay/max", 3.0);
    pnh.setParam("/a/rate/min", 5.0);
    pnh.setParam("/a/rate/max", 5.0);
    pnh.setParam("/a/rate/tolerance", 0.1);
    pnh.setParam("/a/rate/window_size", 10);
    pnh.setParam("/a/delay/min", 2.0);
    pnh.setParam("/a/delay/max", 3.0);
  }

public:
  void testAdvertiseNoHeader()
  {
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();
    auto tnh = ros::NodeHandle(this->getNodeHandle(), "test");
    auto rnh = ros::NodeHandle(this->getNodeHandle(), "", {{"a", "d"}});

    this->getDiagUpdater(true);  // force creating a new updater
    
    ros::Time::setNow({10, 0});
    diagnostic_msgs::DiagnosticArrayConstPtr msg;
    size_t numDiagCalled {0};
    auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000,
      [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});
  
    size_t numCalled {0};

    using Msg = std_msgs::Header;
    using MsgConstPtr = std_msgs::HeaderConstPtr;
    
    auto sub1 = nh.subscribe<Msg>("a", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub2a = pnh.subscribe<Msg>("b", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub2b = nh.subscribe<Msg>("b", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub3a = tnh.subscribe<Msg>("c", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub3b = nh.subscribe<Msg>("c", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub4 = nh.subscribe<Msg>("d", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    
    std::vector<std::unique_ptr<cras::DiagnosedPublisher<Msg>>> p;
    std::vector<Stat> expectedMsgs;
    std::vector<std::string> expectedTopics;
    auto e = [&expectedMsgs,&expectedTopics](const Stat& msg, const ::std::string& topic)
    {
      expectedMsgs.emplace_back(msg);
      expectedTopics.emplace_back(topic);
    };

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS, "topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS, "topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "~topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "~topic", "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS, "~topic", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS, "~topic", "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "~topic", "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "b", 10)); e(L, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "b", 10)); e(L, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>("b", 10)); e(L, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", "b", 10)); e(L, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, OPTS, "topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, OPTS, "topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "topic", "b", 10)); e(L, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "~topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "~topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", "b", 10)); e(H, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, OPTS, "~topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, OPTS, "~topic", "b", 10)); e(H, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "~topic", "b", 10)); e(H, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "c", 10)); e(L, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>("c", 10)); e(H, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", "c", 10)); e(L, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, OPTS, "topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, OPTS, "topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "topic", "c", 10)); e(L, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "~topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "~topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", "c", 10)); e(H, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, OPTS, "~topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, OPTS, "~topic", "c", 10)); e(H, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "~topic", "c", 10)); e(H, "/c");
    
    ros::AdvertiseOptions opts16a, opts16b, opts16c;
    opts16a.init<Msg>("a", 10);
    opts16b.init<Msg>("a", 10);
    opts16c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, opts16a)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, opts16b)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(opts16c)); e(L, "/a");
    
    ros::AdvertiseOptions opts17a, opts17b, opts17c;
    opts17a.init<Msg>("a", 10);
    opts17b.init<Msg>("a", 10);
    opts17c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "topic", opts17a)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "topic", opts17b)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", opts17c)); e(L, "/a");
    
    ros::AdvertiseOptions opts18a, opts18b, opts18c;
    opts18a.init<Msg>("a", 10);
    opts18b.init<Msg>("a", 10);
    opts18c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS, "topic", opts18a)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS, "topic", opts18b)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "topic", opts18c)); e(L, "/a");
    
    ros::AdvertiseOptions opts19a, opts19b, opts19c;
    opts19a.init<Msg>("a", 10);
    opts19b.init<Msg>("a", 10);
    opts19c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "~topic", opts19a)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "~topic", opts19b)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", opts19c)); e(H, "/a");
    
    ros::AdvertiseOptions opts20a, opts20b, opts20c;
    opts20a.init<Msg>("a", 10);
    opts20b.init<Msg>("a", 10);
    opts20c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS, "~topic", opts20a)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS, "~topic", opts20b)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "~topic", opts20c)); e(H, "/a");
  
    // topic remapped to /d, but parameter read from ~a
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, rnh, "a", 10)); e(H, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, "a", 10)); e(L, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>("a", 10)); e(L, "/a");
    // topic remapped to /d, but parameter read from /a
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, rnh, "/a", 10)); e(H, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, "/a", 10)); e(H, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "/a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(H, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS, "foo", "a", 10)); e(O, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS, "foo", "a", 10)); e(O, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS, "foo", "a", 10)); e(O, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS_L, "foo", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS_L, "foo", "a", 10)); e(L, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS_L, "foo", "a", 10)); e(L, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS_H, "foo", "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS_H, "foo", "a", 10)); e(H, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS_H, "foo", "a", 10)); e(H, "/a");
    
    ros::WallDuration(0.25).sleep();
    
    const size_t numCallbacks = p.size();
  
    for (size_t i = 0; i < 100 && numDiagCalled < numCallbacks; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(numCallbacks, numDiagCalled);
    
    Msg pubMsg;
    
    for (size_t i = 0; i < 10; ++i)
    {
      ros::Time::setNow(ros::Time(10) + ros::Duration(0.1) * i);
      pubMsg.stamp = ros::Time(9.9) + ros::Duration(0.1) * i;
      for (const auto& pub : p)
        pub->publish(pubMsg);
    }
    
    ros::Time::setNow({11, 0});
  
    for (size_t i = 0; i < 100 && numCalled < numCallbacks * 10u; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(numCallbacks * 10u, numCalled);
    
    msg = nullptr;
    numDiagCalled = 0;
    this->getDiagUpdater().force_update();
    for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(1u, numDiagCalled);
    ASSERT_NE(nullptr, msg);
    ASSERT_EQ(numCallbacks, msg->status.size());
    
    for (size_t i = 0; i < numCallbacks; ++i)
    {
      SCOPED_TRACE("Failure was in iteration " + std::to_string(i));
      EXPECT_EQ(expectedMsgs[i].message, msg->status[i].message);
      EXPECT_EQ(expectedMsgs[i].level, msg->status[i].level);
      EXPECT_LE(7u, msg->status[i].values.size());
      EXPECT_EQ(expectedTopics[i], p[i]->getPublisher().getTopic());
    }
  }

  void testAdvertiseWithHeader()
  {
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();
    auto tnh = ros::NodeHandle(this->getNodeHandle(), "test");
    auto rnh = ros::NodeHandle(this->getNodeHandle(), "", {{"a", "d"}});
    
    this->getDiagUpdater(true);  // force creating a new updater
    
    ros::Time::setNow({10, 0});
    diagnostic_msgs::DiagnosticArrayConstPtr msg;
    size_t numDiagCalled {0};
    auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000,
      [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});
  
    size_t numCalled {0};

    using Msg = diagnostic_msgs::DiagnosticArray;
    using MsgConstPtr = diagnostic_msgs::DiagnosticArrayConstPtr;
    
    auto sub1 = nh.subscribe<Msg>("a", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub2a = pnh.subscribe<Msg>("b", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub2b = nh.subscribe<Msg>("b", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub3a = tnh.subscribe<Msg>("c", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub3b = nh.subscribe<Msg>("c", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    auto sub4 = nh.subscribe<Msg>("d", 1000, [&numCalled](const MsgConstPtr&){++numCalled;});
    
    std::vector<std::unique_ptr<cras::DiagnosedPublisher<Msg>>> p;
    std::vector<Stat> expectedMsgs;
    std::vector<std::string> expectedTopics;
    auto e = [&expectedMsgs,&expectedTopics](const Stat& msg, const ::std::string& topic)
    {
      expectedMsgs.emplace_back(msg);
      expectedTopics.emplace_back(topic);
    };

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "a", 10)); e(LP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("a", 10)); e(LP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2, "topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2, "topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "~topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "~topic", "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2, "~topic", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2, "~topic", "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "~topic", "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "b", 10)); e(LP, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "b", 10)); e(LP, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>("b", 10)); e(LP, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", "b", 10)); e(LF, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, OPTS2, "topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, OPTS2, "topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "topic", "b", 10)); e(LF, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "~topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "~topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", "b", 10)); e(HF, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, OPTS2, "~topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, OPTS2, "~topic", "b", 10)); e(HF, "/nodelet/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "~topic", "b", 10)); e(HF, "/b");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "c", 10)); e(LP, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "c", 10)); e(HP, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>("c", 10)); e(HP, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", "c", 10)); e(LF, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, OPTS2, "topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, OPTS2, "topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "topic", "c", 10)); e(LF, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "~topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "~topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", "c", 10)); e(HF, "/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, OPTS2, "~topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, OPTS2, "~topic", "c", 10)); e(HF, "/test/c");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "~topic", "c", 10)); e(HF, "/c");
    
    ros::AdvertiseOptions opts16a, opts16b, opts16c;
    opts16a.init<Msg>("a", 10);
    opts16b.init<Msg>("a", 10);
    opts16c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, opts16a)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, opts16b)); e(LP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(opts16c)); e(LP, "/a");
    
    ros::AdvertiseOptions opts17a, opts17b, opts17c;
    opts17a.init<Msg>("a", 10);
    opts17b.init<Msg>("a", 10);
    opts17c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "topic", opts17a)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "topic", opts17b)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("topic", opts17c)); e(LF, "/a");
    
    ros::AdvertiseOptions opts18a, opts18b, opts18c;
    opts18a.init<Msg>("a", 10);
    opts18b.init<Msg>("a", 10);
    opts18c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2, "topic", opts18a)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2, "topic", opts18b)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "topic", opts18c)); e(LF, "/a");
    
    ros::AdvertiseOptions opts19a, opts19b, opts19c;
    opts19a.init<Msg>("a", 10);
    opts19b.init<Msg>("a", 10);
    opts19c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "~topic", opts19a)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "~topic", opts19b)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("~topic", opts19c)); e(HF, "/a");
    
    ros::AdvertiseOptions opts20a, opts20b, opts20c;
    opts20a.init<Msg>("a", 10);
    opts20b.init<Msg>("a", 10);
    opts20c.init<Msg>("a", 10);
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2, "~topic", opts20a)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2, "~topic", opts20b)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "~topic", opts20c)); e(HF, "/a");
  
    // topic remapped to /d, but parameter read from ~a
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, rnh, "a", 10)); e(HF, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, "a", 10)); e(LP, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>("a", 10)); e(LP, "/a");
    // topic remapped to /d, but parameter read from /a
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, rnh, "/a", 10)); e(HF, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>(rnh, "/a", 10)); e(HF, "/d");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, "/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, "/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, pnh, "/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(pnh, "/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, tnh, "/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(tnh, "/a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>("/a", 10)); e(HF, "/a");
    
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2, "foo", "a", 10)); e(OO, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2, "foo", "a", 10)); e(OO, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2, "foo", "a", 10)); e(OO, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_LP, "foo", "a", 10)); e(LP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_LP, "foo", "a", 10)); e(LP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_LP, "foo", "a", 10)); e(LP, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_HP, "foo", "a", 10)); e(HP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_HP, "foo", "a", 10)); e(HP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_HP, "foo", "a", 10)); e(HP, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_LF, "foo", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_LF, "foo", "a", 10)); e(LF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_LF, "foo", "a", 10)); e(LF, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_HF, "foo", "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_HF, "foo", "a", 10)); e(HF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_HF, "foo", "a", 10)); e(HF, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_LO, "foo", "a", 10)); e(LO, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_LO, "foo", "a", 10)); e(LO, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_LO, "foo", "a", 10)); e(LO, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_HO, "foo", "a", 10)); e(HO, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_HO, "foo", "a", 10)); e(HO, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_HO, "foo", "a", 10)); e(HO, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_OP, "foo", "a", 10)); e(OP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_OP, "foo", "a", 10)); e(OP, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_OP, "foo", "a", 10)); e(OP, "/a");

    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, nh, OPTS2_OF, "foo", "a", 10)); e(OF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(nh, OPTS2_OF, "foo", "a", 10)); e(OF, "/a");
    p.emplace_back(this->advertiseDiagnosed<Msg>(OPTS2_OF, "foo", "a", 10)); e(OF, "/a");
    
    ros::WallDuration(0.25).sleep();
    
    const size_t numCallbacks = p.size();
  
    for (size_t i = 0; i < 100 && numDiagCalled < numCallbacks; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(numCallbacks, numDiagCalled);
    
    Msg pubMsg;
    
    for (size_t i = 0; i < 10; ++i)
    {
      ros::Time::setNow(ros::Time(10) + ros::Duration(0.1) * i);
      pubMsg.header.stamp = ros::Time(9.9) + ros::Duration(0.1) * i;
      for (const auto& pub : p)
        pub->publish(pubMsg);
    }
    
    ros::Time::setNow({11, 0});
  
    for (size_t i = 0; i < 100 && numCalled < numCallbacks * 10u; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(numCallbacks * 10u, numCalled);
    
    msg = nullptr;
    numDiagCalled = 0;
    this->getDiagUpdater().force_update();
    for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(1u, numDiagCalled);
    ASSERT_NE(nullptr, msg);
    ASSERT_EQ(numCallbacks, msg->status.size());
    
    for (size_t i = 0; i < numCallbacks; ++i)
    {
      SCOPED_TRACE("Failure was in iteration " + std::to_string(i));
      EXPECT_EQ(expectedMsgs[i].message, msg->status[i].message);
      EXPECT_EQ(expectedMsgs[i].level, msg->status[i].level);
      EXPECT_LE(7u, msg->status[i].values.size());
      EXPECT_EQ(expectedTopics[i], p[i]->getPublisher().getTopic());
    }
  }
  
  void testSubscribeSignaturesNoHeader2Nh2Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};
    
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();
  
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh, {}, "ns",), CRAS_TEST_SINGLE_ARG("a", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)
    
    auto pub = nh.advertise<Msg>("a", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);
  
    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }
  
    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader2Nh2Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};
    
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();
  
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh, {}, "ns",), CRAS_TEST_SINGLE_ARG("b", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
    
    auto pub = nh.advertise<Msg>("b", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);
  
    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }
  
    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader2Nh2ParamDesignatedInit()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};
    
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();
  
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh, {CRAS_TEST_SINGLE_ARG(.maxRate = 10.0, .maxDelay = 11.0)}, "ns",),
      CRAS_TEST_SINGLE_ARG("c", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)
    
    auto pub = nh.advertise<Msg>("c", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);
  
    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }
  
    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader2Nh1Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh, "ns",), CRAS_TEST_SINGLE_ARG("d", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("d", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader2Nh1Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};
    
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh, "ns",), CRAS_TEST_SINGLE_ARG("e", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("e", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);
  
    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }
  
    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader2Nh0Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh,), CRAS_TEST_SINGLE_ARG("f", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("f", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader2Nh0Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};
    
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, pnh,), CRAS_TEST_SINGLE_ARG("g", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("g", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);
  
    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }
  
    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader1Nh2Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    // (nh, {}, string, ...) is ambiguous because both SimpleTopicStatusParams and NodeHandle have no-arg constructors
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, {CRAS_TEST_SINGLE_ARG(5.0, 10.0, 0.1, 10)}, "ns",), CRAS_TEST_SINGLE_ARG("h", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("h", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader1Nh2Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    // (nh, {}, string, ...) is ambiguous because both SimpleTopicStatusParams and NodeHandle have no-arg constructors
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, {CRAS_TEST_SINGLE_ARG(5.0, 10.0, 0.1, 10, 0.1, 9.0)}, "ns",),
      CRAS_TEST_SINGLE_ARG("i", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("i", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader1Nh2ParamDesignatedInit()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, {CRAS_TEST_SINGLE_ARG(.maxRate = 10.0, .maxDelay = 11.0)}, "ns",),
      CRAS_TEST_SINGLE_ARG("j", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("j", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader1Nh1Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, "ns",), CRAS_TEST_SINGLE_ARG("k", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("k", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader1Nh1Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh, "ns",), CRAS_TEST_SINGLE_ARG("l", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("l", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader1Nh0Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh,), CRAS_TEST_SINGLE_ARG("m", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("m", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader1Nh0Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(nh,), CRAS_TEST_SINGLE_ARG("n", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("n", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }

  void testSubscribeSignaturesNoHeader0Nh2Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    // ({}, string, ...) is ambiguous because both SimpleTopicStatusParams and NodeHandle have no-arg constructors
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG({CRAS_TEST_SINGLE_ARG(5.0, 10.0, 0.1, 10)}, "ns",), CRAS_TEST_SINGLE_ARG("o", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("o", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader0Nh2Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    // ({}, string, ...) is ambiguous because both SimpleTopicStatusParams and NodeHandle have no-arg constructors
    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG({CRAS_TEST_SINGLE_ARG(5.0, 10.0, 0.1, 10, 0.1, 9.0)}, "ns",),
      CRAS_TEST_SINGLE_ARG("p", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("p", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader0Nh2ParamDesignatedInit()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG({CRAS_TEST_SINGLE_ARG(.maxRate = 10.0, .maxDelay = 11.0)}, "ns",),
      CRAS_TEST_SINGLE_ARG("q", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("q", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader0Nh1Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG("ns",), CRAS_TEST_SINGLE_ARG("r", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("r", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader0Nh1Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG("ns",), CRAS_TEST_SINGLE_ARG("s", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("s", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesNoHeader0Nh0Param()
  {
    using TestClass = CbTest;
    using Msg = std_msgs::Header;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(), CRAS_TEST_SINGLE_ARG("t", 10,),,
      m.frame_id, m->frame_id, m.getConstMessage()->frame_id)

    auto pub = nh.advertise<Msg>("t", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeSignaturesWithHeader0Nh0Param()
  {
    using TestClass = CbTestHeader;
    using Msg = diagnostic_msgs::DiagnosticArray;
    auto getFrame = [](Msg& msg) -> std::string& {return msg.header.frame_id;};

    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    CRAS_TEST_SUBSCRIBE_CALLBACKS(this->subscribeDiagnosed,  // NOLINT
      CRAS_TEST_SINGLE_ARG(), CRAS_TEST_SINGLE_ARG("u", 10,), Header,
      m.header.frame_id, m->header.frame_id, m.getConstMessage()->header.frame_id)

    auto pub = nh.advertise<Msg>("u", 100);
    auto message = Msg();
    getFrame(message) = "test";
    pub.publish(message);

    auto end = ros::WallTime::now() + ros::WallDuration(2);
    size_t numOk {0};
    while (ros::WallTime::now() < end && numOk < strs.size())
    {
      ros::spinOnce();
      numOk = 0;
      for (const auto str : strs)
        numOk += *str == getFrame(message);
      ros::WallDuration(0.01).sleep();
    }

    size_t i = 0;
    for (const auto str : strs)
    {
      SCOPED_TRACE("Error was in interation " + std::to_string(i));
      EXPECT_EQ(getFrame(message), *str);
      ++i;
    }
  }
  
  void testSubscribeConfiguration()
  {
    ros::Time::setNow({10, 0});
    
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();
    auto tnh = ros::NodeHandle(nh, "test");
    auto rnh = ros::NodeHandle(nh, "", {{"a", "d"}});

    this->getDiagUpdater(true);  // force creating a new updater

    diagnostic_msgs::DiagnosticArrayConstPtr msg;
    size_t numDiagCalled {0};
    auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000,
      [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});

    size_t numCalled {0};
    boost::function<void(const diagnostic_msgs::DiagnosticArrayConstPtr&)> cb =
      [&numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr&) {++numCalled;};
    
    std::vector<std::unique_ptr<cras::DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray>>> s;
    std::vector<Stat> expectedMsgs;
    std::vector<std::string> expectedTopics;
    auto e = [&expectedMsgs,&expectedTopics](const Stat& msg, const ::std::string& topic)
    {
      expectedMsgs.emplace_back(msg);
      expectedTopics.emplace_back(topic);
    };
    
    using M = diagnostic_msgs::DiagnosticArray;

    ros::Time::setNow({10, 0});
    
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, "a", 10, cb)); e(HF, "/a");  // 0
    s.emplace_back(this->subscribeDiagnosed<M>(nh, "a", 10, cb)); e(LP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>("a", 10, cb)); e(LP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, "topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, "topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>("topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2, "topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2, "topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, "~topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, "~topic", "a", 10, cb)); e(HF, "/a");  // 10
    s.emplace_back(this->subscribeDiagnosed<M>("~topic", "a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2, "~topic", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2, "~topic", "a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "~topic", "a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, pnh, "b", 10, cb)); e(LP, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, "b", 10, cb)); e(LP, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>("b", 10, cb)); e(LP, "/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, pnh, "topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, "topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>("topic", "b", 10, cb)); e(LF, "/b");  // 20
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, pnh, OPTS2, "topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, OPTS2, "topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "topic", "b", 10, cb)); e(LF, "/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, pnh, "~topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, "~topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>("~topic", "b", 10, cb)); e(HF, "/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, pnh, OPTS2, "~topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, OPTS2, "~topic", "b", 10, cb)); e(HF, "/nodelet/b");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "~topic", "b", 10, cb)); e(HF, "/b");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, tnh, "c", 10, cb)); e(LP, "/test/c");  // 30
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, "c", 10, cb)); e(HP, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>("c", 10, cb)); e(HP, "/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, tnh, "topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, "topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>("topic", "c", 10, cb)); e(LF, "/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, tnh, OPTS2, "topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, OPTS2, "topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "topic", "c", 10, cb)); e(LF, "/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, tnh, "~topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, "~topic", "c", 10, cb)); e(HF, "/test/c");  // 40
    s.emplace_back(this->subscribeDiagnosed<M>("~topic", "c", 10, cb)); e(HF, "/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, tnh, OPTS2, "~topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, OPTS2, "~topic", "c", 10, cb)); e(HF, "/test/c");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "~topic", "c", 10, cb)); e(HF, "/c");
    
    ros::SubscribeOptions opts16a, opts16b, opts16c;
    opts16a.init<M>("a", 10, cb);
    opts16b.init<M>("a", 10, cb);
    opts16c.init<M>("a", 10, cb);
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, opts16a)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, opts16b)); e(LP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(opts16c)); e(LP, "/a");
    
    ros::SubscribeOptions opts17a, opts17b, opts17c;
    opts17a.init<M>("a", 10, cb);
    opts17b.init<M>("a", 10, cb);
    opts17c.init<M>("a", 10, cb);
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, "topic", opts17a)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, "topic", opts17b)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>("topic", opts17c)); e(LF, "/a");  // 50
    
    ros::SubscribeOptions opts18a, opts18b, opts18c;
    opts18a.init<M>("a", 10, cb);
    opts18b.init<M>("a", 10, cb);
    opts18c.init<M>("a", 10, cb);
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2, "topic", opts18a)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2, "topic", opts18b)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "topic", opts18c)); e(LF, "/a");
    
    ros::SubscribeOptions opts19a, opts19b, opts19c;
    opts19a.init<M>("a", 10, cb);
    opts19b.init<M>("a", 10, cb);
    opts19c.init<M>("a", 10, cb);
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, "~topic", opts19a)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, "~topic", opts19b)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>("~topic", opts19c)); e(HF, "/a");
    
    ros::SubscribeOptions opts20a, opts20b, opts20c;
    opts20a.init<M>("a", 10, cb);
    opts20b.init<M>("a", 10, cb);
    opts20c.init<M>("a", 10, cb);
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2, "~topic", opts20a)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2, "~topic", opts20b)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "~topic", opts20c)); e(HF, "/a");
  
    // topic remapped to /d, but parameter read from ~a
    s.emplace_back(this->subscribeDiagnosed<M>(rnh, rnh, "a", 10, cb)); e(HF, "/d");  // 60
    s.emplace_back(this->subscribeDiagnosed<M>(rnh, "a", 10, cb)); e(LP, "/d");
    s.emplace_back(this->subscribeDiagnosed<M>("a", 10, cb)); e(LP, "/a");
    // topic remapped to /d, but parameter read from /a
    s.emplace_back(this->subscribeDiagnosed<M>(rnh, rnh, "/a", 10, cb)); e(HF, "/d");
    s.emplace_back(this->subscribeDiagnosed<M>(rnh, "/a", 10, cb)); e(HF, "/d");
    s.emplace_back(this->subscribeDiagnosed<M>("/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, "/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, "/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>("/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, pnh, "/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(pnh, "/a", 10, cb)); e(HF, "/a");  // 70
    s.emplace_back(this->subscribeDiagnosed<M>("/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, tnh, "/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(tnh, "/a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>("/a", 10, cb)); e(HF, "/a");
    
    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2, "foo", "a", 10, cb)); e(OO, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2, "foo", "a", 10, cb)); e(OO, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2, "foo", "a", 10, cb)); e(OO, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_LP, "foo", "a", 10, cb)); e(LP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_LP, "foo", "a", 10, cb)); e(LP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_LP, "foo", "a", 10, cb)); e(LP, "/a");  // 80

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_HP, "foo", "a", 10, cb)); e(HP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_HP, "foo", "a", 10, cb)); e(HP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_HP, "foo", "a", 10, cb)); e(HP, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_LF, "foo", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_LF, "foo", "a", 10, cb)); e(LF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_LF, "foo", "a", 10, cb)); e(LF, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_HF, "foo", "a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_HF, "foo", "a", 10, cb)); e(HF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_HF, "foo", "a", 10, cb)); e(HF, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_LO, "foo", "a", 10, cb)); e(LO, "/a"); // 90
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_LO, "foo", "a", 10, cb)); e(LO, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_LO, "foo", "a", 10, cb)); e(LO, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_HO, "foo", "a", 10, cb)); e(HO, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_HO, "foo", "a", 10, cb)); e(HO, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_HO, "foo", "a", 10, cb)); e(HO, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_OP, "foo", "a", 10, cb)); e(OP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_OP, "foo", "a", 10, cb)); e(OP, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_OP, "foo", "a", 10, cb)); e(OP, "/a");

    s.emplace_back(this->subscribeDiagnosed<M>(nh, nh, OPTS2_OF, "foo", "a", 10, cb)); e(OF, "/a");
    s.emplace_back(this->subscribeDiagnosed<M>(nh, OPTS2_OF, "foo", "a", 10, cb)); e(OF, "/a");  // 100
    s.emplace_back(this->subscribeDiagnosed<M>(OPTS2_OF, "foo", "a", 10, cb)); e(OF, "/a");
    
    const auto numCallbacks = s.size();
    
    auto pub1 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 1000);
    auto pub2a = pnh.advertise<diagnostic_msgs::DiagnosticArray>("b", 1000);
    auto pub2b = nh.advertise<diagnostic_msgs::DiagnosticArray>("b", 1000);
    auto pub3a = tnh.advertise<diagnostic_msgs::DiagnosticArray>("c", 1000);
    auto pub3b = nh.advertise<diagnostic_msgs::DiagnosticArray>("c", 1000);
    auto pub4 = nh.advertise<diagnostic_msgs::DiagnosticArray>("d", 1000);

    for (size_t i = 0; i < 100 && numDiagCalled < numCallbacks; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(numCallbacks, numDiagCalled);
    
    ros::WallDuration(0.25).sleep();
    
    auto message = diagnostic_msgs::DiagnosticArray();
    message.header.frame_id = "test";
    numCalled = 0;
    for (size_t i = 0; i < 10; ++i)
    {
      ros::Time::setNow(ros::Time(10) + ros::Duration(0.1) * i);
      message.header.stamp = ros::Time(9.9) + ros::Duration(0.1) * i;
      pub1.publish(message);
      pub2a.publish(message);
      pub2b.publish(message);
      pub3a.publish(message);
      pub3b.publish(message);
      pub4.publish(message);

      auto end = ros::WallTime::now() + ros::WallDuration(1);
      while (numCalled < numCallbacks * (i + 1) && ros::WallTime::now() < end)
      {
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
      }
    }

    ros::Time::setNow({11, 0});
    EXPECT_EQ(numCallbacks * 10u, numCalled);
    
    msg = nullptr;
    numDiagCalled = 0;
    this->getDiagUpdater().force_update();
    for (size_t i = 0; i < 100 && numDiagCalled == 0; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_EQ(1u, numDiagCalled);
    ASSERT_NE(nullptr, msg);
    ASSERT_EQ(numCallbacks, msg->status.size());
    
    for (size_t i = 0; i < numCallbacks; ++i)
    {
      SCOPED_TRACE("Failure was in iteration " + std::to_string(i));
      EXPECT_EQ(expectedMsgs[i].message, msg->status[i].message);
      EXPECT_EQ(expectedMsgs[i].level, msg->status[i].level);
      EXPECT_LE(7u, msg->status[i].values.size());
      EXPECT_EQ(expectedTopics[i], s[i]->getSubscriber().getTopic());
    }
  }
  
  void testDiagTimer()
  {
    ros::Time::setNow({10, 0});
    
    auto nh = this->getNodeHandle();
    this->getDiagUpdater(true);  // force creating a new updater

    diagnostic_msgs::DiagnosticArrayConstPtr msg;
    size_t numDiagCalled {0};
    auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000,
      [&msg, &numDiagCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) { msg = m; numDiagCalled++;});

    size_t numCallbacks {10};
    size_t minCallbacks {numCallbacks - 2u};
    
    this->startDiagTimer(nh);
    
    ros::WallDuration(0.05).sleep();
    
    for (size_t i = 0; i < numCallbacks; ++i)
    {
      ros::Time::setNow(ros::Time(10 + i, 1000 * i));
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }

    for (size_t i = 0; i < 100 && numDiagCalled < minCallbacks; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_LE(minCallbacks, numDiagCalled);
    
    this->stopDiagTimer();
    
    for (size_t i = 0; i < numCallbacks; ++i)
    {
      ros::Time::setNow(ros::Time(20 + i, 2000 * i));
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }

    EXPECT_GE(numCallbacks, numDiagCalled);

    numDiagCalled = 0;
    this->startDiagTimer(nh);

    ros::WallDuration(0.05).sleep();

    for (size_t i = 0; i < numCallbacks; ++i)
    {
      ros::Time::setNow(ros::Time(30 + i, 3000 * i));
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }

    for (size_t i = 0; i < 100 && numDiagCalled < minCallbacks; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    EXPECT_LE(minCallbacks, numDiagCalled);
  }

};

NodeletDiagnosticsTest* getNodelet()
{
  static NodeletDiagnosticsTest* nodelet {nullptr};
  if (nodelet)
    return nodelet;
  // we have to manually manage the lifetime of nodelet; if shared_ptr were used here, ros shutdown would destroy the
  // contained node handles before the nodelet is destroyed, and then nodelet would try to destroy them again when it
  // is being destroyed itself
  nodelet = new NodeletDiagnosticsTest;
  nodelet->init("/nodelet", {}, {});
  return nodelet;
}

TEST(NodeletDiagnostics, AdvertiseDiagnosedNoHeader)  // NOLINT
{
  auto* n = getNodelet();
  n->testAdvertiseNoHeader();
}

TEST(NodeletDiagnostics, AdvertiseDiagnosedWithHeader)  // NOLINT
{
  auto* n = getNodelet();
  n->testAdvertiseWithHeader();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader2Nh2Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader2Nh2Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader2Nh2Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader2Nh2Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader2Nh2ParamDesignatedInit)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader2Nh2ParamDesignatedInit();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader2Nh1Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader2Nh1Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader2Nh1Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader2Nh1Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader2Nh0Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader2Nh0Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader2Nh0Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader2Nh0Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader1Nh2Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader1Nh2Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader1Nh2Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader1Nh2Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader1Nh2ParamDesignatedInit)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader1Nh2ParamDesignatedInit();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader1Nh1Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader1Nh1Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader1Nh1Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader1Nh1Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader1Nh0Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader1Nh0Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader1Nh0Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader1Nh0Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader0Nh2Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader0Nh2Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader0Nh2Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader0Nh2Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader0Nh2ParamDesignatedInit)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader0Nh2ParamDesignatedInit();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader0Nh1Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader0Nh1Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader0Nh1Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader0Nh1Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesNoHeader0Nh0Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesNoHeader0Nh0Param();
}

TEST(NodeletDiagnostics, SubscribeSignaturesWithHeader0Nh0Param)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeSignaturesWithHeader0Nh0Param();
}

TEST(NodeletDiagnostics, SubscribeConfiguration)  // NOLINT
{
  auto* n = getNodelet();
  n->testSubscribeConfiguration();
}

TEST(NodeletDiagnostics, TestDiagTimer)  // NOLINT
{
  auto* n = getNodelet();
  n->testDiagTimer();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_nodelet_diagnostics");
  ros::start();

  const auto& result = RUN_ALL_TESTS();

  // manually destroy nodelet; see the comment in getNodelet()
  delete getNodelet();

  return result;
}