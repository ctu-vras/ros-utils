/**
 * \file
 * \brief Test for generic_lazy_pubsub.hpp .
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

// HACK: we need to access TopicManager::subscriptions_
#define private public
#include <ros/topic_manager.h>
#undef private

#include <memory>
#include <string>

#include <ros/ros.h>
#include <ros/subscription.h>
#include <std_msgs/Header.h>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>

void spin(double duration, double wait)
{
  double waited = 0;
  while (waited < duration)
  {
    ros::spinOnce();
    ros::WallDuration(wait).sleep();
    waited += wait;
  }
}

void spin(double duration)
{
  spin(duration, duration / 1000.0);
}

size_t getNumSubscriptions(const std::string& topic)
{
  size_t num = 0;
  auto subs = ros::TopicManager::instance()->subscriptions_;
  for (const auto& sub : subs)
  {
    if (sub->getName() == topic)
    {
      num += sub->getNumCallbacks();
    }
  }
  return num;
}

class TestLazyPubSub : public cras::GenericLazyPubSub<>
{
public:
  TestLazyPubSub(const ::std::string& topicIn, const ::std::string& topicOut, const ::ros::NodeHandle& nh = {},
    const size_t inQueueSize = 10, const size_t outQueueSize = 10) :
      cras::GenericLazyPubSub<>(topicIn, topicOut, nh, inQueueSize, outQueueSize)
  {
  }

  ros::Publisher& getPub()
  {
    return this->pub;
  }

  ros::Subscriber& getSub()
  {
    return this->sub;
  }

protected:
  void processMessage(const ros::MessageEvent<topic_tools::ShapeShifter const>& event) override
  {
    GenericLazyPubSub::processMessage(event);
    this->numProcessed++;
  }

public:
  using cras::LazySubscriberBase<>::updateSubscription;
  size_t numProcessed {0};
};

TEST(GenericLazyPubSub, OverallTest)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test1");
  const auto ns = pnh.getNamespace();
  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

  size_t numOutReceived = 0;

  spin(0.1);
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(true, lazySub->isSubscribed());

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(0, lazySub->numProcessed);
    EXPECT_EQ(0, numOutReceived);
    EXPECT_FALSE(lazySub->getPub());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    inPub.publish(std_msgs::Header());

    spin(0.2);
    EXPECT_EQ(1, lazySub->numProcessed);
    EXPECT_EQ(1, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
  }

  spin(0.1);
  EXPECT_EQ(1, lazySub->numProcessed);
  EXPECT_EQ(1, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());

  inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(1, lazySub->numProcessed);
  EXPECT_EQ(1, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);
    auto outSub2 = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(1, lazySub->numProcessed);
    EXPECT_EQ(1, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(2, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    // Simulate connection from a second node; this can't be done from within a single executable.
    // This should not have any effect as we're only interested in the first or last subscriber.
    lazySub->updateSubscription();

    spin(0.1);
    EXPECT_EQ(1, lazySub->numProcessed);
    EXPECT_EQ(1, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(2, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    inPub.publish(std_msgs::Header());
    inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(3, lazySub->numProcessed);
    EXPECT_EQ(5, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(2, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
  }

  spin(0.1);
  EXPECT_EQ(3, lazySub->numProcessed);
  EXPECT_EQ(5, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());

  inPub.publish(std_msgs::Header());
  inPub.publish(std_msgs::Header());
  inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(3, lazySub->numProcessed);
  EXPECT_EQ(5, numOutReceived);
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(3, lazySub->numProcessed);
    EXPECT_EQ(5, numOutReceived);
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    // Delete the pubsub object - it should disconnect the subscriber.
    lazySub.reset();

    spin(0.1);
    EXPECT_EQ(5, numOutReceived);
    EXPECT_EQ(0, inPub.getNumSubscribers());
    EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
  }
}

TEST(GenericLazyPubSub, StartSequenceNothing)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test2");
  const auto ns = pnh.getNamespace();
  size_t numOutReceived = 0;

  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());

  {
    auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

    spin(0.1);
    EXPECT_EQ(0, lazySub->numProcessed);
    EXPECT_EQ(0, numOutReceived);
    EXPECT_FALSE(lazySub->getPub());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(0, lazySub->numProcessed);
    EXPECT_EQ(0, numOutReceived);
    EXPECT_FALSE(lazySub->getPub());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    for (size_t i = 0; i < 10; ++i)
      inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(10, lazySub->numProcessed);
    EXPECT_EQ(10, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
  }

  spin(0.1);
  EXPECT_EQ(10, lazySub->numProcessed);
  EXPECT_EQ(10, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());
}

TEST(GenericLazyPubSub, StartSequenceIn)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test3");
  const auto ns = pnh.getNamespace();
  size_t numOutReceived = 0;

  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

  spin(0.1);
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(0, lazySub->numProcessed);
    EXPECT_EQ(0, numOutReceived);
    EXPECT_FALSE(lazySub->getPub());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    for (size_t i = 0; i < 10; ++i)
      inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(10, lazySub->numProcessed);
    EXPECT_EQ(10, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
  }

  spin(0.1);
  EXPECT_EQ(10, lazySub->numProcessed);
  EXPECT_EQ(10, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());
}

TEST(GenericLazyPubSub, StartSequenceOut)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test4");
  const auto ns = pnh.getNamespace();
  size_t numOutReceived = 0;

  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

  spin(0.1);
  EXPECT_EQ(0, outSub.getNumPublishers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(0, outSub.getNumPublishers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());

  {
    auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

    spin(0.1);
    EXPECT_EQ(0, lazySub->numProcessed);
    EXPECT_EQ(0, numOutReceived);
    EXPECT_FALSE(lazySub->getPub());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(0, outSub.getNumPublishers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());

    for (size_t i = 0; i < 10; ++i)
      inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(10, lazySub->numProcessed);
    EXPECT_EQ(10, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, outSub.getNumPublishers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
  }

  spin(0.1);
  EXPECT_EQ(10, lazySub->numProcessed);
  EXPECT_EQ(10, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(1, outSub.getNumPublishers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
}

TEST(GenericLazyPubSub, StartSequenceInOut)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test5");
  const auto ns = pnh.getNamespace();
  size_t numOutReceived = 0;

  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);
  auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

  spin(0.1);
  EXPECT_EQ(0, outSub.getNumPublishers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_EQ(0, numOutReceived);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(0, outSub.getNumPublishers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());

  for (size_t i = 0; i < 10; ++i)
    inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(10, lazySub->numProcessed);
  EXPECT_EQ(10, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, outSub.getNumPublishers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
}

TEST(GenericLazyPubSub, StartSequenceNoOut)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test5");
  const auto ns = pnh.getNamespace();

  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

  spin(0.1);
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());

  inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(1, lazySub->numProcessed);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());
}

TEST(GenericLazyPubSub, QueueSize)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test8");
  size_t numOutReceived = 0;

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);
  auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);
  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh, 5, 3);

  spin(0.1);

  for (size_t i = 0; i < 10; ++i)
    inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_LE(5, lazySub->numProcessed);
  EXPECT_GE(6, lazySub->numProcessed);
  EXPECT_LE(5, numOutReceived);  // ideally would be 3, but there's no way to test it with local-only connections
  EXPECT_GE(6, numOutReceived);
}

TEST(GenericLazyPubSub, MultiLatchedInputs)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test9");
  size_t numOutReceived = 0;

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  // Topic /latch has 5 latched publishers in the rostest launch file. This test tests whether the relay works well for
  // a large inrush of messages at the beginning.
  auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);
  auto lazySub = std::make_unique<TestLazyPubSub>("/latched", "out", pnh);

  spin(0.5);

  EXPECT_EQ(5, lazySub->numProcessed);
  EXPECT_EQ(5, numOutReceived);
}

class TestAdvertiseOptions : public TestLazyPubSub
{
public:
  TestAdvertiseOptions(const std::string& topicIn, const std::string& topicOut, const ros::NodeHandle& nh) :
    TestLazyPubSub(topicIn, topicOut, nh)
  {
  }

protected:
  ros::AdvertiseOptions createAdvertiseOptions(const ros::MessageEvent<topic_tools::ShapeShifter const>& event) override
  {
    auto opts = GenericLazyPubSub::createAdvertiseOptions(event);
    opts.topic = "topic_from_opts";
    return opts;
  }
};

TEST(GenericLazyPubSub, OverrideAdvertiseOptions)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test10");
  size_t numOutReceived = 0;

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
  {
    ++numOutReceived;
  };

  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);
  auto outSub = pnh.subscribe<std_msgs::Header>("topic_from_opts", 10, outCb);
  auto lazySub = std::make_unique<TestAdvertiseOptions>("in", "out", pnh);

  spin(0.1);

  for (size_t i = 0; i < 10; ++i)
    inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(10, lazySub->numProcessed);
  EXPECT_EQ(10, numOutReceived);
  EXPECT_FALSE(lazySub->getPub().isLatched());
}

TEST(GenericLazyPubSub, LatchIsRetained)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test11");
  size_t numOutReceived = 0;

  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
  {
    ++numOutReceived;
  };

  auto inPub = pnh.advertise<std_msgs::Header>("in", 10, true);
  auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);
  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);

  for (size_t i = 0; i < 10; ++i)
    inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(10, lazySub->numProcessed);
  EXPECT_EQ(10, numOutReceived);
  EXPECT_TRUE(lazySub->getPub().isLatched());
}

TEST(GenericLazyPubSub, SetLazy)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test6");
  const auto ns = pnh.getNamespace();
  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

  size_t numOutReceived = 0;
  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };

  auto lazySub = std::make_unique<TestLazyPubSub>("in", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_EQ(0, numOutReceived);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
  EXPECT_EQ(true, lazySub->isLazy());

  lazySub->setLazy(false);

  spin(0.1);
  EXPECT_EQ(0, lazySub->numProcessed);
  EXPECT_EQ(0, numOutReceived);
  EXPECT_FALSE(lazySub->getPub());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
  EXPECT_EQ(false, lazySub->isLazy());

  inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(1, lazySub->numProcessed);
  EXPECT_EQ(0, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
  EXPECT_EQ(false, lazySub->isLazy());

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(1, lazySub->numProcessed);
    EXPECT_EQ(0, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
    EXPECT_EQ(false, lazySub->isLazy());

    inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(2, lazySub->numProcessed);
    EXPECT_EQ(1, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
    EXPECT_EQ(false, lazySub->isLazy());
  }

  spin(0.1);
  EXPECT_EQ(2, lazySub->numProcessed);
  EXPECT_EQ(1, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
  EXPECT_EQ(false, lazySub->isLazy());

  lazySub->setLazy(true);

  spin(0.1);
  EXPECT_EQ(2, lazySub->numProcessed);
  EXPECT_EQ(1, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());
  EXPECT_EQ(true, lazySub->isLazy());

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(2, lazySub->numProcessed);
    EXPECT_EQ(1, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
    EXPECT_EQ(true, lazySub->isLazy());

    inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(3, lazySub->numProcessed);
    EXPECT_EQ(2, numOutReceived);
    ASSERT_TRUE(lazySub->getPub());
    EXPECT_EQ(1, lazySub->getPub().getNumSubscribers());
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
    EXPECT_EQ(true, lazySub->isSubscribed());
    EXPECT_EQ(true, lazySub->isLazy());
  }

  spin(0.1);
  EXPECT_EQ(3, lazySub->numProcessed);
  EXPECT_EQ(2, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(0, inPub.getNumSubscribers());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(false, lazySub->isSubscribed());
  EXPECT_EQ(true, lazySub->isLazy());

  lazySub->setLazy(false);

  spin(0.1);
  EXPECT_EQ(3, lazySub->numProcessed);
  EXPECT_EQ(2, numOutReceived);
  ASSERT_TRUE(lazySub->getPub());
  EXPECT_EQ(0, lazySub->getPub().getNumSubscribers());
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
  EXPECT_EQ(true, lazySub->isSubscribed());
  EXPECT_EQ(false, lazySub->isLazy());
}

TEST(GenericLazyPubSub, Chain)  // NOLINT
{
  ros::NodeHandle pnh({"test"}, "test7");
  const auto ns = pnh.getNamespace();
  auto inPub = pnh.advertise<std_msgs::Header>("in", 10);

  size_t numOutReceived = 0;
  boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
    {
      ++numOutReceived;
    };
  TestLazyPubSub lazySub1("in", "inter1", pnh);
  TestLazyPubSub lazySub2("inter1", "inter2", pnh);
  TestLazyPubSub lazySub3("inter2", "out", pnh);

  spin(0.1);
  EXPECT_EQ(0, lazySub1.numProcessed);
  EXPECT_EQ(0, lazySub2.numProcessed);
  EXPECT_EQ(0, lazySub3.numProcessed);
  EXPECT_EQ(0, numOutReceived);
  EXPECT_EQ(1, inPub.getNumSubscribers());
  EXPECT_FALSE(lazySub1.getPub());
  EXPECT_FALSE(lazySub2.getPub());
  EXPECT_FALSE(lazySub3.getPub());
  EXPECT_EQ(true, lazySub1.isSubscribed());
  EXPECT_EQ(true, lazySub2.isSubscribed());
  EXPECT_EQ(true, lazySub3.isSubscribed());
  EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/inter1"));
  EXPECT_EQ(1, getNumSubscriptions(ns + "/inter2"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  {
    auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

    spin(0.1);
    EXPECT_EQ(0, lazySub1.numProcessed);
    EXPECT_EQ(0, lazySub2.numProcessed);
    EXPECT_EQ(0, lazySub3.numProcessed);
    EXPECT_EQ(0, numOutReceived);
    EXPECT_EQ(1, inPub.getNumSubscribers());
    EXPECT_FALSE(lazySub1.getPub());
    EXPECT_FALSE(lazySub2.getPub());
    EXPECT_FALSE(lazySub3.getPub());
    EXPECT_EQ(true, lazySub1.isSubscribed());
    EXPECT_EQ(true, lazySub2.isSubscribed());
    EXPECT_EQ(true, lazySub3.isSubscribed());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/inter1"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/inter2"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));

    inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(1, lazySub1.numProcessed);
    EXPECT_EQ(1, lazySub2.numProcessed);
    EXPECT_EQ(1, lazySub3.numProcessed);
    EXPECT_EQ(1, numOutReceived);
    EXPECT_EQ(1, inPub.getNumSubscribers());
    ASSERT_TRUE(lazySub1.getPub());
    EXPECT_EQ(1, lazySub1.getPub().getNumSubscribers());
    ASSERT_TRUE(lazySub2.getPub());
    EXPECT_EQ(1, lazySub2.getPub().getNumSubscribers());
    ASSERT_TRUE(lazySub3.getPub());
    EXPECT_EQ(true, lazySub1.isSubscribed());
    EXPECT_EQ(true, lazySub2.isSubscribed());
    EXPECT_EQ(true, lazySub3.isSubscribed());
    EXPECT_EQ(1, lazySub3.getPub().getNumSubscribers());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/inter1"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/inter2"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));

    inPub.publish(std_msgs::Header());

    spin(0.1);
    EXPECT_EQ(2, lazySub1.numProcessed);
    EXPECT_EQ(2, lazySub2.numProcessed);
    EXPECT_EQ(2, lazySub3.numProcessed);
    EXPECT_EQ(2, numOutReceived);
    EXPECT_EQ(1, inPub.getNumSubscribers());
    ASSERT_TRUE(lazySub1.getPub());
    EXPECT_EQ(1, lazySub1.getPub().getNumSubscribers());
    ASSERT_TRUE(lazySub2.getPub());
    EXPECT_EQ(1, lazySub2.getPub().getNumSubscribers());
    ASSERT_TRUE(lazySub3.getPub());
    EXPECT_EQ(1, lazySub3.getPub().getNumSubscribers());
    EXPECT_EQ(true, lazySub1.isSubscribed());
    EXPECT_EQ(true, lazySub2.isSubscribed());
    EXPECT_EQ(true, lazySub3.isSubscribed());
    EXPECT_EQ(1, getNumSubscriptions(ns + "/in"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/inter1"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/inter2"));
    EXPECT_EQ(1, getNumSubscriptions(ns + "/out"));
  }

  spin(0.1);
  EXPECT_EQ(2, lazySub1.numProcessed);
  EXPECT_EQ(2, lazySub2.numProcessed);
  EXPECT_EQ(2, lazySub3.numProcessed);
  EXPECT_EQ(2, numOutReceived);
  EXPECT_EQ(0, inPub.getNumSubscribers());
  ASSERT_TRUE(lazySub1.getPub());
  EXPECT_EQ(0, lazySub1.getPub().getNumSubscribers());
  ASSERT_TRUE(lazySub2.getPub());
  EXPECT_EQ(0, lazySub2.getPub().getNumSubscribers());
  ASSERT_TRUE(lazySub3.getPub());
  EXPECT_EQ(0, lazySub3.getPub().getNumSubscribers());
  EXPECT_EQ(false, lazySub1.isSubscribed());
  EXPECT_EQ(false, lazySub2.isSubscribed());
  EXPECT_EQ(false, lazySub3.isSubscribed());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/inter1"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/inter2"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));

  inPub.publish(std_msgs::Header());

  spin(0.1);
  EXPECT_EQ(2, lazySub1.numProcessed);
  EXPECT_EQ(2, lazySub2.numProcessed);
  EXPECT_EQ(2, lazySub3.numProcessed);
  EXPECT_EQ(2, numOutReceived);
  EXPECT_EQ(0, inPub.getNumSubscribers());
  ASSERT_TRUE(lazySub1.getPub());
  EXPECT_EQ(0, lazySub1.getPub().getNumSubscribers());
  ASSERT_TRUE(lazySub2.getPub());
  EXPECT_EQ(0, lazySub2.getPub().getNumSubscribers());
  ASSERT_TRUE(lazySub3.getPub());
  EXPECT_EQ(0, lazySub3.getPub().getNumSubscribers());
  EXPECT_EQ(false, lazySub1.isSubscribed());
  EXPECT_EQ(false, lazySub2.isSubscribed());
  EXPECT_EQ(false, lazySub3.isSubscribed());
  EXPECT_EQ(0, getNumSubscriptions(ns + "/in"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/inter1"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/inter2"));
  EXPECT_EQ(0, getNumSubscriptions(ns + "/out"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_generic_lazy_pubsub");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();
  // This prevents the ROS background threads from shutting down after the first test's nodehandles disappear
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
