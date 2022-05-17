/**
 * \file
 * \brief Test for lazy_subscriber.hpp .
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

// HACK: we need to access TopicManager::subscriptions_
#define private public
#include <ros/topic_manager.h>
#undef private

#include <ros/ros.h>
#include <ros/subscription.h>
#include <std_msgs/Header.h>

#include <cras_topic_tools/lazy_subscriber.hpp>

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

template<typename M>
class TestLazySubscriber : public cras::LazySubscriber<M>
{
public:
	TestLazySubscriber(const ros::Publisher& pub, const typename cras::LazySubscriber<M>::ConnectFn& connectFn,
    const typename cras::LazySubscriber<M>::DisconnectFn& disconnectFn = [](ros::Subscriber& sub) { sub.shutdown(); },
    cras::LogHelperPtr logHelper = std::make_shared<cras::NodeLogHelper>()) :
			cras::LazySubscriber<M>(pub, connectFn, disconnectFn, logHelper)
	{
	}
	using cras::LazySubscriber<M>::connectCb;
};

TEST(LazySubscriber, Test)  // NOLINT
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh({"tmp"}, "my");
	auto outPub = pnh.advertise<std_msgs::Header>("out", 10);
	auto inPub = nh.advertise<std_msgs::Header>("in", 10);

	size_t numOutSubscribes = 0;
	size_t numInReceived = 0;
	size_t numOutReceived = 0;
	boost::function<void(const std_msgs::Header&)> relayCb = [&](const std_msgs::Header& h)
		{
		  ++numInReceived;
			outPub.publish(h);
		};
	boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
		{
		  ++numOutReceived;
		};
	
	auto lazySub = std::make_unique<TestLazySubscriber<std_msgs::Header>>(outPub, [&](ros::Subscriber& sub)
	  {
		  ++numOutSubscribes;
			sub = nh.subscribe<std_msgs::Header>("in", 10, relayCb);
		});
	
	spin(0.1);
	EXPECT_EQ(0, numOutSubscribes);
	EXPECT_EQ(0, numInReceived);
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, getNumSubscriptions("/i"));
	EXPECT_EQ(false, lazySub->isSubscribed());

	{
		auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

		spin(0.1);
		EXPECT_EQ(1, numOutSubscribes);
		EXPECT_EQ(0, numInReceived);
		EXPECT_EQ(0, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, getNumSubscriptions("/in"));
		EXPECT_EQ(1, getNumSubscriptions("/tmp/my/out"));
		EXPECT_EQ(true, lazySub->isSubscribed());
		
		inPub.publish(std_msgs::Header());

		spin(0.1);
		EXPECT_EQ(1, numOutSubscribes);
		EXPECT_EQ(1, numInReceived);
		EXPECT_EQ(1, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, getNumSubscriptions("/in"));
		EXPECT_EQ(1, getNumSubscriptions("/tmp/my/out"));
		EXPECT_EQ(true, lazySub->isSubscribed());
	}

	spin(0.1);
	EXPECT_EQ(1, numOutSubscribes);
	EXPECT_EQ(1, numInReceived);
	EXPECT_EQ(1, numOutReceived);
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, getNumSubscriptions("/in"));
	EXPECT_EQ(0, getNumSubscriptions("/tmp/my/out"));
	EXPECT_EQ(false, lazySub->isSubscribed());

	inPub.publish(std_msgs::Header());

	spin(0.1);
	EXPECT_EQ(1, numOutSubscribes);
	EXPECT_EQ(1, numInReceived);
	EXPECT_EQ(1, numOutReceived);
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, getNumSubscriptions("/in"));
	EXPECT_EQ(0, getNumSubscriptions("/tmp/my/out"));
	EXPECT_EQ(false, lazySub->isSubscribed());

	{
		auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);
		auto outSub2 = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

		spin(0.1);
		EXPECT_EQ(2, numOutSubscribes);
		EXPECT_EQ(1, numInReceived);
		EXPECT_EQ(1, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, getNumSubscriptions("/in"));
		EXPECT_EQ(2, getNumSubscriptions("/tmp/my/out"));
		EXPECT_EQ(true, lazySub->isSubscribed());
		
		// Simulate connection from a second node; this can't be done from within a single executable.
		// This should not have any effect as we're only interested in the first or last subscriber.
		lazySub->connectCb({{}});

		spin(0.1);
		EXPECT_EQ(2, numOutSubscribes);
		EXPECT_EQ(1, numInReceived);
		EXPECT_EQ(1, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, getNumSubscriptions("/in"));
		EXPECT_EQ(2, getNumSubscriptions("/tmp/my/out"));
		EXPECT_EQ(true, lazySub->isSubscribed());

		inPub.publish(std_msgs::Header());
		inPub.publish(std_msgs::Header());

		spin(0.1);
		EXPECT_EQ(2, numOutSubscribes);
		EXPECT_EQ(3, numInReceived);
		EXPECT_EQ(5, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, getNumSubscriptions("/in"));
		EXPECT_EQ(2, getNumSubscriptions("/tmp/my/out"));
		EXPECT_EQ(true, lazySub->isSubscribed());
	}

	spin(0.1);
	EXPECT_EQ(2, numOutSubscribes);
	EXPECT_EQ(3, numInReceived);
	EXPECT_EQ(5, numOutReceived);
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, getNumSubscriptions("/in"));
	EXPECT_EQ(0, getNumSubscriptions("/tmp/my/out"));
	EXPECT_EQ(false, lazySub->isSubscribed());

	inPub.publish(std_msgs::Header());
	inPub.publish(std_msgs::Header());
	inPub.publish(std_msgs::Header());

	spin(0.1);
	EXPECT_EQ(2, numOutSubscribes);
	EXPECT_EQ(3, numInReceived);
	EXPECT_EQ(5, numOutReceived);
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, getNumSubscriptions("/in"));
	EXPECT_EQ(0, getNumSubscriptions("/tmp/my/out"));
	EXPECT_EQ(false, lazySub->isSubscribed());

	{
		auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

		spin(0.1);
		EXPECT_EQ(3, numOutSubscribes);
		EXPECT_EQ(3, numInReceived);
		EXPECT_EQ(5, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, getNumSubscriptions("/in"));
		EXPECT_EQ(1, getNumSubscriptions("/tmp/my/out"));
		EXPECT_EQ(true, lazySub->isSubscribed());

		// Delete the pubsub object - it should disconnect the subscriber.
		lazySub.reset();

		spin(0.1);
		EXPECT_EQ(3, numOutSubscribes);
		EXPECT_EQ(3, numInReceived);
		EXPECT_EQ(5, numOutReceived);
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(0, inPub.getNumSubscribers());
		EXPECT_EQ(0, getNumSubscriptions("/in"));
		EXPECT_EQ(1, getNumSubscriptions("/tmp/my/out"));
	}
}

TEST(LazySubscriber, Chain)  // NOLINT
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh({"tmp"}, "my");
	auto outPub = pnh.advertise<std_msgs::Header>("out", 10);
	auto inPub = nh.advertise<std_msgs::Header>("in", 10);
	auto inter1Pub = nh.advertise<std_msgs::Header>("inter1", 10);
	auto inter2Pub = nh.advertise<std_msgs::Header>("inter2", 10);

	size_t numInter1Subscribes = 0;
	size_t numInter2Subscribes = 0;
	size_t numOutSubscribes = 0;
	size_t numInReceived = 0;
	size_t numInter1Received = 0;
	size_t numInter2Received = 0;
	size_t numOutReceived = 0;
	boost::function<void(const std_msgs::Header&)> relayToInter1Cb = [&](const std_msgs::Header& h)
		{
		  ++numInReceived;
			inter1Pub.publish(h);
		};
	boost::function<void(const std_msgs::Header&)> relayToInter2Cb = [&](const std_msgs::Header& h)
		{
		  ++numInter1Received;
			inter2Pub.publish(h);
		};
	boost::function<void(const std_msgs::Header&)> relayToOutCb = [&](const std_msgs::Header& h)
		{
		  ++numInter2Received;
			outPub.publish(h);
		};
	boost::function<void(const std_msgs::Header&)> outCb = [&](const std_msgs::Header& h)
		{
		  ++numOutReceived;
		};
	TestLazySubscriber<std_msgs::Header> lazySub1(inter1Pub, [&](ros::Subscriber& sub)
	  {
		  ++numInter1Subscribes;
			sub = nh.subscribe<std_msgs::Header>("in", 10, relayToInter1Cb);
		});
	TestLazySubscriber<std_msgs::Header> lazySub2(inter2Pub, [&](ros::Subscriber& sub)
	  {
		  ++numInter2Subscribes;
			sub = nh.subscribe<std_msgs::Header>("inter1", 10, relayToInter2Cb);
		});
	TestLazySubscriber<std_msgs::Header> lazySub3(outPub, [&](ros::Subscriber& sub)
	  {
		  ++numOutSubscribes;
			sub = nh.subscribe<std_msgs::Header>("inter2", 10, relayToOutCb);
		});
	
	spin(0.1);
	EXPECT_EQ(0, numInter1Subscribes);
	EXPECT_EQ(0, numInter2Subscribes);
	EXPECT_EQ(0, numOutSubscribes);
	EXPECT_EQ(0, numInReceived);
	EXPECT_EQ(0, numInter1Received);
	EXPECT_EQ(0, numInter2Received);
	EXPECT_EQ(0, numOutReceived);
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, inter1Pub.getNumSubscribers());
	EXPECT_EQ(0, inter2Pub.getNumSubscribers());
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(false, lazySub1.isSubscribed());
	EXPECT_EQ(false, lazySub2.isSubscribed());
	EXPECT_EQ(false, lazySub3.isSubscribed());

	{
		auto outSub = pnh.subscribe<std_msgs::Header>("out", 10, outCb);

		spin(0.1);
		EXPECT_EQ(1, numInter1Subscribes);
		EXPECT_EQ(1, numInter2Subscribes);
		EXPECT_EQ(1, numOutSubscribes);
		EXPECT_EQ(0, numInReceived);
		EXPECT_EQ(0, numInter1Received);
		EXPECT_EQ(0, numInter2Received);
		EXPECT_EQ(0, numOutReceived);
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, inter1Pub.getNumSubscribers());
		EXPECT_EQ(1, inter2Pub.getNumSubscribers());
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(true, lazySub1.isSubscribed());
		EXPECT_EQ(true, lazySub2.isSubscribed());
		EXPECT_EQ(true, lazySub3.isSubscribed());
		
		inPub.publish(std_msgs::Header());

		spin(0.1);
		EXPECT_EQ(1, numInter1Subscribes);
		EXPECT_EQ(1, numInter2Subscribes);
		EXPECT_EQ(1, numOutSubscribes);
		EXPECT_EQ(1, numInReceived);
		EXPECT_EQ(1, numInter1Received);
		EXPECT_EQ(1, numInter2Received);
		EXPECT_EQ(1, numOutReceived);
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, inter1Pub.getNumSubscribers());
		EXPECT_EQ(1, inter2Pub.getNumSubscribers());
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(true, lazySub1.isSubscribed());
		EXPECT_EQ(true, lazySub2.isSubscribed());
		EXPECT_EQ(true, lazySub3.isSubscribed());
		
		inPub.publish(std_msgs::Header());

		spin(0.1);
		EXPECT_EQ(1, numInter1Subscribes);
		EXPECT_EQ(1, numInter2Subscribes);
		EXPECT_EQ(1, numOutSubscribes);
		EXPECT_EQ(2, numInReceived);
		EXPECT_EQ(2, numInter1Received);
		EXPECT_EQ(2, numInter2Received);
		EXPECT_EQ(2, numOutReceived);
		EXPECT_EQ(1, inPub.getNumSubscribers());
		EXPECT_EQ(1, inter1Pub.getNumSubscribers());
		EXPECT_EQ(1, inter2Pub.getNumSubscribers());
		EXPECT_EQ(1, outPub.getNumSubscribers());
		EXPECT_EQ(true, lazySub1.isSubscribed());
		EXPECT_EQ(true, lazySub2.isSubscribed());
		EXPECT_EQ(true, lazySub3.isSubscribed());
	}

	spin(0.1);
	EXPECT_EQ(1, numInter1Subscribes);
	EXPECT_EQ(1, numInter2Subscribes);
	EXPECT_EQ(1, numOutSubscribes);
	EXPECT_EQ(2, numInReceived);
	EXPECT_EQ(2, numInter1Received);
	EXPECT_EQ(2, numInter2Received);
	EXPECT_EQ(2, numOutReceived);
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, inter1Pub.getNumSubscribers());
	EXPECT_EQ(0, inter2Pub.getNumSubscribers());
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(false, lazySub1.isSubscribed());
	EXPECT_EQ(false, lazySub2.isSubscribed());
	EXPECT_EQ(false, lazySub3.isSubscribed());

	inPub.publish(std_msgs::Header());

	spin(0.1);
	EXPECT_EQ(1, numInter1Subscribes);
	EXPECT_EQ(1, numInter2Subscribes);
	EXPECT_EQ(1, numOutSubscribes);
	EXPECT_EQ(2, numInReceived);
	EXPECT_EQ(2, numInter1Received);
	EXPECT_EQ(2, numInter2Received);
	EXPECT_EQ(2, numOutReceived);
	EXPECT_EQ(0, inPub.getNumSubscribers());
	EXPECT_EQ(0, inter1Pub.getNumSubscribers());
	EXPECT_EQ(0, inter2Pub.getNumSubscribers());
	EXPECT_EQ(0, outPub.getNumSubscribers());
	EXPECT_EQ(false, lazySub1.isSubscribed());
	EXPECT_EQ(false, lazySub2.isSubscribed());
	EXPECT_EQ(false, lazySub3.isSubscribed());
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_param_utils");
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();
	// This prevents the ROS background threads from shutting down after the first test's nodehandles disappear
	ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}