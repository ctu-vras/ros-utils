// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for Resettable.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>

#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/resettable.h>


void spin(const double duration, const double wait)
{
    double waited = 0;
    while (waited < duration)
    {
        ros::spinOnce();
        ros::WallDuration(wait).sleep();
        waited += wait;
    }
}

void spin(const double duration)
{
    spin(duration, duration / 1000.0);
}

class TestNode : public cras::TimeJumpResettable
{
public:
    explicit TestNode(const cras::LogHelperPtr& log) : TimeJumpResettable(log)
    {
    }

    void reset() override
    {
        this->numResetCalls += 1;
    }

    size_t numResetCalls {0u};
};

TEST(Resettable, Reset)  // NOLINT
{
    ros::NodeHandle pnh("~");
    auto resetPub = pnh.advertise<std_msgs::Bool>("/reset", 1);
    auto resetPrivPub = pnh.advertise<std_msgs::Bool>("reset", 1);

    ros::Time::setNow(ros::Time::MIN);

    auto node = TestNode(std::make_shared<cras::NodeLogHelper>());

    EXPECT_EQ(0u, node.numResetCalls);
    node.reset();
    EXPECT_EQ(1u, node.numResetCalls);

    node.initRos(pnh);

    EXPECT_EQ(1u, node.numResetCalls);

    node.checkTimeJump(ros::Time(10));
    EXPECT_EQ(1, node.numResetCalls);
    node.checkTimeJump(ros::Time(11));
    EXPECT_EQ(1, node.numResetCalls);
    node.checkTimeJump(ros::Time(12));
    EXPECT_EQ(1, node.numResetCalls);
    // by default, in simtime the jump back threshold is 3 seconds
    node.checkTimeJump(ros::Time(9));
    EXPECT_EQ(2, node.numResetCalls);
    node.checkTimeJump(ros::Time(10));
    EXPECT_EQ(2, node.numResetCalls);
    // by default, in simtime the jump forward threshold is 10 seconds
    node.checkTimeJump(ros::Time(21));
    EXPECT_EQ(3, node.numResetCalls);

    node.checkTimeJump(ros::Time(10));
    ros::Time::setNow(ros::Time(10));

    node.startAutoCheckTimeJump(ros::WallRate(10));
    EXPECT_EQ(4, node.numResetCalls);
    ros::Time::setNow(ros::Time(10.1));
    EXPECT_EQ(4, node.numResetCalls);
    spin(0.1);
    EXPECT_EQ(4, node.numResetCalls);
    ros::Time::setNow(ros::Time(10.2));
    EXPECT_EQ(4, node.numResetCalls);
    spin(0.1);
    EXPECT_EQ(4, node.numResetCalls);
    ros::Time::setNow(ros::Time(7));
    EXPECT_EQ(4, node.numResetCalls);
    spin(0.5);
    EXPECT_EQ(5, node.numResetCalls);
    node.stopAutoCheckTimeJump();
    spin(0.5);
    EXPECT_EQ(5, node.numResetCalls);

    std_msgs::Bool msg;
    msg.data = true;
    resetPub.publish(msg);
    for (size_t i = 0; i < 100; ++i)
    {
        if (node.numResetCalls == 6)
            break;
        spin(0.005);
    }
    EXPECT_EQ(6, node.numResetCalls);

    msg.data = false;
    resetPrivPub.publish(msg);
    for (size_t i = 0; i < 100; ++i)
    {
        if (node.numResetCalls == 7)
            break;
        spin(0.005);
    }
    EXPECT_EQ(7, node.numResetCalls);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "resettable_test");
    return RUN_ALL_TESTS();
}
