/**
 * \file
 * \brief Unit test for nodelet_utils/nodelet_with_shared_tf_buffer.hpp
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <thread>

#include <geometry_msgs/TransformStamped.h>
#include <nodelet/loader.h>
#include <ros/ros.h>
#include <tf2/buffer_core.h>
#include <tf2_msgs/TFMessage.h>

#include <cras_cpp_common/nodelet_utils/nodelet_with_shared_tf_buffer.hpp>

/**
 * \brief Simple NodeletWithSharedTfBuffer.
 */
class TestNodelet : public cras::NodeletWithSharedTfBuffer<>
{
  void onInit() override
  {
  }
};

/**
 * \brief NodeletWithSharedTfBuffer that subscribes to topic "test" and calls `cb` on every message.
 */
class SubNodelet : public cras::NodeletWithSharedTfBuffer<>
{
public:
  void onInit() override
  {
    this->sub = this->getNodeHandle().subscribe("test", 1, &SubNodelet::msgCb, this);
  }
  void msgCb(const std_msgs::Header&) const
  {
    if (this->cb)
      this->cb();
  }
  using nodelet::Nodelet::getNodeHandle;
  std::function<void()> cb;
  ros::Subscriber sub;
};

/**
 * \brief Get a TransformStamped used for the tests.
 */
geometry_msgs::TransformStamped getTransform(const std::string& parentFrame = "a", const std::string& childFrame = "b",
  const ros::Time& stamp = ros::Time(10), const double x = 1, const double y = 0, const double z = 0,
  const double rx = 0, const double ry = 0, const double rz = 0, const double rw = 1)
{
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = parentFrame;
  tf.child_frame_id = childFrame;
  tf.header.stamp = stamp;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;
  tf.transform.rotation.x = rx;
  tf.transform.rotation.y = ry;
  tf.transform.rotation.z = rz;
  tf.transform.rotation.w = rw;
  return tf;
}

/**
 * \brief Check working with a shared buffer.
 */
TEST(NodeletWithSharedTfBuffer, Shared)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  auto buf = std::make_shared<tf2_ros::Buffer>(ros::Duration(30));
  buf->setUsingDedicatedThread(true);
  buf->setTransform(getTransform(), "test");

  {
    TestNodelet nodelet;
    nodelet.init("test", {}, {});
    nodelet.setBuffer(buf);

    EXPECT_TRUE(nodelet.usesSharedBuffer());
    EXPECT_EQ(buf->getCacheLength(), nodelet.getBuffer().getCacheLength());

    EXPECT_TRUE(nodelet.getBuffer().canTransform("a", "b", {10, 0}, {0, 0}));

    nodelet.getBuffer().setTransform(getTransform("a", "b", {11, 0}), "test");
    EXPECT_TRUE(buf->canTransform("a", "b", {11, 0}, {0, 0}));
    EXPECT_TRUE(nodelet.getBuffer().canTransform("a", "b", {11, 0}, {0, 0}));

    buf->setTransform(getTransform("a", "b", {12, 0}), "test");
    EXPECT_TRUE(buf->canTransform("a", "b", {12, 0}, {0, 0}));
    EXPECT_TRUE(nodelet.getBuffer().canTransform("a", "b", {12, 0}, {0, 0}));
  }

  // Check that buf is working even after the nodelet has been destroyed.
  EXPECT_TRUE(buf->canTransform("a", "b", {11, 0}, {0, 0}));
}

/**
 * \brief Check working with a standalone buffer.
 */
TEST(NodeletWithSharedTfBuffer, Standalone)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time

  TestNodelet nodelet;
  nodelet.init("test", {}, {});
  auto& buf = nodelet.getBuffer();

  EXPECT_FALSE(nodelet.usesSharedBuffer());
  EXPECT_EQ(10, buf.getCacheLength().sec);

  buf.setTransform(getTransform(), "test");
  EXPECT_TRUE(buf.canTransform("a", "b", {10, 0}, {0, 0}));

  buf.setTransform(getTransform("a", "b", {11, 0}), "test");
  EXPECT_TRUE(buf.canTransform("a", "b", {11, 0}, {0, 0}));
}

/**
 * \brief Wait until the given condition is satisfied or a timeout occurs.
 * \param[in] condition The condition to wait for. While it is false, this function blocks.
 * \param[in] timeout How long to wait.
 * \return True if condition got satisfied. False if timeout occurred.
 */
bool waitFor(const std::function<bool()>& condition, const ros::WallDuration& timeout)
{
  const auto end = ros::WallTime::now() + timeout;
  while (!condition() && ros::WallTime::now() < end && ros::ok())
    ros::WallDuration(0, 1000).sleep();
  return condition();
}

/**
 * \brief Try running a possibly infinite TF lookup in a NodeletWithSharedTFBuffer and check that the lookup is
 * terminated once the nodelet is unloaded.
 */
TEST(NodeletWithSharedTfBuffer, UnloadShared)  // NOLINT
{
  // The test runs with paused sim time to make the canTransform() call infinitely waiting.
  ros::Time::setNow({10, 0});
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);

  // Can't be shared_ptr - it would lead to a segfault as Loader::unload() destroys the callback queues that the nodelet
  // needs for its own destruction (to unregister its subscription helper). If the nodelet would outlive Loader in a
  // shared_ptr, its destruction would therefore fail.
  SubNodelet* nodelet;
  // Loader is the standard class used for managing nodelets in a nodelet manager. We pass it a custom create_instance
  // function that disables the normal pluginlib lookup algorithm and allows us to create the instances manually.
  // We create it in a separate detached thread so that its destructor (which waits for all worker threads to finish)
  // does not block the other tests (in case this test would never finish). One of the worker threads is spinning the
  // nodelet's callback queue, so it might happen that it gets stuck.
  nodelet::Loader* loaderPointer {nullptr};
  bool stop = false;
  std::thread loaderThread([&]()
    {
      nodelet::Loader l([&](const std::string&)
        {
          return boost::shared_ptr<SubNodelet>(nodelet = new SubNodelet);
        });
      loaderPointer = &l;
      // Keep loaderThread running until the end of this test.
      while (!stop)
        ros::WallDuration(0, 1000000).sleep();
    });
  // Wait until loaderThread creates the loader
  while (loaderPointer == nullptr)
    ros::WallDuration(0, 1000).sleep();
  // Detach the loaderThread so that it doesn't cause a segfault when exiting the program (as we can't join() it).
  loaderThread.detach();
  nodelet::Loader& loader = *loaderPointer;

  // Load a nodelet using the Loader. This will trigger the custom create_instance function and set `nodelet`.
  EXPECT_TRUE(loader.load("my_nodelet", "MyNodelet", {}, {}));
  EXPECT_NE(nullptr, nodelet);

  auto parentBuf = std::make_shared<tf2_ros::Buffer>(ros::Duration(30));
  nodelet->setBuffer(parentBuf);

  auto& buf = nodelet->getBuffer();

  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::Header>("test", 1);
  ASSERT_TRUE(waitFor([&](){ return nodelet->sub.getNumPublishers() > 0; }, ros::WallDuration(0.2)));

  // Put some transform in the TF buffer.
  parentBuf->setTransform(getTransform(), "test");

  // We have data at time 10, so no waiting is needed and canTransform() can immediately return with success.
  EXPECT_TRUE(buf.canTransform("b", "a", ros::Time(10), ros::Duration(0.1)));

  // There is no data for time 11, so the buffer waits; but we do not advance time. Normally, the canTransform() call
  // should wait infinitely (until rostime reaches 11, which it never will), but if we unload the nodelet, the
  // canTransform() call should return false almost immediately.
  bool started = false;
  bool executed = false;
  nodelet->cb = [&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf.canTransform("b", "a", ros::Time(11), ros::Duration(0.1), &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  };

  // The callback has not yet been triggered.
  EXPECT_FALSE(started);
  EXPECT_FALSE(executed);

  // Publish a message and thus trigger nodelet->cb(). Spinning of the nodelet callback queue is done in a worker thread
  // by Loader, so we just wait a while and cb() should get called.
  pub.publish(std_msgs::Header());

  // Give things a little time to get connected and processed.
  ros::WallDuration(0.2).sleep();

  // The callback should have started being processed by now and should be hanging in an infinite wait. The timeout of
  // the callback is 0.1 s, and above we have waited for 0.2 s. So if the callback would for some reason timeout in
  // wall time, the value of `executed` should be wrong.
  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  // Unload the nodelet. This should break the infinite canTransform() call. Do not use `nodelet` pointer from now on!
  loader.unload("my_nodelet");

  // Give things a little time to get processed.
  ros::WallDuration(0.1).sleep();

  // After the nodelet is unloaded and some short wait, the callback should be stopped.
  EXPECT_TRUE(executed);

  // Stop loaderThread if everything went well. loaderThread reference and loaderPointer have to be treated as invalid
  // as of now.
  stop = true;
}

/**
 * \brief Try running a possibly infinite TF lookup in a NodeletWithSharedTFBuffer and check that the lookup is
 * terminated once the nodelet is unloaded.
 */
TEST(NodeletWithSharedTfBuffer, UnloadStandalone)  // NOLINT
{
  // The test runs with paused sim time to make the canTransform() call infinitely waiting.
  ros::Time::setNow({10, 0});
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);

  // Can't be shared_ptr - it would lead to a segfault as Loader::unload() destroys the callback queues that the nodelet
  // needs for its own destruction (to unregister its subscription helper). If the nodelet would outlive Loader in a
  // shared_ptr, its destruction would therefore fail.
  SubNodelet* nodelet;
  // Loader is the standard class used for managing nodelets in a nodelet manager. We pass it a custom create_instance
  // function that disables the normal pluginlib lookup algorithm and allows us to create the instances manually.
  // We create it in a separate detached thread so that its destructor (which waits for all worker threads to finish)
  // does not block the other tests (in case this test would never finish). One of the worker threads is spinning the
  // nodelet's callback queue, so it might happen that it gets stuck.
  nodelet::Loader* loaderPointer {nullptr};
  bool stop = false;
  std::thread loaderThread([&]()
    {
      nodelet::Loader l([&](const std::string&)
        {
          return boost::shared_ptr<SubNodelet>(nodelet = new SubNodelet);
        });
      loaderPointer = &l;
      // Keep loaderThread running until the end of this test.
      while (!stop)
        ros::WallDuration(0, 1000000).sleep();
    });
  // Wait until loaderThread creates the loader
  while (loaderPointer == nullptr)
    ros::WallDuration(0, 1000).sleep();
  // Detach the loaderThread so that it doesn't cause a segfault when exiting the program (as we can't join() it).
  loaderThread.detach();
  nodelet::Loader& loader = *loaderPointer;

  // Load a nodelet using the Loader. This will trigger the custom create_instance function and set `nodelet`.
  EXPECT_TRUE(loader.load("my_nodelet", "MyNodelet", {}, {}));
  EXPECT_NE(nullptr, nodelet);

  ASSERT_TRUE(ros::Time::isSimTime());

  auto& buf = nodelet->getBuffer();
  tf2_ros::TransformListener l(buf.getRawBuffer(), nodelet->getNodeHandle());
  ros::WallDuration(0.2).sleep();  // Give the TF thread some time to register subscribers.

  ros::NodeHandle nh;

  auto pub = nh.advertise<std_msgs::Header>("test", 1);
  ASSERT_TRUE(waitFor([&](){ return nodelet->sub.getNumPublishers() > 0; }, ros::WallDuration(0.2)));

  // the standalone buffer should spawn a TF listener, so we set the transform via the listener
  auto pubTf = nh.advertise<tf2_msgs::TFMessage>("/tf", 1);
  ASSERT_TRUE(waitFor([&](){ return pubTf.getNumSubscribers() > 0; }, ros::WallDuration(0.2)));

  // Put some transform in the TF buffer.
  tf2_msgs::TFMessage tf; tf.transforms.push_back(getTransform());
  pubTf.publish(tf);
  ros::spinOnce();
  ros::WallDuration(0.2).sleep();
  ros::spinOnce();

  // We have data at time 10, so no waiting is needed and canTransform() can immediately return with success.
  EXPECT_TRUE(buf.canTransform("b", "a", ros::Time(10), ros::Duration(0)));

  // There is no data for time 11, so the buffer waits; but we do not advance time. Normally, the canTransform() call
  // should wait infinitely (until rostime reaches 11, which it never will), but if we unload the nodelet, the
  // canTransform() call should return false almost immediately.
  bool started = false;
  bool executed = false;
  nodelet->cb = [&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf.canTransform("b", "a", ros::Time(11), ros::Duration(0.1), &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  };

  // The callback has not yet been triggered.
  EXPECT_FALSE(started);
  EXPECT_FALSE(executed);

  // Publish a message and thus trigger nodelet->cb(). Spinning of the nodelet callback queue is done in a worker thread
  // by Loader, so we just wait a while and cb() should get called.
  pub.publish(std_msgs::Header());

  // Give things a little time to get connected and processed.
  ros::WallDuration(0.2).sleep();

  // The callback should have started being processed by now and should be hanging in an infinite wait. The timeout of
  // the callback is 0.1 s, and above we have waited for 0.2 s. So if the callback would for some reason timeout in
  // wall time, the value of `executed` should be wrong.
  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  // Unload the nodelet. This should break the infinite canTransform() call. Do not use `nodelet` pointer from now on!
  loader.unload("my_nodelet");

  // Give things a little time to get processed.
  ros::WallDuration(0.1).sleep();

  // After the nodelet is unloaded and some short wait, the callback should be stopped.
  EXPECT_TRUE(executed);

  // Stop loaderThread if everything went well. loaderThread reference and loaderPointer have to be treated as invalid
  // as of now.
  stop = true;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // ROS init.
  ros::init(argc, argv, "test_nodelet_with_shared_tf_buffer");
  ros::start();

  return RUN_ALL_TESTS();
}
