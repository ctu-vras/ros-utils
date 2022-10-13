/**
 * \file
 * \brief Unit test for nodelet_aware_tf_buffer.h
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <thread>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cras_cpp_common/nodelet_utils/stateful_nodelet.hpp>
#include <cras_cpp_common/nodelet_utils/nodelet_aware_tf_buffer.h>

using namespace cras;

/**
 * \brief Testing stub nodelet based on nodelet::Nodelet. It subscribes to topic "test" with Header messages, and calls
 * this->cb() every time it receives a message.
 */
class NormalNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    this->sub = this->getNodeHandle().subscribe("test", 1, &NormalNodelet::msgCb, this);
  }
  void msgCb(const std_msgs::Header&) const
  {
    if (this->cb)
      this->cb();
  }
  std::function<void()> cb;
  ros::Subscriber sub;
};

/**
 * \brief Testing stub nodelet base on StatefulNodelet. It subscribes to topic "test" with Header messages, and calls
 * this->cb() every time it receives a message.
 */
class TestNodelet : public StatefulNodelet<NormalNodelet>
{
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
 * \brief Try running a possibly infinite TF lookup in a NodeletAwareTFBuffer and check that the lookup is terminated
 * once `requestStop()` is called on the StatefulNodelet.
 */
TEST(NodeletAwareTfBuffer, RequestStop)  // NOLINT
{
  // The test runs with paused sim time to make the canTransform() call infinitely waiting.
  ros::Time::setNow({10, 0});
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);

  auto nodelet = boost::make_shared<TestNodelet>();
  // Global callback queues are used which are spun by ros::spin(); this means nodelet->ok() will skip the "callback
  // queue validity check" workaround and the only way how ok() can be switched to false is calling requestStop().
  nodelet->init("my_nodelet", {}, {}, nullptr, nullptr);
  NodeletAwareTFBuffer buf(*nodelet);

  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::Header>("test", 1);
  while (nodelet->sub.getNumPublishers() == 0 && ros::ok())
    ros::WallDuration(0, 1000).sleep();
  ASSERT_EQ(1, nodelet->sub.getNumPublishers());

  // Put some transform in the TF buffer.
  buf.setTransform(getTransform(), "test");

  // We have data at time 10, so no waiting is needed and canTransform() can immediately return with success.
  EXPECT_TRUE(buf.canTransform("b", "a", ros::Time(10), ros::Duration(0.1)));

  // There is no data for time 11, so the buffer waits; but we do not advance time. Normally, the canTransform() call
  // should wait infinitely (until rostime reaches 11, which it never will), but if we call nodelet.requestStop(), the
  // canTransform() call should return false almost immediately.

  bool started = false;
  bool executed = false;
  nodelet->cb = [&]()
  {
    ASSERT_TRUE(ros::Time::isSimTime());
    EXPECT_TRUE(buf.ok());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf.canTransform("b", "a", ros::Time(11), ros::Duration(0.1), &errstr));
    EXPECT_FALSE(buf.ok());
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    executed = true;
  };

  // The callback has not yet been triggered.
  EXPECT_FALSE(started);
  EXPECT_FALSE(executed);

  // Publish a message and thus trigger nodelet->cb(). It is not, however, processed until someone spins the global
  // callback queue.
  pub.publish(std_msgs::Header());
  ros::WallDuration(0.1).sleep();  // Wait for the publication to get processed (this sleep prevents flakiness)
  EXPECT_FALSE(started);
  EXPECT_FALSE(executed);

  // Spin the global callback queue, triggering nodelet->cb(). Do it in a thread so that the main program can continue.
  std::thread t(&ros::spinOnce);

  // Give things a little time to get processed.
  ros::WallDuration(0.2).sleep();

  // The callback should have started being processed by now and should be hanging in an infinite wait. The timeout of
  // the callback is 0.1 s, and above we have waited for 0.2 s. So if the callback would for some reason timeout in
  // wall time, the value of `executed` should be wrong.
  EXPECT_TRUE(started);
  EXPECT_FALSE(executed);

  // Calling requestStop() should break the infinite wait.
  nodelet->requestStop();

  // Give things a little time to get processed.
  ros::WallDuration(0.1).sleep();

  // By now, the callback should already have finished.
  EXPECT_TRUE(executed);

  // Do not wait for the spinning thread. If the test would block indefinitely, we cannot join() the thread here. So we
  // detach it, letting its cleanup to be done once the whole test suite is finished.
  if (t.joinable())
    t.detach();
}

/**
 * \brief Try running a possibly infinite TF lookup in a NodeletAwareTFBuffer and check that the lookup is terminated
 * once the nodelet is unloaded. Test on a nodelet based on StatefulNodelet.
 */
TEST(NodeletAwareTfBuffer, UnloadStateful)  // NOLINT
{
  // The test runs with paused sim time to make the canTransform() call infinitely waiting.
  ros::Time::setNow({10, 0});
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);

  // Can't be shared_ptr - it would lead to a segfault as Loader::unload() destroys the callback queues that the nodelet
  // needs for its own destruction (to unregister its subscription helper). If the nodelet would outlive Loader in a
  // shared_ptr, its destruction would therefore fail.
  TestNodelet* nodelet;
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
          return boost::shared_ptr<TestNodelet>(nodelet = new TestNodelet);
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

  NodeletAwareTFBuffer buf(*nodelet);

  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::Header>("test", 1);
  while (nodelet->sub.getNumPublishers() == 0 && ros::ok())
    ros::WallDuration(0, 1000).sleep();
  ASSERT_EQ(1, nodelet->sub.getNumPublishers());

  // Put some transform in the TF buffer.
  buf.setTransform(getTransform(), "test");

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
    EXPECT_TRUE(buf.ok());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf.canTransform("b", "a", ros::Time(11), ros::Duration(0.1), &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    EXPECT_FALSE(buf.ok());
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
}

/**
 * \brief Try running a possibly infinite TF lookup in a NodeletAwareTFBuffer and check that the lookup is terminated
 * once the nodelet is unloaded. Test with a nodelet based on nodelet::Nodelet.
 */
TEST(NodeletAwareTfBuffer, UnloadNormal)  // NOLINT
{
  // The test runs with paused sim time to make the canTransform() call infinitely waiting.
  ros::Time::setNow({10, 0});
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);

  // Can't be shared_ptr - it would lead to a segfault as Loader::unload() destroys the callback queues that the nodelet
  // needs for its own destruction (to unregister its subscription helper). If the nodelet would outlive Loader in a
  // shared_ptr, its destruction would therefore fail.
  NormalNodelet* nodelet;
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
          return boost::shared_ptr<NormalNodelet>(nodelet = new NormalNodelet);
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

  NodeletAwareTFBuffer buf(*nodelet);

  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::Header>("test", 1);
  while (nodelet->sub.getNumPublishers() == 0 && ros::ok())
    ros::WallDuration(0, 1000).sleep();
  ASSERT_EQ(1, nodelet->sub.getNumPublishers());

  // Put some transform in the TF buffer.
  buf.setTransform(getTransform(), "test");

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
    EXPECT_TRUE(buf.ok());
    started = true;
    std::string errstr;
    EXPECT_FALSE(buf.canTransform("b", "a", ros::Time(11), ros::Duration(0.1), &errstr));
    EXPECT_EQ("Lookup has been interrupted.", errstr);
    EXPECT_FALSE(buf.ok());
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
 * \brief Test basic transform-related functionality of the buffer: canTransform(), lookupTransform(), transform()...
 */
TEST(NodeletAwareTfBuffer, Transforms)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use sim time
  ASSERT_TRUE(ros::Time::isSimTime());
  ASSERT_EQ(10, ros::Time::now().sec);

  const auto nodelet = boost::make_shared<TestNodelet>();
  nodelet->init("my_nodelet", {}, {});

  NodeletAwareTFBuffer buf(*nodelet);

  buf.setTransform(getTransform(), "test");
  buf.setTransform(getTransform("a", "b", {12, 0}), "test");
  buf.setTransform(getTransform("b", "c", {10, 0}, 0, 1), "test");
  buf.setTransform(getTransform("b", "d", {10, 0}, 0, 0, 0, 0, 0, 1, 0), "test");
  buf.setTransform(getTransform("b", "d", {12, 0}, 0, 0, 0, 0, 0, 1, 0), "test");

  EXPECT_TRUE(buf.canTransform("a", "c", {10, 0}, {1, 0}));
  auto tf = buf.lookupTransform("a", "c", {10, 0}, {1, 0});

  EXPECT_EQ(10, tf.header.stamp.sec);
  EXPECT_EQ(0, tf.header.stamp.nsec);
  EXPECT_EQ("a", tf.header.frame_id);
  EXPECT_EQ("c", tf.child_frame_id);
  EXPECT_EQ(1, tf.transform.translation.x);
  EXPECT_EQ(1, tf.transform.translation.y);
  EXPECT_EQ(0, tf.transform.translation.z);
  EXPECT_EQ(0, tf.transform.rotation.x);
  EXPECT_EQ(0, tf.transform.rotation.y);
  EXPECT_EQ(0, tf.transform.rotation.z);
  EXPECT_EQ(1, tf.transform.rotation.w);

  EXPECT_TRUE(buf.canTransform("a", "d", {10, 0}, {1, 0}));
  tf = buf.lookupTransform("a", "d", {10, 0}, {1, 0});

  EXPECT_EQ(10, tf.header.stamp.sec);
  EXPECT_EQ(0, tf.header.stamp.nsec);
  EXPECT_EQ("a", tf.header.frame_id);
  EXPECT_EQ("d", tf.child_frame_id);
  EXPECT_EQ(1, tf.transform.translation.x);
  EXPECT_EQ(0, tf.transform.translation.y);
  EXPECT_EQ(0, tf.transform.translation.z);
  EXPECT_EQ(0, tf.transform.rotation.x);
  EXPECT_EQ(0, tf.transform.rotation.y);
  EXPECT_EQ(1, tf.transform.rotation.z);
  EXPECT_EQ(0, tf.transform.rotation.w);

  EXPECT_TRUE(buf.canTransform("d", {10, 0}, "d", {12, 0}, "a", {1, 0}));
  tf = buf.lookupTransform("d", {10, 0}, "d", {12, 0}, "a", {1, 0});

  EXPECT_EQ(10, tf.header.stamp.sec);
  EXPECT_EQ(0, tf.header.stamp.nsec);
  EXPECT_EQ("d", tf.header.frame_id);
  EXPECT_EQ("d", tf.child_frame_id);
  EXPECT_EQ(0, tf.transform.translation.x);
  EXPECT_EQ(0, tf.transform.translation.y);
  EXPECT_EQ(0, tf.transform.translation.z);
  EXPECT_EQ(0, tf.transform.rotation.x);
  EXPECT_EQ(0, tf.transform.rotation.y);
  EXPECT_EQ(0, tf.transform.rotation.z);
  EXPECT_EQ(1, tf.transform.rotation.w);

  geometry_msgs::Vector3Stamped v;
  v.header.stamp = {10, 0};
  v.header.frame_id = "a";
  v.vector.x = 10;
  v.vector.y = 0;
  v.vector.z = 0;

  // Vector3 is transformed just by rotation
  const auto v2 = buf.transform(v, std::string("d"), {1, 0});
  EXPECT_EQ(10, v2.header.stamp.sec);
  EXPECT_EQ(0, v2.header.stamp.nsec);
  EXPECT_EQ("d", v2.header.frame_id);
  EXPECT_EQ(-10, v2.vector.x);
  EXPECT_EQ(0, v2.vector.y);
  EXPECT_EQ(0, v2.vector.z);

  geometry_msgs::PointStamped p;
  p.header.stamp = {10, 0};
  p.header.frame_id = "a";
  p.point.x = 10;
  p.point.y = 0;
  p.point.z = 0;

  // Point is transformed both by rotation and translation
  const auto p2 = buf.transform(p, std::string("d"), {1, 0});
  EXPECT_EQ(10, p2.header.stamp.sec);
  EXPECT_EQ(0, p2.header.stamp.nsec);
  EXPECT_EQ("d", p2.header.frame_id);
  EXPECT_EQ(-9, p2.point.x);
  EXPECT_EQ(0, p2.point.y);
  EXPECT_EQ(0, p2.point.z);

  // Test searching for a nonexistent frame. With 0 timeout, the function should not block and rather immediately
  // return failure together with an error message.
  std::string errstr;
  EXPECT_FALSE(buf.canTransform("nonexistent", "d", {10, 0}, {0, 0}, &errstr));
  EXPECT_EQ("canTransform: target_frame nonexistent does not exist. canTransform returned after 0 s, timeout was 0 s.",
    errstr);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_nodelet_aware_tf_buffer");
  ros::start();
  ros::Time::setNow({10, 0});  // use sim time
  return RUN_ALL_TESTS();
}
