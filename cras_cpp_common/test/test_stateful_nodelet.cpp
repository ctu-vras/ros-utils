/**
 * \file
 * \brief Unit test for stateful_nodelet.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>
#include <thread>

#include <nodelet/loader.h>
#include <ros/ros.h>

#include <cras_cpp_common/nodelet_utils/stateful_nodelet.hpp>

using namespace cras;

/**
 * \brief A mock of the nodelet::Nodelet class with empty onInit().
 */
struct TestNodelet : public cras::StatefulNodelet<::nodelet::Nodelet>
{
  /**
   * \brief If wasUnloading is not null, set the value of isNodeletUnloading(*this) to it while the object is destroyed.
   */
  ~TestNodelet() override
  {
    if (this->wasUnloading != nullptr)
      *this->wasUnloading = cras::isNodeletUnloading(*this);
    if (this->wasOk != nullptr)
      *this->wasOk = this->ok();
  }

  /**
   * \brief Empty.
   */
  void onInit() override
  {
  }

  //! \brief The value of isNodeletUnloading(*this) during destruction.
  volatile bool* wasUnloading {nullptr};

  //! \brief The value of this->ok() during destruction.
  volatile bool* wasOk {nullptr};
};

/**
 * \brief Test `ok()` and `requestStop()` functions.
 */
TEST(StatefulNodelet, ok)  // NOLINT
{
  bool wasUnloading {true};
  bool wasOk {true};
  {
    TestNodelet nodelet;
    nodelet.wasUnloading = &wasUnloading;
    nodelet.wasOk = &wasOk;
    EXPECT_TRUE(nodelet.ok());

    nodelet.requestStop();
    EXPECT_FALSE(nodelet.ok());
  }
  EXPECT_FALSE(wasUnloading);
  EXPECT_FALSE(wasOk);
}

/**
 * \brief Test interrupting `sleep()` function with system time.
 */
TEST(StatefulNodelet, sleepInterruptSystime)  // NOLINT
{
  ros::Time::init();  // use system time
  ASSERT_TRUE(ros::Time::useSystemTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started, then interrupt it, and after
  // another 0.1 s, check that it has finished (if not, `finished` should still be false after the 0.2 s of waiting).

  TestNodelet nodelet;
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_FALSE(nodelet.sleep(ros::Duration(1)));
      finished = true;
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  nodelet.requestStop();
  ros::WallDuration(0.1).sleep();

  EXPECT_TRUE(finished);

  t.join();
}

/**
 * \brief Test interrupting `sleep()` function with simulation time.
 */
TEST(StatefulNodelet, sleepInterruptSimtime)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use simulation time
  ASSERT_TRUE(ros::Time::isSimTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started, then interrupt it, and after
  // another 0.1 s, check that it has finished (if not, `finished` should still be false after the 0.2 s of waiting).

  TestNodelet nodelet;
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_FALSE(nodelet.sleep({1, 0}));
      finished = true;
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  ros::Time::setNow(ros::Time(10.1));

  nodelet.requestStop();
  ros::WallDuration(0.1).sleep();

  EXPECT_TRUE(finished);

  t.detach();  // detach the thread so that it doesn't block if the sleep did not end
}

/**
 * \brief Test finishing `sleep()` function with system time.
 */
TEST(StatefulNodelet, sleepFinishSystime)  // NOLINT
{
  ros::Time::init();  // use system time
  ASSERT_TRUE(ros::Time::isSystemTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started but not finished. After another
  // 1 s, check that it has finished.

  TestNodelet nodelet;
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_TRUE(nodelet.sleep(ros::Duration(1)));
      finished = true;
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  ros::WallDuration(1).sleep();

  EXPECT_TRUE(finished);

  t.detach();  // detach the thread so that it doesn't block if the sleep did not end
}

/**
 * \brief Test finishing `sleep()` function with simulation time.
 */
TEST(StatefulNodelet, sleepFinishSimtime)  // NOLINT
{
  ros::Time::setNow({10, 0});  // use simulation time
  ASSERT_TRUE(ros::Time::isSimTime());

  // Create a thread that sleeps for 1 second. After 0.1 s, check that it has started but not finished. After another
  // 1.1 s, check that it has finished.

  TestNodelet nodelet;
  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      started = true;
      EXPECT_TRUE(nodelet.sleep({1, 0}));
      finished = true;
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  ros::Time::setNow(ros::Time(11.1));
  ros::WallDuration(0.1).sleep();

  EXPECT_TRUE(finished);

  t.detach();  // detach the thread so that it doesn't block if the sleep did not end
}

/**
 * \brief Test `isNodeletUnloading()` on an uninitialized and unmanaged nodelet.
 */
TEST(StatefulNodelet, isNodeletUnloading)  // NOLINT
{
  bool wasUnloading {true};
  bool wasOk {true};
  {
    TestNodelet nodelet;
    nodelet.wasUnloading = &wasUnloading;
    nodelet.wasOk = &wasOk;
    EXPECT_FALSE(cras::isNodeletUnloading(nodelet));

    nodelet.requestStop();
    EXPECT_FALSE(cras::isNodeletUnloading(nodelet));

    ros::WallDuration(0.1).sleep();
    EXPECT_FALSE(cras::isNodeletUnloading(nodelet));
  }
  EXPECT_FALSE(wasUnloading);
  EXPECT_FALSE(wasOk);
}

/**
 * \brief Test that `sleep()` can be interrupted by nodelet unloading in system time. Also test `isNodeletUnloading()`.
 */
TEST(StatefulNodelet, sleepInterruptByUnloadSystime)  // NOLINT
{
  ros::Time::init();
  ASSERT_TRUE(ros::Time::isSystemTime());

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
  volatile bool wasUnloading {false};
  volatile bool wasOk {true};
  nodelet->wasUnloading = &wasUnloading;
  nodelet->wasOk = &wasOk;
  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
  EXPECT_TRUE(nodelet->ok());

  // First, try if the sleep will finish if waiting enough time

  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
      started = true;
      EXPECT_TRUE(nodelet->sleep(ros::Duration(1)));
      finished = true;
      EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  ros::WallDuration(1).sleep();
  EXPECT_TRUE(finished);

  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));

  // Now, test that the sleep can be interrupted.

  started = false;
  finished = false;
  std::thread t2([&]()
    {
      EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
      started = true;
      EXPECT_FALSE(nodelet->sleep({1, 0}));
      finished = true;
      // cannot test EXPECT_TRUE(cras::isNodeletUnloading(*nodelet)) here as nodelet is no longer valid
      // the following values are flaky for some reason, so wait a bit to help them settle
      ros::WallDuration(0.1).sleep();
      EXPECT_TRUE(wasUnloading);
      EXPECT_FALSE(wasOk);
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
  EXPECT_FALSE(wasUnloading);
  EXPECT_TRUE(wasOk);
  loader.unload("my_nodelet");
  EXPECT_TRUE(wasUnloading);
  EXPECT_FALSE(wasOk);

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(finished);

  t.detach();  // detach the threads so that they don't block if the sleep did not end
  t2.detach();
  stop = true;  // stop the loaderThread
}

/**
 * \brief Test that `sleep()` can be interrupted by nodelet unloading in sim time. Also test `isNodeletUnloading()`.
 */
TEST(StatefulNodelet, sleepInterruptByUnloadSimtime)  // NOLINT
{
  // The test runs with paused sim time to make the sleep() call infinitely waiting.
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
  volatile bool wasUnloading {false};
  volatile bool wasOk {true};
  nodelet->wasUnloading = &wasUnloading;
  nodelet->wasOk = &wasOk;
  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
  EXPECT_TRUE(nodelet->ok());

  // First, try if the sleep will finish if waiting enough time

  bool started = false;
  bool finished = false;
  std::thread t([&]()
    {
      EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
      EXPECT_TRUE(nodelet->ok());
      started = true;
      EXPECT_TRUE(nodelet->sleep({1, 0}));
      finished = true;
      EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
      EXPECT_TRUE(nodelet->ok());
    });

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
  EXPECT_TRUE(nodelet->ok());

  ros::Time::setNow(ros::Time(11.1));
  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(finished);

  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
  EXPECT_TRUE(nodelet->ok());

  // Now, test that the sleep can be interrupted.

  ros::Time::setNow(ros::Time(10));

  started = false;
  finished = false;
  std::thread t2([&]()
    {
      EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
      EXPECT_TRUE(nodelet->ok());
      started = true;
      EXPECT_FALSE(nodelet->sleep({1, 0}));
      finished = true;
      // cannot test EXPECT_TRUE(cras::isNodeletUnloading(*nodelet)) here as nodelet is no longer valid
      // the following values are flaky for some reason, so wait a bit to help them settle
      ros::WallDuration(0.1).sleep();
      EXPECT_TRUE(wasUnloading);
      EXPECT_FALSE(wasOk);
    });

  ros::Time::setNow(ros::Time(10.1));  // not enough for the sleep to finish
  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);

  EXPECT_FALSE(cras::isNodeletUnloading(*nodelet));
  EXPECT_TRUE(nodelet->ok());
  EXPECT_FALSE(wasUnloading);
  EXPECT_TRUE(wasOk);
  loader.unload("my_nodelet");
  EXPECT_TRUE(wasUnloading);
  EXPECT_FALSE(wasOk);

  ros::WallDuration(0.1).sleep();
  EXPECT_TRUE(finished);

  t.detach();  // detach the threads so that they don't block if the sleep did not end
  t2.detach();
  stop = true;  // stop the loaderThread
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_stateful_nodelet");
  ros::start();
  return RUN_ALL_TESTS();
}
