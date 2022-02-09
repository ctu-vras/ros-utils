/**
 * \file
 * \brief Unit test for thread_utils.h
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <ctime>
#include <string>
#include <thread>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/thread_utils.h>
#include <cras_cpp_common/thread_utils/semaphore.hpp>

/**
 * \brief Test performance of calling setThreadName(). It should be at least 1M calls per second.
 */
TEST(SetThreadName, SetPerformance)  // NOLINT
{
  const auto start = std::clock();
  
  size_t i = 0;
  while (std::clock() - start < CLOCKS_PER_SEC)
  {
    cras::setThreadName("test");
    ++i;
  }
  
  EXPECT_LT(500000, i);  // about 1.8M in debug mode, 3.2M in release mode, but keep some slack
}

/**
 * \brief Test performance of calling setThreadName(). It should be at least 1M calls per second.
 */
TEST(SetThreadName, SetPerformanceLong)  // NOLINT
{
  const auto start = std::clock();
  
  size_t i = 0;
  while (std::clock() - start < CLOCKS_PER_SEC)
  {
    cras::setThreadName("test_very_long_name_exceeding_15_chars");
    ++i;
  }
  
  EXPECT_LT(500000, i);  // about 1.8M in debug mode, 3.2M in release mode, but keep some slack
}

/**
 * \brief Test performance of calling setThreadName() and reading back the name to verify it was actually set.
 *        It should be at least 1M calls per second.
 */
TEST(SetThreadName, SetGetPerformance)  // NOLINT
{
  const auto start = std::clock();
  
  const std::string names[2] = {"test-a", "test-b"};
  
  size_t i = 0;
  while (std::clock() - start < CLOCKS_PER_SEC)
  {
    cras::setThreadName(names[i % 2]);
    EXPECT_EQ(names[i % 2], cras::getThreadName());
    ++i;
  }

  EXPECT_LT(500000, i);  // about 1.4M in debug mode, 2.2M in release mode, but keep some slack
}

/**
 * \brief Verify that a name that is set is read back the same.
 */
TEST(SetThreadName, SetGet)  // NOLINT
{
  const auto origName = cras::getThreadName();

  cras::setThreadName("test2");
  EXPECT_EQ("test2", cras::getThreadName());
  EXPECT_NE("test2", origName);
}

/**
 * \brief Verify that a name longer than 15 chars gets truncated and set using setThreadName().
 */
TEST(SetThreadName, SetLong)  // NOLINT
{
  const auto name = "very_long_thread_name_that_exceeds_15_chars";
  cras::setThreadName(name);
  EXPECT_NE(name, cras::getThreadName());
  EXPECT_TRUE(cras::startsWith(cras::getThreadName(), "very"));
  EXPECT_TRUE(cras::endsWith(cras::getThreadName(), "chars"));
  EXPECT_FALSE(cras::contains(name, "."));
  EXPECT_TRUE(cras::contains(cras::getThreadName(), "."));
}

/**
 * \brief Test that calling setThreadName() in a different thread does not change the name of the current thread.
 */
TEST(SetThreadName, SetOther)  // NOLINT
{
  const auto origName = cras::getThreadName();
  EXPECT_NE("test3", origName);

  bool executed {false};
  std::thread t([&]()
  {
    cras::setThreadName("test3");
    EXPECT_EQ("test3", cras::getThreadName());
    executed = true;
  });
  t.join();
  EXPECT_EQ(origName, cras::getThreadName());
  EXPECT_TRUE(executed);
}

/**
 * \brief Wait until a thread finishes its work or a timeout happens.
 * \param[in] thread The thread to wait for.
 * \param[in] finished Pointer to a bool that is set to true once the thread finishes.
 * \param[in] timeout Timeout.
 * \param[in] join Whether to join/detach the thread at the end. If false, nothing is done with the thread.
 */
void waitForResult(std::thread& thread, const bool* finished, const ros::WallDuration& timeout, const bool join=true)
{
  const auto endTime = ros::WallTime::now() + timeout;
  while (ros::WallTime::now() < endTime && !*finished)
    ros::WallDuration(0, 1000000).sleep();
  if (!join)
    return;
  if (*finished)
    thread.join();
  else
    thread.detach();
}

/**
 * \brief Test that waitZero() doesn't block right after semaphore creation.
 */
TEST(ReverseSemaphore, ZeroAfterCreation)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    EXPECT_TRUE(sem.waitZero());
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that calling disable() disables the following acquire() call so that the count is still zero after it.
 */
TEST(ReverseSemaphore, DisablePreventsAcquire)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    sem.disable();
    sem.acquire();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that acquire() blocks waitZero().
 */
TEST(ReverseSemaphore, AcquireWorks)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    sem.acquire();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
}

/**
 * \brief Test that acquire() itself is non-blocking.
 */
TEST(ReverseSemaphore, AcquireDoesNotBlock)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    sem.acquire();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that even a zero semaphore can be released() - nothing should happen, just an error is printed to stderr.
 */
TEST(ReverseSemaphore, ReleaseZeroWorks)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    sem.release();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that aquire() works after a pair of disable()/enable() calls.
 */
TEST(ReverseSemaphore, DisableEnable)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    sem.disable();
    sem.enable();
    sem.acquire();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
}

/**
 * \brief Test that a pair of acquire()/release() calls leaves the semaphore at zero.
 */
TEST(ReverseSemaphore, AcquireRelease)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    cras::ReverseSemaphore sem(false);
    started = true;
    sem.acquire();
    sem.release();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that two pairs of acquire()/release() calls leave the semaphore at zero.
 */
TEST(ReverseSemaphore, AcquireAcquireReleaseRelease)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    sem.acquire();
    sem.acquire();
    sem.release();
    sem.release();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that two pairs of acquire()/release() calls leave the semaphore at zero.
 */
TEST(ReverseSemaphore, AcquireReleaseAcquireRelease)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    cras::ReverseSemaphore sem(false);
    started = true;
    sem.acquire();
    sem.release();
    sem.acquire();
    sem.release();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that two pairs of release()/acquire() block waitZero() - the first release() does nothing.
 */
TEST(ReverseSemaphore, ReleaseAcquireReleaseAcquire)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    cras::ReverseSemaphore sem(false);
    started = true;
    sem.release();
    sem.acquire();
    sem.release();
    sem.acquire();
    sem.waitZero();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
}

/**
 * \brief Test asynchronous calls to acquire()/release() and waitZero().
 */
TEST(ReverseSemaphore, Async)  // NOLINT
{
  cras::ReverseSemaphore sem(false);
  
  // 1. acquire() is called, which increases the semaphore.
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    sem.acquire();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  ASSERT_TRUE(finished);
  
  // 2. waitZero() is called, but the semaphore is not yet 0, so it blocks
  started = false; finished = false;
  bool mainFinished {false};
  std::thread main([&](){
    started = true;
    sem.waitZero();
    finished = true;
    mainFinished = true;
  });
  waitForResult(main, &finished, ros::WallDuration(0.1), false);
  ASSERT_FALSE(finished);

  // 3. release() is called, finally unblocking the hanged waitZero()
  started = false; finished = false;
  t = std::thread([&](){
    started = true;
    sem.release();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(finished);

  EXPECT_TRUE(mainFinished);
  
  if (mainFinished)
    main.join();
  else
    main.detach();
}

/**
 * \brief Test asynchronous calls to acquire()/release() and waitZero().
 */
TEST(ReverseSemaphore, AsyncMulti)  // NOLINT
{
  cras::ReverseSemaphore sem(false);
  
  // 1. acquire() nr. 1, sem count is 1
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    sem.acquire();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  ASSERT_TRUE(finished);

  // 1. acquire() nr. 2, sem count is 2
  started = false; finished = false;
  t = std::thread([&](){
    started = true;
    sem.acquire();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  ASSERT_TRUE(finished);
  
  // 3. waitZero() hangs, sem count is 2
  started = false; finished = false;
  bool mainFinished {false};
  std::thread main([&](){
    started = true;
    sem.waitZero();
    finished = true;
    mainFinished = true;
  });
  waitForResult(main, &finished, ros::WallDuration(0.1), false);
  ASSERT_FALSE(finished);

  // 4. release() nr. 1, sem count is 1, waitZero() hangs
  started = false; finished = false;
  t = std::thread([&](){
    started = true;
    sem.release();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(finished);

  EXPECT_FALSE(mainFinished);

  // 5. release() nr. 2, sem count is 0, waitZero() is unblocked
  started = false; finished = false;
  t = std::thread([&](){
    started = true;
    sem.release();
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(finished);

  EXPECT_TRUE(mainFinished);
  
  if (mainFinished)
    main.join();
  else
    main.detach();
}

/**
 * \brief Test multiple asynchronous calls to acquire()/release().
 */
TEST(ReverseSemaphore, AsyncMega)  // NOLINT
{
  cras::ReverseSemaphore sem(false);
  const auto iters = 1000;
  
  // This is bidirectional producer-consumer queues. The condition variables make sure that there is always at least
  // as many acquire() calls as there is release(). This is needed because release() called on a zero semaphore is a
  // no-op and we would thus be left with a few non-released acquire()s.
  volatile size_t num {0};
  volatile size_t num2 {0};
  std::mutex m, m2;
  std::condition_variable cv, cv2;

  bool started1 {false};
  bool finished1 {false};
  std::thread t1([&](){
    started1 = true;
    for (size_t i = 0; i < iters; ++i)
    {
      {
        std::unique_lock<std::mutex> lock(m2);
        cv2.wait(lock, [&num2, &i]() { return num2 <= i; });
      }
      sem.acquire();
      std::scoped_lock<std::mutex> lock(m);
      num++;
      cv.notify_one();
    }
    finished1 = true;
  });

  bool started2 {false};
  bool finished2 {false};
  std::thread t2([&](){
    started2 = true;
    for (size_t i = 0; i < iters; ++i)
    {
      {
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [&num, &i]() { return num > i; });
      }
      sem.release();
      std::scoped_lock<std::mutex> lock(m2);
      num2++;
      cv2.notify_one();
      ros::WallDuration(0, 1000).sleep();
    }
    finished2 = true;
  });
  waitForResult(t1, &started1, ros::WallDuration(0.1), false);
  waitForResult(t2, &started2, ros::WallDuration(0.1), false);
  EXPECT_TRUE(started1);
  EXPECT_TRUE(started2);
  
  // Both feeding threads are started, it's time to call waitZero()
  
  bool mainFinished {false};
  std::thread main([&](){
    EXPECT_TRUE(sem.waitZero());
    mainFinished = true;
  });

  // Wait until the feeding threads finish.
  waitForResult(t1, &finished1, ros::WallDuration(1.1));
  waitForResult(t2, &finished2, ros::WallDuration(1.1));
  EXPECT_TRUE(finished1);
  EXPECT_TRUE(finished2);
  
  // After a little while, the semaphore should also be unblocked.
  waitForResult(main, &mainFinished, ros::WallDuration(0.1), false);
  EXPECT_TRUE(mainFinished);
  
  if (mainFinished)
    main.join();
  else
    main.detach();
}

/**
 * \brief Test async acquire()/release() with the implicit waitZero() called from semaphore destructor.
 */
TEST(ReverseSemaphore, AsyncDestructor)  // NOLINT
{
  // We do not use a smart pointer because if the destructor fails to unblock, the smart pointer would go out of scope
  // at the end of this test and we would have two destructor calls acting on the same object (one is triggered manually
  // from a thread). So we rather use a raw pointer and risk leaking the object (well, it'd just a test, so no problem).
  auto sem = new cras::ReverseSemaphore(true);
  
  // 1. acquire(), sem count is 1
  bool started1 {false};
  bool finished1 {false};
  std::thread t([&](){
    started1 = true;
    sem->acquire();
    finished1 = true;
  });
  waitForResult(t, &finished1, ros::WallDuration(0.1));
  ASSERT_TRUE(finished1);

  // 2. destroy the semaphore; but its count is 1, so the delete call hangs on destructor
  bool started2 {false};
  bool finished2 {false};
  bool mainFinished {false};
  std::thread main([&](){
    started2 = true;
    delete sem;  // Call sem destructor
    sem = nullptr;
    finished2 = true;
    mainFinished = true;
  });
  waitForResult(main, &finished2, ros::WallDuration(0.1), false);
  EXPECT_FALSE(finished2);

  // 3. release(), sem count is 0, the destructor can finish now
  bool started3 {false};
  bool finished3 {false};
  t = std::thread([&](){
    started3 = true;
    if (sem != nullptr)
      sem->release();
    else
      GTEST_NONFATAL_FAILURE_("sem is null");
    finished3 = true;
  });
  waitForResult(t, &finished3, ros::WallDuration(0.1));
  EXPECT_TRUE(finished3);

  EXPECT_TRUE(mainFinished);
  
  if (mainFinished)
    main.join();
  else
    main.detach();
}

/**
 * \brief Test that the semaphore is released after the guard goes out of scope.
 */
TEST(SemaphoreGuard, OutOfScopeReleases)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    {
      cras::SemaphoreGuard<cras::ReverseSemaphore> guard(sem);
      EXPECT_TRUE(guard.acquired());
    }
    EXPECT_TRUE(sem.waitZero());
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

/**
 * \brief Test that the semaphore is acquired when the guard is in scope.
 */
TEST(SemaphoreGuard, InScopeAcquires)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(false);
    {
      cras::SemaphoreGuard<cras::ReverseSemaphore> guard(sem);
      EXPECT_TRUE(guard.acquired());
      EXPECT_FALSE(sem.waitZero());
    }
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
}

/**
 * \brief Test that the semaphore is acquired when the guard is in scope.
 */
TEST(SemaphoreGuard, InScopeAcquires2)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(true);
    {
      cras::SemaphoreGuard<cras::ReverseSemaphore> guard(sem);
      EXPECT_TRUE(guard.acquired());
      EXPECT_FALSE(sem.waitZero());
    }
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_FALSE(finished);
}

/**
 * \brief Test guard acquired() method on a disabled semaphore.
 */
TEST(SemaphoreGuard, Acquired)  // NOLINT
{
  bool started {false};
  bool finished {false};
  std::thread t([&](){
    started = true;
    cras::ReverseSemaphore sem(true);
    sem.disable();
    {
      cras::SemaphoreGuard<cras::ReverseSemaphore> guard(sem);
      EXPECT_FALSE(guard.acquired());
      EXPECT_TRUE(sem.waitZero());
    }
    finished = true;
  });
  waitForResult(t, &finished, ros::WallDuration(0.1));
  EXPECT_TRUE(started);
  EXPECT_TRUE(finished);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}