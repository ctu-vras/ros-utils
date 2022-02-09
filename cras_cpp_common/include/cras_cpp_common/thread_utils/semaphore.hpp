/**
 * \file
 * \brief Implementation of a reversed semaphore usable for thread synchronization.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <condition_variable>
#include <mutex>

namespace cras
{

/**
 * \brief A reverse counting semaphore which can wait until its count is zero. Each `acquire()` increases this count and
 * each `release()` decreases it. `waitZero()` is the function that waits until the internal count is zero. The
 * semaphore can be disabled, which means no new `acquire()` calls will be accepted. This is useful if you plan to quit.
 */
class ReverseSemaphore
{
public:
  /**
   * \brief Create the semaphore (internal count is zero).
   * \param[in] waitZeroAtDestroy Whether the semaphore should `waitForZero()` when it is destroyed. 
   */
  explicit ReverseSemaphore(bool waitZeroAtDestroy=true);
  
  /**
   * \brief Destroys this semaphore. Internally blocks it and waits for zero count.
   */
  ~ReverseSemaphore();
  
  /**
   * \brief Acquire the semaphore (increase its count). This method never blocks for long.
   * \return Whether acquisition succeeded. It can fail if the semaphore is disabled.
   */
  bool acquire();
  
  /**
   * \brief Release the semaphore (decrease its count).
   * \note If the internal count decreases to zero, all outstanding `waitZero()` calls are notified.
   * \note If the internal count should go below 0, an error is printed to stderr and all outstanding `waitZero()` calls
   *       are notified.
   * \note `release()` calls are processed even if the semaphore is disabled.
   */
  void release();
  
  /**
   * \brief Wait until the internal count reaches zero.
   * \return Whether the wait succeeded. False can be returned if this semaphore is being destroyed.
   * \note It is suggested to call `disable()` before this method if you call it because some object needs to exit.
   */
  bool waitZero();
  
  /**
   * \brief Disable the semaphore. All following `acquire()` calls will return immediately with false.
   */
  void disable();
  
  /**
   * \brief Enable the semaphore. Calling `acquire()` works normally after this call.
   */
  void enable();

private:
  //! \brief Whether to wait for zero when the object is being destroyed.
  bool waitZeroAtDestroy;
  
  //! \brief True if the destructor has begun.
  bool isDestroying {false};

  //! \brief The internal count of the semaphore.
  volatile size_t count {0};
  
  //! \brief Whether the semaphore is disabled.
  volatile bool disabled {false};
  
  //! \brief Mutex protecting `cv`, `count` and `disabled`.
  ::std::mutex mutex;
  
  //! \brief Condition variable used for signalling between `release()` and `waitZero()`.
  ::std::condition_variable cv;
};

/**
 * \brief RAII guard for operations with a semaphore. On creation, the semaphore is acquired, and on destruction,
 *        it is released.
 * \tparam T Type of the semaphore. Can be any type with methods `bool acquire()` and `void release()`.
 * \note The usage pattern is slightly different than the normal lock_guard - because acquire() can return false (e.g.
 *       if the semaphore is blocked for new acquisitions), the pattern should be the following:
 *       `SemaphoreGuard<ReverseSemaphore> guard(sem); if (!guard.acquired()) return;
 */
template <typename T>
class SemaphoreGuard
{
public:
  explicit SemaphoreGuard(T& semaphore) : semaphore(semaphore)
  {
    this->acquireSucceeded = this->semaphore.acquire();
  }

  ~SemaphoreGuard()
  {
    if (this->acquireSucceeded)
      this->semaphore.release();
  }
  
  /**
   * \brief Whether the semaphore acquisition succeeded when constructing this guard.
   * \return Success value.
   * \note Always check the result of this function after constructing a guard. The semaphore may be disabled.
   */
  bool acquired() const
  {
    return this->acquireSucceeded;
  }

private:
  //! \brief The guarded semaphore.
  T& semaphore;
  
  //! \brief Whether the acquire succeeded.
  bool acquireSucceeded {false};
};
}