#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of a reversed semaphore usable for thread synchronization.
 * \author Martin Pecka
 */

#include <condition_variable>
#include <mutex>

namespace cras {

/**
 * \brief A reverse counting semaphore which can wait until its count is zero. Each `acquire()` increases this count and
 * each `release()` decreases it. `waitZero()` is the function that waits until the internal count is zero. The
 * semaphore can be disabled, which means no new `acquire()` calls will be accepted. This is useful if you plan to quit.
 */
class ReverseSemaphore {
public:
  /**
   * \brief Create the semaphore (internal count is zero).
   * \param[in] wait_zero_at_destroy Whether the semaphore should `waitForZero()` when it is destroyed.
   */
  explicit ReverseSemaphore(bool wait_zero_at_destroy = true);

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

  /**
   * \brief Whether the semaphore is enabled or not.
   */
  bool isEnabled() const;

  /**
   * \brief Get the current number of unreleased `acquire()`s.
   * \note Do not use this to tell when all `acquire()`s are released. Use `waitZero()` for that.
   * \return The number of unreleased `acquire()`s.
   */
  size_t getCount() const;

private:
  //! \brief Whether to wait for zero when the object is being destroyed.
  bool wait_zero_at_destroy_;

  //! \brief True if the destructor has begun.
  bool is_destroying_ {false};

  //! \brief The internal count of the semaphore.
  volatile size_t count_ {0};

  //! \brief Whether the semaphore is disabled.
  volatile bool disabled_ {false};

  //! \brief Mutex protecting `cv_`, `count_` and `disabled_`.
  mutable ::std::mutex mutex_;

  //! \brief Condition variable used for signalling between `release()` and `waitZero()`.
  ::std::condition_variable cv_;
};

/**
 * \brief RAII guard for operations with a semaphore. On creation, the semaphore is acquired, and on destruction,
 *        it is released.
 * \tparam T Type of the semaphore. Can be any type with methods `bool acquire()` and `void release()`.
 * \note The usage pattern is slightly different than the normal lock_guard - because acquire() can return false (e.g.
 *       if the semaphore is blocked for new acquisitions), the pattern should be the following:
 *       `SemaphoreGuard&lt;ReverseSemaphore&gt; guard(sem); if (!guard.acquired()) return;
 */
template<typename T>
class SemaphoreGuard {
public:
  explicit SemaphoreGuard(T& semaphore) : semaphore_(semaphore) {
    acquire_succeeded_ = semaphore_.acquire();
  }

  ~SemaphoreGuard() {
    if (acquire_succeeded_) {
      semaphore_.release();
    }
  }

  /**
   * \brief Whether the semaphore acquisition succeeded when constructing this guard.
   * \return Success value.
   * \note Always check the result of this function after constructing a guard. The semaphore may be disabled.
   */
  bool acquired() const {
    return acquire_succeeded_;
  }

private:
  //! \brief The guarded semaphore.
  T& semaphore_;

  //! \brief Whether the acquire succeeded.
  bool acquire_succeeded_ {false};
};

}  // namespace cras
