// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of a reversed semaphore usable for thread synchronization.
 * \author Martin Pecka
 */

#include <iostream>

#include <cras_cpp_common/thread_utils/semaphore.hpp>

namespace cras {

ReverseSemaphore::ReverseSemaphore(const bool wait_zero_at_destroy) : wait_zero_at_destroy_(wait_zero_at_destroy) {
}

ReverseSemaphore::~ReverseSemaphore() {
  is_destroying_ = true;
  disable();
  if (wait_zero_at_destroy_) {
    waitZero();
  } else {
    // needed for cv destructor to finish
    std::lock_guard lock(mutex_);
    cv_.notify_all();
  }
}

bool ReverseSemaphore::acquire() {
  std::lock_guard lock(mutex_);
  if (disabled_) {
    return false;
  }
  count_ = count_ + 1;
  return true;
}

void ReverseSemaphore::release() {
  auto report_error{false};
  {
    std::lock_guard lock(mutex_);
    if (count_ > 0) {
      count_ = count_ - 1;
    } else {
      report_error = true;
    }
    if (count_ == 0) {
      cv_.notify_all();
    }
  }
  if (report_error) {
    std::cerr << "ReverseSemaphore released more times than acquired!" << std::endl;
  }
}

bool ReverseSemaphore::waitZero() {
  std::unique_lock lock(mutex_);
  cv_.wait(lock, [this] { return count_ == 0 || (!wait_zero_at_destroy_ && is_destroying_); });
  return count_ == 0;
}

void ReverseSemaphore::disable() {
  std::lock_guard lock(mutex_);
  disabled_ = true;
}

void ReverseSemaphore::enable() {
  std::lock_guard lock(mutex_);
  disabled_ = false;
}

bool ReverseSemaphore::isEnabled() const {
  std::lock_guard lock(mutex_);
  return !disabled_;
}

size_t ReverseSemaphore::getCount() const {
  std::lock_guard lock(mutex_);
  return count_;
}

}  // namespace cras
