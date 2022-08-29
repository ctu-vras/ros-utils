/**
 * \file
 * \brief Implementation of a reversed semaphore usable for thread synchronization.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <iostream>

#include <cras_cpp_common/thread_utils/semaphore.hpp>

namespace cras
{

ReverseSemaphore::ReverseSemaphore(bool waitZeroAtDestroy) : waitZeroAtDestroy(waitZeroAtDestroy)
{
}

ReverseSemaphore::~ReverseSemaphore()
{
  this->isDestroying = true;
  this->disable();
  if (this->waitZeroAtDestroy)
  {
    this->waitZero();
  }
  else
  {
    // needed for cv destructor to finish
    std::lock_guard<decltype(this->mutex)> lock(this->mutex);
    this->cv.notify_all();
  }
}

bool ReverseSemaphore::acquire()
{
  std::lock_guard<decltype(this->mutex)> lock(this->mutex);
  if (this->disabled)
    return false;
  this->count++;
  return true;
}

void ReverseSemaphore::release()
{
  bool reportError{false};
  {
    std::lock_guard<decltype(this->mutex)> lock(this->mutex);
    if (this->count > 0)
      this->count--;
    else
      reportError = true;
    if (this->count == 0)
      this->cv.notify_all();
  }
  if (reportError)
    std::cerr << "ReverseSemaphore released more times than acquired!" << std::endl;
}

bool ReverseSemaphore::waitZero()
{
  std::unique_lock<decltype(this->mutex)> lock(this->mutex);
  this->cv.wait(lock, [this](){ return this->count == 0 || (!this->waitZeroAtDestroy && this->isDestroying); });
  return this->count == 0;
}

void ReverseSemaphore::disable()
{
  std::lock_guard<decltype(this->mutex)> lock(this->mutex);
  this->disabled = true;
}

void ReverseSemaphore::enable()
{
  std::lock_guard<decltype(this->mutex)> lock(this->mutex);
  this->disabled = false;
}

bool ReverseSemaphore::isEnabled() const
{
  std::lock_guard<decltype(this->mutex)> lock(this->mutex);
  return !this->disabled;
}

size_t ReverseSemaphore::getCount() const
{
  std::lock_guard<decltype(this->mutex)> lock(this->mutex);
  return this->count;
}

}