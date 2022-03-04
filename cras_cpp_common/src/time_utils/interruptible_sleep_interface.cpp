/**
 * \file
 * \brief Object implementing an `ok()` method that can interrupt pending sleeps when it returns false.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cras_cpp_common/time_utils/interruptible_sleep_interface.h>

#include <ros/init.h>
#include <ros/time.h>

#include <cras_cpp_common/thread_utils/semaphore.hpp>

namespace cras
{

struct InterruptibleSleepInterfacePrivate
{
  //! \brief This semaphore prevents the object to be destructed before pending sleeps finish.
  mutable cras::ReverseSemaphore semaphore;
};

InterruptibleSleepInterface::InterruptibleSleepInterface() : data(new InterruptibleSleepInterfacePrivate)
{
}

InterruptibleSleepInterface::~InterruptibleSleepInterface()
{
  this->data->semaphore.disable();
  this->data->semaphore.waitZero();
}

bool InterruptibleSleepInterface::ok() const
{
  // This should only be called when the object is partially destroyed. Normally, the override method is called.
  return this->data->semaphore.isEnabled();
}

bool InterruptibleSleepInterface::sleep(const ros::Duration& duration) const
{
  if (duration.isZero())
    return true;

  // Fast-track exit in case ok() is false to prevent additional locks of the mutex that is needed for object
  // destruction.
  if (!this->ok() || (ros::isInitialized() && !ros::ok()))
    return false;

  cras::SemaphoreGuard<cras::ReverseSemaphore> guard(this->data->semaphore);
  
  // code heavily inspired by BSD-licensed https://github.com/ros/roscpp_core/blob/noetic-devel/rostime/src/time.cpp
  // what is added is the this->ok() check in the while loop and making the system time sleep also interruptible

  if (ros::Time::useSystemTime())
  {
    const auto start = ros::WallTime::now();
    const auto wallDuration = ros::WallDuration(duration.sec, duration.nsec);
    const auto pollWallDuration = wallDuration * 0.01;
    const auto end = start + wallDuration;

    bool rc = ros::WallTime::now() >= end;  // if the duration was veery short, we might already have finished now
    while ((ros::ok() || !ros::isInitialized()) && this->ok() && (ros::WallTime::now() < end))
    {
      pollWallDuration.sleep();
      rc = true;
    }

    return rc && (ros::ok() || !ros::isInitialized()) && this->ok();
  }

  auto start = ros::Time::now();
  auto end = start + duration;
  if (start.isZero())
    end = ros::TIME_MAX;

  bool rc = ros::Time::now() >= end;  // if the duration was veery short, we might already have finished now
  while ((ros::ok() || !ros::isInitialized()) && this->ok() && (ros::Time::now() < end))
  {
    this->pollDuration.sleep();
    rc = true;

    // If we started at time 0 wait for the first actual time to arrive before starting the timer on
    // our sleep
    if (start.isZero())
    {
      start = ros::Time::now();
      end = start + duration;
    }

    // If time jumped backwards from when we started sleeping, return immediately
    if (ros::Time::now() < start)
      return false;
  }
  return rc && (ros::ok() || !ros::isInitialized()) && this->ok();
}

}
