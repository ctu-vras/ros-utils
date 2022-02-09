/**
 * \file
 * \brief Object implementing an `ok()` method that can interrupt pending sleeps when it returns false.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <mutex>

#include <ros/duration.h>

namespace cras
{

/**
 * \brief Interface to an object whose `sleep()` calls can be interrupted externally. Only one `sleep()` call can be
 * happening at a time (they are protected by mutex). The object waits for the last `sleep()` call to finish on
 * destruction. Make sure that `ok()` returns false when this object is about to be destroyed.
 */
struct InterruptibleSleepInterface
{
public:
  /**
   * \brief Destroy the object waiting for a pending `sleep()` call to finish.
   */
  virtual ~InterruptibleSleepInterface();

  /**
   * \brief Sleep for the given duration or until `ok()` returns false.
   * \param[in] duration The duration to sleep for.
   * \return Whether the requested duration has elapsed.
   */
  virtual bool sleep(const ::ros::Duration& duration) const;

  /**
   * \brief Whether it is OK to continue sleeping. If false, a pending `sleep()` should stop as soon as possible.
   * \return Whether it is OK to continue.
   * \note Always override this function as its default implementation returns false (used in case this function is
   *       called after the descendant parts of the objects have already been destructed).
   */
  virtual bool ok() const;

protected:
  //! \brief How long to wait between querying the `ok()` status and other conditions.
  ::ros::WallDuration pollDuration {0, 1000000};

private:
  //! \brief This mutex prevents the object to be destructed before a pending sleep finishes.
  mutable ::std::mutex mutex;
};

}