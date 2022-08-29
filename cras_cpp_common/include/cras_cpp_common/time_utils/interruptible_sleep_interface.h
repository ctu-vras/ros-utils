#pragma once

/**
 * \file
 * \brief Object implementing an `ok()` method that can interrupt pending sleeps when it returns false.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>

#include <ros/duration.h>

namespace cras
{

struct InterruptibleSleepInterfacePrivate;

/**
 * \brief Interface to an object whose `sleep()` calls can be interrupted externally. Multiple `sleep()` calls can be
 * happening at a time. The object waits for the last `sleep()` call to finish on destruction. No more `sleep()` calls
 * can be made once destruction of the object started (`sleep()` will return false in such case).
 * Make sure that `ok()` returns false when this object is about to be destroyed.
 */
struct InterruptibleSleepInterface
{
public:
  InterruptibleSleepInterface();

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
  //! \brief PIMPL data container.
  ::std::unique_ptr<InterruptibleSleepInterfacePrivate> data;
};

}
