#pragma once

/**
 * \file
 * \brief A nodelet mixin that can report that it is being unloaded.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <nodelet/nodelet.h>
#include <ros/duration.h>

#include <cras_cpp_common/time_utils/interruptible_sleep_interface.h>

namespace cras
{

/**
 * \brief Tells whether the nodelet is currently being unloaded (with some pending callbacks still running).
 * \param[in] nodelet The nodelet to check.
 * \return Whether the nodelet is being unloaded.
 * \note Be sure to not call this function after the nodelet has finished unloading and its destructor has finished.
 */
bool isNodeletUnloading(const ::nodelet::Nodelet& nodelet);

/**
 * \brief A non-templated interface of the mixin that can tell when a nodelet is being unloaded.
 * \note InterruptibleSleepInterface also provides a `sleep()` method that should be used for sleeps inside the nodelet
 *       so that they are automatically interrupted on nodelet unload.
 */
struct StatefulNodeletInterface : public ::cras::InterruptibleSleepInterface
{
public:
  /**
   * \brief Call this function to request stopping this nodelet. `ok()` should return false after calling this. It
   *        terminates all ongoing sleeps called by `this->sleep()`.
   * \note This method is automatically called from the destructor (but rather call it as soon as you know the nodelet
   *       should be stopped).
   */
  virtual void requestStop() = 0;
};

/** 
 * \brief A mixin that can tell when a nodelet is being unloaded.
 * \tparam NodeletType Type of the base nodelet.
 */
template <typename NodeletType = ::nodelet::Nodelet>
class StatefulNodelet : public virtual NodeletType, public ::cras::StatefulNodeletInterface
{
public:
  virtual ~StatefulNodelet();
  bool ok() const override;
  void requestStop() override;

protected:
  using NodeletType::getName;

private:
  //! \brief Whether `requestStop()` has been called.
  volatile bool shouldStop = false;
};

}

#include "impl/stateful_nodelet.hpp"