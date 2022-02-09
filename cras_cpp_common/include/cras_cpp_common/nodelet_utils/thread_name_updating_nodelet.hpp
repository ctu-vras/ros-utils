/**
 * \file
 * \brief ThreadNameUpdatingNodelet mixin allows nodelet to update the name of the thread it gets executed in. 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <nodelet/nodelet.h>

namespace cras
{

/**
 * \brief This mixin allows the nodelet to update the OS name of the thread in which it is currently executing.
 * \tparam NodeletType Type of the base nodelet.
 */
template<typename NodeletType = ::nodelet::Nodelet>
class ThreadNameUpdatingNodelet : public virtual NodeletType
{
public:
  ~ThreadNameUpdatingNodelet() override;

protected:
  /**
   * \brief Set custom name of the current thread to this nodelet's name.
   *
   * \note The name will be automatically shortened if longer than 15 chars.
   * \note You can see the custom names in htop when you enable display of custom thread names in options.
   * \note This function doesn't reset the name back to the original.
   * \note You should call this function at the beginning of all your callbacks.
   * \note This function is actually pretty fast. It can be called more than 1 million times per second.
   */
  void updateThreadName() const;
};

}

#include "impl/thread_name_updating_nodelet.hpp"