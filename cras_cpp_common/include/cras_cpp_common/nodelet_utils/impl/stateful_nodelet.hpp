/**
 * \file
 * \brief A nodelet mixin that can report that it is being unloaded (private implementation details, do not include
 * this directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include "../stateful_nodelet.hpp"

#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/init.h>
#include <ros/time.h>

namespace cras
{

template <typename NodeletType>
::cras::StatefulNodelet<NodeletType>::~StatefulNodelet()
{
  this->requestStop();  // maybe this is superfluous as it might already be too late
}

template <typename NodeletType>
bool ::cras::StatefulNodelet<NodeletType>::ok() const
{
  return !this->shouldStop && !cras::isNodeletUnloading(*this);
}

template <typename NodeletType>
void ::cras::StatefulNodelet<NodeletType>::requestStop()
{
  if (!this->shouldStop)
    NODELET_INFO("Nodelet has been requested to stop.");
  this->shouldStop = true;
}

template <typename NodeletType>
void ::cras::StatefulNodelet<NodeletType>::shutdown()
{
  this->requestStop();
}

}