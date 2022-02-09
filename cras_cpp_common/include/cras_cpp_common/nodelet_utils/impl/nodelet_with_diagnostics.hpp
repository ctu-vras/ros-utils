/**
 * \file
 * \brief Helpers for setting up diagnostics for nodelets (private implementation details, do not include this
 * directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include "../nodelet_with_diagnostics.hpp"

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <nodelet/nodelet.h>
#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/timer.h>

#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/diagnostics/ImprovedUpdater.h>

namespace cras
{

namespace impl
{

/**
 * \brief Private implementation (PIMPL) class for NodeletWithDiagnostics.
 */
struct NodeletWithDiagnosticsPrivate
{
  //! \brief The diagnostics updater used for the nodelet.
  ::std::shared_ptr<::diagnostic_updater::ImprovedUpdater> updater;

  //! \brief The diagnostics updater timer.
  ::ros::Timer timer;
};

}

template <typename NodeletType>
NodeletWithDiagnostics<NodeletType>::NodeletWithDiagnostics() : data(new ::cras::impl::NodeletWithDiagnosticsPrivate)
{
}

template <typename NodeletType>
NodeletWithDiagnostics<NodeletType>::~NodeletWithDiagnostics()
{
}

template <typename NodeletType>
::diagnostic_updater::Updater& NodeletWithDiagnostics<NodeletType>::getDiagUpdater() const
{
  if (this->data->updater == nullptr)
  {
    const auto* nodelet = dynamic_cast<const ::nodelet::Nodelet*>(this);
    if (nodelet != nullptr)
    {
      this->data->updater = ::std::make_shared<::diagnostic_updater::ImprovedUpdater>(
        // TODO if NodeletType::getNodeHandle() is used as first argument, we get a segfault on nodelet unload
        ::ros::NodeHandle(), NodeletType::getPrivateNodeHandle(), NodeletType::getName());
    }
    else
    {
      this->data->updater = ::std::make_shared<::diagnostic_updater::ImprovedUpdater>();
    }
  }
  return *this->data->updater;
}

template <typename NodeletType>
void NodeletWithDiagnostics<NodeletType>::startDiagTimer(const ::ros::NodeHandle& nh) const
{
  this->data->timer = nh.createTimer(::ros::Duration(1.0),
    [this](const ::ros::TimerEvent&) { this->getDiagUpdater().update(); });
}

template <typename NodeletType>
void NodeletWithDiagnostics<NodeletType>::stopDiagTimer() const
{
  this->data->timer.stop();
}

template <typename NodeletType>
::cras::BoundParamHelperPtr NodeletWithDiagnostics<NodeletType>::getDiagnosedPublisherParams(
  const ::ros::NodeHandle& nh, const ::std::string& paramNamespace)
{
  // if param helper is set up for this nodelet, use it to get the better logging using NODELET_ macros
  auto* params = dynamic_cast<::cras::NodeletParamHelper<NodeletType>*>(this);

  if (params != nullptr)
    return params->privateParams()->paramsInNamespace(paramNamespace);
  else
    return ::cras::nodeParams(NodeletType::getPrivateNodeHandle())->paramsInNamespace(paramNamespace);
}

}