/**
 * \file
 * \brief Utilities for working with ROS message files.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <ros/message_traits.h>
#include <ros/parameter_adapter.h>

namespace cras
{

//! \brief Return the base message type of M (i.e. removes MessageEvent, const, &, shared_ptr etc.).
template<typename M>
using BaseMessage = typename ::ros::ParameterAdapter<M>::Message;

//! \brief Tells whether the given signature designates a ROS message parameters (possibly surrounded by const, &,
//! shared_ptr, MessageEvent etc.).
template <typename M>
using IsMessageParam = ::ros::message_traits::IsMessage<::cras::BaseMessage<M>>;

}
