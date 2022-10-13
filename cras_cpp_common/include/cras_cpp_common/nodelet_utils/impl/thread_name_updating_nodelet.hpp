/**
 * \file
 * \brief ThreadNameUpdatingNodelet mixin allows nodelet to update the name of the thread it gets executed in (private
 * implementation details, do not include this directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include "../thread_name_updating_nodelet.hpp"

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/thread_utils.h>

namespace cras
{

template<typename NodeletType>
ThreadNameUpdatingNodelet<NodeletType>::~ThreadNameUpdatingNodelet() = default;

template<typename NodeletType>
void ThreadNameUpdatingNodelet<NodeletType>::updateThreadName() const
{
  ::cras::setThreadName(::cras::stripLeadingSlash(NodeletType::getName()));
}

}
