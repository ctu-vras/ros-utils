/**
 * \file
 * \brief Log helper redirecting the logging calls to NODELET_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <nodelet/nodelet.h>

#include <cras_cpp_common/log_utils/nodelet.h>

using namespace ::cras;

NodeletLogHelper::NodeletLogHelper(const GetNameFn& getNameFn) : getNameFn(getNameFn)
{
}

NodeletLogHelper::~NodeletLogHelper() = default;

void NodeletLogHelper::printDebug(const ::std::string& text) const
{
  NODELET_DEBUG("%s", text.c_str());
}

void NodeletLogHelper::printInfo(const ::std::string& text) const
{
  NODELET_INFO("%s", text.c_str());
}

void NodeletLogHelper::printWarn(const ::std::string& text) const
{
  NODELET_WARN("%s", text.c_str());
}

void NodeletLogHelper::printError(const ::std::string& text) const
{
  NODELET_ERROR("%s", text.c_str());
}

void NodeletLogHelper::printFatal(const ::std::string& text) const
{
  NODELET_FATAL("%s", text.c_str());
}

const ::std::string& NodeletLogHelper::getName() const
{
  return this->getNameFn();
}
