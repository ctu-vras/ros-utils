/**
 * \file
 * \brief Log helper redirecting the logging calls to NODELET_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <nodelet/nodelet.h>

#include <cras_cpp_common/log_utils/nodelet.h>

namespace cras
{

const std::string NOT_SET {"getNameFn is not set!"};

NodeletLogHelper::NodeletLogHelper(const GetNameFn& getNameFn) : getNameFn(getNameFn)
{
}

NodeletLogHelper::~NodeletLogHelper() = default;

const std::string& NodeletLogHelper::getName() const
{
  if (this->getNameFn)
    return this->getNameFn();
  else
    return NOT_SET;
}

std::string NodeletLogHelper::getSuffixedName(const std::string& suffix) const
{
  return this->getNameFn() + "." + suffix;
}

}