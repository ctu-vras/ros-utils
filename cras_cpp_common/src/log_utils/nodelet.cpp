// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Log helper redirecting the logging calls to NODELET_ macros.
 * \author Martin Pecka
 */

#include <string>

#include <cras_cpp_common/log_utils/nodelet.h>
#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

const static std::string NOT_SET {"getNameFn is not set!"};  // NOLINT

NodeletLogHelper::NodeletLogHelper(const GetNameFn& getNameFn) : getNameFn(getNameFn)
{
  if (!this->getNameFn)
    this->getNameFn = []() -> const std::string& { return NOT_SET; };
}

void NodeletLogHelper::initializeLogLocationImpl(
  ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const
{
  // We need to put getNameFn() in between the base name and the added suffix

  auto newName = name;
  const auto prefix = std::string(ROSCONSOLE_NAME_PREFIX) + ".";
  if (cras::startsWith(name, prefix))
  {
    const auto nodeletPrefix = prefix + this->getNameFn() + ".";
    if (!cras::startsWith(name, nodeletPrefix))
      newName = nodeletPrefix + cras::removePrefix(name, prefix);
  }
  else if (name == ROSCONSOLE_NAME_PREFIX)
    newName = name + "." + this->getNameFn();

  RosconsoleLogHelper::initializeLogLocationImpl(loc, newName, level);
}

}
