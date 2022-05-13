#pragma once

/**
 * \file
 * \brief Specializations of cras::to_string() for XmlRpcValue values.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <xmlrpcpp/XmlRpcValue.h>

namespace cras
{

inline ::std::string to_string(const ::XmlRpc::XmlRpcValue& value)
{
  return value.toXml();
}

}
