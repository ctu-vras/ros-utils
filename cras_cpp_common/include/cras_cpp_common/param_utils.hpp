#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This file provides helper methods for working with parameters.
 * \author Martin Pecka
 */

#include <optional>

#include <rclcpp/parameter_value.hpp>

namespace cras
{

template<typename T>
::std::optional<T> maybeParam(const ::rclcpp::ParameterValue& value)
{
  if (value.get_type() == ::rclcpp::PARAMETER_NOT_SET)
    return ::std::nullopt;

  try
  {
    return value.get<T>();
  }
  catch (const ::rclcpp::ParameterTypeException& e)
  {
    return ::std::nullopt;
  }
}

}
