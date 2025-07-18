#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief QoS utilities.
 * \author Martin Pecka
 */

#include <optional>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>

namespace cras
{

/**
 * \brief Parse a QoS preset name into the QoS object.
 * \param[in] qosPreset The QoS profile string to parse (uppercase).
 * \return The parsed QoS preset.
 * \throws std::invalid_argument If the string does not represent any QoS preset.
 */
::rclcpp::QoS parseQoSPreset(const ::std::string& qosPreset);

bool configureQoSProfile(::rclcpp::QoS& profile, const ::std::optional<int>& depth,
  const ::std::optional<::std::string>& history, const ::std::optional<::std::string>& reliability,
  const ::std::optional<::std::string>& durability, const ::std::optional<::std::string>& liveliness,
  const ::std::optional<double>& livelinessLeaseDurationSeconds);

void configureQoSProfile(::rclcpp::QoS& profile, const ::std::optional<size_t>& depth,
  const ::std::optional<::rmw_qos_history_policy_t>& history,
  const ::std::optional<::rmw_qos_reliability_policy_t>& reliability,
  const ::std::optional<::rmw_qos_durability_policy_t>& durability,
  const ::std::optional<::rmw_qos_liveliness_policy_t>& liveliness,
  const ::std::optional<::rclcpp::Duration>& livelinessLeaseDuration);

}
