// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief QoS utilities.
 * \author Martin Pecka
 */

#include <cras_cpp_common/qos.hpp>

#include <rmw/qos_string_conversions.h>

#include <stdexcept>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

namespace cras {

rclcpp::QoS parseQoSPreset(const std::string& qos_preset) {
  if (qos_preset == "CLOCK") {
    return rclcpp::ClockQoS();
  }
  if (qos_preset == "SENSOR_DATA") {
    return rclcpp::SensorDataQoS();
  }
  if (qos_preset == "PARAMETERS") {
    return rclcpp::ParametersQoS();
  }
  if (qos_preset == "SERVICES") {
    return rclcpp::ServicesQoS();
  }
  if (qos_preset == "PARAMETER_EVENTS") {
    return rclcpp::ParameterEventsQoS();
  }
  if (qos_preset == "ROSOUT") {
    return rclcpp::RosoutQoS();
  }
  if (qos_preset == "SYSTEM_DEFAULT") {
    return rclcpp::SystemDefaultsQoS();
  }
  if (qos_preset == "BEST_AVAILABLE") {
    return rclcpp::BestAvailableQoS();
  }

  throw std::invalid_argument(std::string("Invalid QoS preset '") + qos_preset + "'");
}

bool configureQoSProfile(rclcpp::QoS& profile, const std::optional<int>& depth,
    const std::optional<std::string>& history, const std::optional<std::string>& reliability,
    const std::optional<std::string>& durability, const std::optional<std::string>& liveliness,
    const std::optional<double>& liveliness_lease_duration_seconds) {
  std::optional<size_t> depth_value;
  if (depth.has_value()) {
    if (*depth < 0) {
      return false;
    }
    depth_value = static_cast<size_t>(*depth);
  }

  std::optional<rmw_qos_history_policy_t> history_value;
  if (history.has_value()) {
    history_value = rmw_qos_history_policy_from_str(history->c_str());
    if (history_value == RMW_QOS_POLICY_HISTORY_UNKNOWN) {
      return false;
    }
  }

  std::optional<rmw_qos_reliability_policy_t> reliability_value;
  if (reliability.has_value()) {
    reliability_value = rmw_qos_reliability_policy_from_str(reliability->c_str());
    if (reliability_value == RMW_QOS_POLICY_RELIABILITY_UNKNOWN) {
      return false;
    }
  }

  std::optional<rmw_qos_durability_policy_t> durability_value;
  if (durability.has_value()) {
    durability_value = rmw_qos_durability_policy_from_str(durability->c_str());
    if (durability_value == RMW_QOS_POLICY_DURABILITY_UNKNOWN) {
      return false;
    }
  }

  std::optional<rmw_qos_liveliness_policy_t> liveliness_value;
  if (liveliness.has_value()) {
    liveliness_value = rmw_qos_liveliness_policy_from_str(liveliness->c_str());
    if (liveliness_value == RMW_QOS_POLICY_LIVELINESS_UNKNOWN) {
      return false;
    }
  }

  std::optional<rclcpp::Duration> liveliness_lease_duration_value;
  if (liveliness_lease_duration_value.has_value()) {
    liveliness_lease_duration_value = rclcpp::Duration(
      std::chrono::duration<float>(*liveliness_lease_duration_seconds));
  }

  configureQoSProfile(profile, depth_value, history_value, reliability_value, durability_value,
                      liveliness_value, liveliness_lease_duration_value);

  return true;
}

void configureQoSProfile(rclcpp::QoS& profile, const std::optional<size_t>& depth,
    const std::optional<rmw_qos_history_policy_t>& history,
    const std::optional<rmw_qos_reliability_policy_t>& reliability,
    const std::optional<rmw_qos_durability_policy_t>& durability,
    const std::optional<rmw_qos_liveliness_policy_t>& liveliness,
    const std::optional<rclcpp::Duration>& liveliness_lease_duration) {
  if (history.has_value()) {
    profile.history(*history);
  }
  if (depth.has_value() && profile.history() == rclcpp::HistoryPolicy::KeepLast) {
    profile.keep_last(*depth);
  }
  if (reliability.has_value()) {
    profile.reliability(*reliability);
  }
  if (durability.has_value()) {
    profile.durability(*durability);
  }
  if (liveliness.has_value()) {
    profile.liveliness(*liveliness);
  }
  if (liveliness_lease_duration.has_value()) {
    profile.liveliness_lease_duration(*liveliness_lease_duration);
  }
}

}  // namespace cras
