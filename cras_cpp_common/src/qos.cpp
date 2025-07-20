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

namespace cras
{

rclcpp::QoS parseQoSPreset(const std::string& qosPreset)
{
  if (qosPreset == "CLOCK")
    return rclcpp::ClockQoS();
  if (qosPreset == "SENSOR_DATA")
    return rclcpp::SensorDataQoS();
  if (qosPreset == "PARAMETERS")
    return rclcpp::ParametersQoS();
  if (qosPreset == "SERVICES")
    return rclcpp::ServicesQoS();
  if (qosPreset == "PARAMETER_EVENTS")
    return rclcpp::ParameterEventsQoS();
  if (qosPreset == "ROSOUT")
    return rclcpp::RosoutQoS();
  if (qosPreset == "SYSTEM_DEFAULT")
    return rclcpp::SystemDefaultsQoS();
  if (qosPreset == "BEST_AVAILABLE")
    return rclcpp::BestAvailableQoS();

  throw std::invalid_argument(std::string("Invalid QoS preset '") + qosPreset + "'");
}

bool configureQoSProfile(rclcpp::QoS& profile, const std::optional<int>& depth,
  const std::optional<std::string>& history, const std::optional<std::string>& reliability,
  const std::optional<std::string>& durability, const std::optional<std::string>& liveliness,
  const std::optional<double>& livelinessLeaseDurationSeconds)
{
  if (depth.value_or(0) < 0)
    return false;
  std::optional<size_t> depthValue {static_cast<size_t>(*depth)};

  std::optional<rmw_qos_history_policy_t> historyValue;
  if (history.has_value())
  {
    historyValue = rmw_qos_history_policy_from_str(history->c_str());
    if (historyValue == RMW_QOS_POLICY_HISTORY_UNKNOWN)
      return false;
  }

  std::optional<rmw_qos_reliability_policy_t> reliabilityValue;
  if (reliability.has_value())
  {
    reliabilityValue = rmw_qos_reliability_policy_from_str(reliability->c_str());
    if (reliabilityValue == RMW_QOS_POLICY_RELIABILITY_UNKNOWN)
      return false;
  }

  std::optional<rmw_qos_durability_policy_t> durabilityValue;
  if (durability.has_value())
  {
    durabilityValue = rmw_qos_durability_policy_from_str(durability->c_str());
    if (durabilityValue == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
      return false;
  }

  std::optional<rmw_qos_liveliness_policy_t> livelinessValue;
  if (liveliness.has_value())
  {
    livelinessValue = rmw_qos_liveliness_policy_from_str(liveliness->c_str());
    if (livelinessValue == RMW_QOS_POLICY_LIVELINESS_UNKNOWN)
      return false;
  }

  std::optional<rclcpp::Duration> livelinessLeaseDurationValue;
  if (livelinessLeaseDurationValue.has_value())
    livelinessLeaseDurationValue = rclcpp::Duration(std::chrono::duration<float>(*livelinessLeaseDurationSeconds));

  configureQoSProfile(profile, depthValue, historyValue, reliabilityValue, durabilityValue,
    livelinessValue, livelinessLeaseDurationValue);

  return true;
}

void configureQoSProfile(rclcpp::QoS& profile, const std::optional<size_t>& depth,
  const std::optional<rmw_qos_history_policy_t>& history,
  const std::optional<rmw_qos_reliability_policy_t>& reliability,
  const std::optional<rmw_qos_durability_policy_t>& durability,
  const std::optional<rmw_qos_liveliness_policy_t>& liveliness,
  const std::optional<rclcpp::Duration>& livelinessLeaseDuration)
{
  if (history.has_value())
    profile.history(*history);
  if (depth.has_value() && profile.history() == rclcpp::HistoryPolicy::KeepLast)
    profile.keep_last(*depth);
  if (reliability.has_value())
    profile.reliability(*reliability);
  if (durability.has_value())
    profile.durability(*durability);
  if (liveliness.has_value())
    profile.liveliness(*liveliness);
  if (livelinessLeaseDuration.has_value())
    profile.liveliness_lease_duration(*livelinessLeaseDuration);
}

}
