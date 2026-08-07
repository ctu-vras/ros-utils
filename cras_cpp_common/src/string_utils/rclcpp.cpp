// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of cras::to_string() for ROS types and messages. Parsing of dates.
 * \author Martin Pecka
 */

#include <chrono>
#include <limits>
#include <regex>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>

#include <cras_cpp_common/format.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/string_utils/rclcpp.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <rclcpp/exceptions/exceptions.hpp>

namespace cras {

std::string to_string(const rclcpp::ParameterValue& value) {
  switch (value.get_type()) {
    case rclcpp::PARAMETER_NOT_SET:
      return "NOT-SET";
    case rclcpp::PARAMETER_BOOL:
      return cras::to_string(value.get<bool>());
    case rclcpp::PARAMETER_INTEGER:
      return cras::to_string(value.get<int64_t>());
    case rclcpp::PARAMETER_DOUBLE:
      return cras::to_string(value.get<double>());
    case rclcpp::PARAMETER_STRING:
      return value.get<std::string>();
    case rclcpp::PARAMETER_BYTE_ARRAY:
      return cras::to_string(value.get<std::vector<uint8_t>>());
    case rclcpp::PARAMETER_BOOL_ARRAY:
      return cras::to_string(value.get<std::vector<bool>>());
    case rclcpp::PARAMETER_INTEGER_ARRAY:
      return cras::to_string(value.get<std::vector<int64_t>>());
    case rclcpp::PARAMETER_DOUBLE_ARRAY:
      return cras::to_string(value.get<std::vector<double>>());
    case rclcpp::PARAMETER_STRING_ARRAY:
      return cras::to_string(value.get<std::vector<std::string>>());
    default:
      return "UNKNOWN-PARAMETER-TYPE";
  }
}

std::string to_string(const rclcpp::Parameter& value) {
  return cras::format("{}: {}", value.get_name(), cras::to_string(value.get_parameter_value()));
}

rclcpp::Duration parseTimezoneOffset(const std::string& s) {
  if (s.empty() || s == "Z") {
    return {0, 0};
  }

  const std::regex zone_offset_regex {R"(([+-]?)(\d{1,2}):?(\d{2}))"};
  std::smatch matches;
  if (!std::regex_match(s, matches, zone_offset_regex)) {
    throw std::invalid_argument("Invalid timezone offset string.");
  }

  const auto sign = (matches[1].matched && matches[1].str() == "-") ? -1 : 1;
  const auto hours = cras::parseUInt8(matches[2].str(), 10);
  const auto minutes = cras::parseUInt8(matches[3].str(), 10);
  // *INDENT-OFF*
  return {sign * (hours * 3600 + minutes * 60), 0};
  // *INDENT-ON*
}

rclcpp::Time parseTime(
    const std::string& s, const std::optional<rclcpp::Duration>& timezone_offset,
    const rclcpp::Time& reference_date, const ::rclcpp::Clock::ConstSharedPtr& clock) {
  if (s.length() == 3 && cras::toLower(s) == "now") {
    const auto& cl = clock != nullptr ? *clock : ::rclcpp::Clock(RCL_SYSTEM_TIME);
    return cl.now();
  }

  // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
  // *INDENT-OFF*
  const std::regex delimiters_regex {
    R"((?:(?:(?:(\d+)[:_/-])?(\d+)[:_/-])?(\d+)[Tt _-])?(\d+)[:_/-](\d+)[:_/-](\d+)(?:[.,](\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"};  // NOLINT
  // *INDENT-ON*
  std::smatch matches;
  if (!std::regex_match(s, matches, delimiters_regex)) {
    // *INDENT-OFF*
    const std::regex no_delims_regex {
      R"((?:((?:\d{2}){1,2})[:_/-]?([01]\d)[:_/-]?([0123]\d)[Tt _-])?([012]\d)[:_/-]?([0-6]\d)[:_/-]?([0-6]\d)(?:[.,](\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"};  // NOLINT
    // *INDENT-ON*
    if (!std::regex_match(s, matches, no_delims_regex)) {
      throw std::invalid_argument("Invalid time format");
    }
  }

  std::chrono::seconds reference_seconds(static_cast<int64_t>(reference_date.seconds()));
  std::chrono::sys_seconds tp{reference_seconds};
  std::chrono::year_month_day ymd = std::chrono::floor<std::chrono::days>(tp);

  uint16_t ref_year = static_cast<uint16_t>(static_cast<int>(ymd.year()));
  uint16_t ref_month = static_cast<uint16_t>(static_cast<unsigned>(ymd.month()));
  uint16_t ref_day = static_cast<uint16_t>(static_cast<unsigned>(ymd.day()));

  std::string year_str;
  if (matches[1].matched) {
    const auto& str = matches[1].str();
    if (str.length() == 2) {
      year_str = std::string("20") + str;
    } else {
      year_str = str;
    }
  }
  const uint16_t year = year_str.empty() ? ref_year : cras::parseUInt16(year_str, 10);
  if (year < 1970) {
    throw std::invalid_argument("Years before 1970 cannot be parsed to ros time.");
  }

  const auto month = matches[2].matched ? cras::parseUInt16(matches[2].str(), 10) : ref_month;
  if (month <= 0) {
    throw std::invalid_argument("Month has to be a positive number (i.e. non-zero).");
  }

  const auto day = matches[3].matched ? cras::parseUInt16(matches[3].str(), 10) : ref_day;
  if (day <= 0) {
    throw std::invalid_argument("Day has to be a positive number (i.e. non-zero).");
  }

  const auto hour = cras::parseUInt16(matches[4].str(), 10);
  const auto minute = cras::parseUInt16(matches[5].str(), 10);
  const auto second = cras::parseUInt16(matches[6].str(), 10);
  const auto zone_offset =
    matches[8].matched ?
      cras::parseTimezoneOffset(matches[8].str()) : timezone_offset.value_or(rclcpp::Duration{0, 0});

  tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;

  const auto maybe_time = cras::fromStructTm(t);
  if (!maybe_time.has_value()) {
    throw std::invalid_argument(cras::format("Invalid time format ({}).", maybe_time.error()));
  }

  uint32_t frac_nsec = 0;
  if (matches[7].matched) {
    auto padded_nsec = matches[7].str();
    // pad with zeros to 9 decimals
    if (padded_nsec.length() < 9) {
      padded_nsec = cras::format("{:0<9}", padded_nsec);
    } else if (padded_nsec.length() > 9) {
      // We could correctly round here, but who cares about one ns?
      padded_nsec = padded_nsec.substr(0, 9);
    }
    frac_nsec = cras::parseUInt32(padded_nsec, 10);
  }

  const auto clock_type = clock != nullptr ? clock->get_clock_type() : RCL_SYSTEM_TIME;
  return {static_cast<int32_t>((*maybe_time - zone_offset).seconds()), frac_nsec, clock_type};
}

rclcpp::Duration parseDuration(const std::string& s) {
  // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
  const std::regex seconds_only_regex {R"(([+-]?)(\d+)(?:[.,](\d+))?)"};
  const std::regex delimiters_regex {R"(([+-]?)(?:(\d+)[:_/-])?(\d+)[:_/-](\d+)(?:[.,](\d+))?)"};

  std::string sign_string;
  uint32_t hours {0u};
  uint32_t minutes {0u};
  uint32_t seconds {0u};
  std::string nsec_string;
  std::smatch matches;

  if (std::regex_match(s, matches, seconds_only_regex)) {
    sign_string = matches[1].str();
    seconds = cras::parseUInt32(matches[2].str(), 10);
    nsec_string = matches[3].matched ? matches[3].str() : "";
  } else if (std::regex_match(s, matches, delimiters_regex)) {
    sign_string = matches[1].str();
    hours = matches[2].matched ? cras::parseUInt32(matches[2].str(), 10) : 0u;
    minutes = cras::parseUInt32(matches[3].str(), 10);
    seconds = cras::parseUInt32(matches[4].str(), 10);
    nsec_string = matches[5].matched ? matches[5].str() : "";
  } else {
    throw std::invalid_argument("Invalid duration format.");
  }

  const int8_t sign = sign_string == "-" ? -1 : 1;

  uint32_t frac_nsec = 0;
  if (!nsec_string.empty()) {
    auto padded_nsec = nsec_string;
    if (padded_nsec.length() < 9) {
      padded_nsec = cras::format("{:0<9}", padded_nsec);
    } else if (padded_nsec.length() > 9) {
      // We could correctly round here, but who cares about one ns?
      padded_nsec = padded_nsec.substr(0, 9);
    }
    frac_nsec = cras::parseUInt32(padded_nsec, 10);
  }

  int64_t all_secs {0};  // Accumulate to int64_t to check for overflow
  all_secs += seconds;
  all_secs += minutes * 60;
  all_secs += hours * 3600;
  all_secs *= sign;

  if (all_secs < std::numeric_limits<int32_t>::min() || all_secs > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument("Invalid duration (overflow).");
  }

  return {static_cast<int32_t>(all_secs), frac_nsec};
}

std::string to_string(const rclcpp::Time& value) {
  const auto [sec, nsec] = cras::secNsec(value);
  return cras::format("{}.{:09}", sec, nsec);
}

std::string to_string(const rclcpp::Duration& value) {
  const auto [sec, nsec] = cras::secNsec(value);
  return cras::format("{}.{:09}", sec, nsec);
}

template<>
std::string to_pretty_string(const rclcpp::Time& value) {
  const auto now = cras::convertTime<std::chrono::system_clock::time_point>(value);
  const auto [sec, nsec] = cras::secNsec(value);
  std::string sec_str = cras::format("{0:%S}", now);
  if (nsec != 0) {
    // Some implementations (fmt < 10.0) do not print subseconds for %S, so we have to check the result
    if (sec_str.length() >= 9) {
      sec_str = sec_str.substr(0, 9);
    } else {
      sec_str = sec_str.substr(0, 2);  // get only the seconds part
      const auto subsecStr = cras::format("{0:06}", nsec);
      sec_str = cras::format("{0}.{1:.6}", sec_str, subsecStr);  // use only first 6 digits from subsec
    }
  } else {
    sec_str = sec_str.substr(0, 2);
  }
  return cras::format("{0:%F}T{0:%R}:{1}Z", now, sec_str);
}

}  // namespace cras
