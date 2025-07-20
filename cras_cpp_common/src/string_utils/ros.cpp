// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of cras::to_string() for ROS types and messages. Parsing of dates.
 * \author Martin Pecka
 */

#include <ctime>
#include <limits>
#include <regex>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include <ros/duration.h>
#include <ros/time.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

template<>
ros::Duration parseTimezoneOffset(const std::string& s)
{
  if (s.empty() || s == "Z")
  {
    return {0, 0};
  }

  const std::regex zoneOffsetRegex {R"(([+-]?)(\d{1,2}):?(\d{2}))"};
  std::smatch matches;
  if (!std::regex_match(s, matches, zoneOffsetRegex))
    throw std::invalid_argument("Invalid timezone offset string.");

  const auto sign = (matches[1].matched && matches[1].str() == "-") ? -1 : 1;
  const auto hours = cras::parseUInt8(matches[2].str(), 10);
  const auto minutes = cras::parseUInt8(matches[3].str(), 10);
  // *INDENT-OFF*
  return {sign * (hours * 3600 + minutes * 60), 0};
  // *INDENT-ON*
}

template<>
ros::WallDuration parseTimezoneOffset(const std::string& s)
{
  return cras::convertDuration<ros::WallDuration>(parseTimezoneOffset<ros::Duration>(s));
}

template<> ros::Time parseTime(
  const std::string& s, const cras::optional<ros::Duration>& timezoneOffset, const ros::Time& referenceDate)
{
  if (s.length() == 3 && cras::toLower(s) == "now")
    return ros::Time::now();

  // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
  // *INDENT-OFF*
  const std::regex delimitersRegex {
    R"((?:(?:(?:(\d+)[:_/-])?(\d+)[:_/-])?(\d+)[Tt _-])?(\d+)[:_/-](\d+)[:_/-](\d+)(?:[.,](\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"};  // NOLINT
  // *INDENT-ON*
  std::smatch matches;
  if (!std::regex_match(s, matches, delimitersRegex))
  {
    // *INDENT-OFF*
    const std::regex noDelimsRegex {
      R"((?:((?:\d{2}){1,2})[:_/-]?([01]\d)[:_/-]?([0123]\d)[Tt _-])?([012]\d)[:_/-]?([0-6]\d)[:_/-]?([0-6]\d)(?:[.,](\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"};  // NOLINT
    // *INDENT-ON*
    if (!std::regex_match(s, matches, noDelimsRegex))
      throw std::invalid_argument("Invalid time format");
  }

  const auto& boostDateRef = referenceDate.toBoost().date();

  std::string yearStr;
  if (matches[1].matched)
  {
    const auto& str = matches[1].str();
    if (str.length() == 2)
      yearStr = std::string("20") + str;
    else
      yearStr = str;
  }
  const uint16_t year = yearStr.empty() ? static_cast<uint16_t>(boostDateRef.year()) : cras::parseUInt16(yearStr, 10);
  if (year < 1970)
    throw std::invalid_argument("Years before 1970 cannot be parsed to ros time.");

  const auto month = matches[2].matched ?
    cras::parseUInt16(matches[2].str(), 10) : static_cast<uint16_t>(boostDateRef.month());
  if (month <= 0)
    throw std::invalid_argument("Month has to be a positive number (i.e. non-zero).");

  const auto day = matches[3].matched ?
    cras::parseUInt16(matches[3].str(), 10) : static_cast<uint16_t>(boostDateRef.day());
  if (day <= 0)
    throw std::invalid_argument("Day has to be a positive number (i.e. non-zero).");

  const auto hour = cras::parseUInt16(matches[4].str(), 10);
  const auto minute = cras::parseUInt16(matches[5].str(), 10);
  const auto second = cras::parseUInt16(matches[6].str(), 10);
  const auto zoneOffset = matches[8].matched ?
    cras::parseTimezoneOffset(matches[8].str()) : timezoneOffset.value_or(ros::Duration{});

  tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;

  const auto maybeTime = cras::fromStructTm(t);
  if (!maybeTime.has_value())
    throw std::invalid_argument(cras::format("Invalid time format (%s).", maybeTime.error().c_str()));

  uint32_t fracNsec = 0;
  if (matches[7].matched)
  {
    auto paddedNsec = matches[7].str();
    if (paddedNsec.length() < 9)
      paddedNsec = cras::format("%s%0*d", paddedNsec.c_str(), static_cast<int>(9 - paddedNsec.length()), 0);
    else if (paddedNsec.length() > 9)
      // We could correctly round here, but who cares about one ns?
      paddedNsec = paddedNsec.substr(0, 9);
    fracNsec = cras::parseUInt32(paddedNsec, 10);
  }

  return {maybeTime->sec - zoneOffset.sec, fracNsec};
}

template<> ros::WallTime parseTime(
  const std::string& s, const cras::optional<ros::WallDuration>& timezoneOffset, const ros::WallTime& referenceTime)
{
  if (s.length() == 3 && cras::toLower(s) == "now")
    return ros::WallTime::now();

  return convertTime<ros::WallTime>(parseTime(
    s,
    timezoneOffset.has_value() ? cras::optional{convertDuration<ros::Duration>(*timezoneOffset)} : cras::nullopt,
    convertTime<ros::Time>(referenceTime)));
}

template<> ros::SteadyTime parseTime(
  const std::string& s, const cras::optional<ros::WallDuration>& timezoneOffset, const ros::SteadyTime& referenceTime)
{
  if (s.length() == 3 && cras::toLower(s) == "now")
    return ros::SteadyTime::now();

  return convertTime<ros::SteadyTime>(parseTime(
    s,
    timezoneOffset.has_value() ? cras::optional{convertDuration<ros::Duration>(*timezoneOffset)} : cras::nullopt,
    convertTime<ros::Time>(referenceTime)));
}

template<> ros::Duration parseDuration(const std::string& s)
{
  // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
  const std::regex secondsOnlyRegex {R"(([+-]?)(\d+)(?:[.,](\d+))?)"};
  const std::regex delimitersRegex {R"(([+-]?)(?:(\d+)[:_/-])?(\d+)[:_/-](\d+)(?:[.,](\d+))?)"};

  std::string signString;
  uint32_t hours {0u};
  uint32_t minutes {0u};
  uint32_t seconds {0u};
  std::string nsecString;
  std::smatch matches;

  if (std::regex_match(s, matches, secondsOnlyRegex))
  {
    signString = matches[1].str();
    seconds = cras::parseUInt32(matches[2].str(), 10);
    nsecString = matches[3].matched ? matches[3].str() : "";
  }
  else if (std::regex_match(s, matches, delimitersRegex))
  {
    signString = matches[1].str();
    hours = matches[2].matched ? cras::parseUInt32(matches[2].str(), 10) : 0u;
    minutes = cras::parseUInt32(matches[3].str(), 10);
    seconds = cras::parseUInt32(matches[4].str(), 10);
    nsecString = matches[5].matched ? matches[5].str() : "";
  }
  else
  {
    throw std::invalid_argument("Invalid duration format.");
  }

  const int8_t sign = signString == "-" ? -1 : 1;

  int32_t fracNsec = 0;
  if (!nsecString.empty())
  {
    auto paddedNsec = nsecString;
    if (paddedNsec.length() < 9)
      paddedNsec = cras::format("%s%0*d", paddedNsec.c_str(), static_cast<int>(9 - paddedNsec.length()), 0);
    else if (paddedNsec.length() > 9)
      // We could correctly round here, but who cares about one ns?
      paddedNsec = paddedNsec.substr(0, 9);
    fracNsec = cras::parseInt32(paddedNsec, 10);
  }

  int64_t allSecs {0};  // Accumulate to int64_t to check for overflow
  allSecs += seconds;
  allSecs += minutes * 60;
  allSecs += hours * 3600;
  allSecs *= sign;

  if (allSecs < std::numeric_limits<int32_t>::min() || allSecs > std::numeric_limits<int32_t>::max())
    throw std::invalid_argument("Invalid duration (overflow).");

  return {static_cast<int32_t>(allSecs), fracNsec};
}

template<> ros::WallDuration parseDuration(const std::string& s)
{
  return cras::convertDuration<ros::WallDuration>(parseDuration<ros::Duration>(s));
}

template<>
std::string to_pretty_string(const ros::Time& value)
{
  return boost::posix_time::to_iso_extended_string(value.toBoost()) + "Z";
}

template<>
std::string to_pretty_string(const ros::WallTime& value)
{
  return to_pretty_string(cras::convertTime<ros::Time>(value));
}

template<>
std::string to_pretty_string(const ros::SteadyTime& value)
{
  return to_pretty_string(cras::convertTime<ros::Time>(value));
}

}
