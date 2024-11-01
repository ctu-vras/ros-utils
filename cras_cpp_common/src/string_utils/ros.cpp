// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Specializations of cras::to_string() for ROS types and messages. Parsing of dates.
 * \author Martin Pecka
 */

#include <regex>
#include <string>
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
  if (s.empty() || s =="Z")
    return {0, 0};

  const std::regex zoneOffsetRegex {R"(([+-]?)(\d{1,2}):?(\d{2}))"};
  std::smatch matches;
  if (!std::regex_match(s, matches, zoneOffsetRegex))
    throw std::invalid_argument("Invalid timezone offset string.");

  const auto sign = (matches[1].matched && matches[1].str() == "-") ? -1 : 1;
  const auto hours = cras::parseUInt8(matches[2].str(), 10);
  const auto minutes = cras::parseUInt8(matches[3].str(), 10);
  return {sign * (hours * 3600 + minutes * 60), 0};
}

template<>
ros::WallDuration parseTimezoneOffset(const std::string& s)
{
  return cras::convertDuration<ros::WallDuration>(parseTimezoneOffset<ros::Duration>(s));
}

template<> ros::Time parseTime(
  const std::string& s, const cras::optional<ros::Duration>& timezoneOffset, const ros::Time& referenceDate)
{
  // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
  const std::regex delimitersRegex {
    R"((?:(?:(?:(\d+)[:_/-])?(\d+)[:_/-])?(\d+)[Tt _-])?(\d+)[:_/-](\d+)[:_/-](\d+)(?:\.(\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"};  // NOLINT
  std::smatch matches;
  if (!std::regex_match(s, matches, delimitersRegex))
  {
    const std::regex noDelimsRegex {
      R"((?:((?:\d{2}){1,2})[:_/-]?([01]\d)[:_/-]?([0123]\d)[Tt _-])?([012]\d)[:_/-]?([0-6]\d)[:_/-]?([0-6]\d)(?:\.(\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"};  // NOLINT
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
  const auto day = matches[3].matched ?
    cras::parseUInt16(matches[3].str(), 10) : static_cast<uint16_t>(boostDateRef.day());
  const auto hour = cras::parseUInt16(matches[4].str(), 10);
  const auto minute = cras::parseUInt16(matches[5].str(), 10);
  const auto second = cras::parseUInt16(matches[6].str(), 10);
  const auto zoneOffset = matches[8].matched ?
    cras::parseTimezoneOffset(matches[8].str()) : timezoneOffset.value_or(ros::Duration::ZERO);

  tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;

  errno = 0;
  int32_t timeSecs = timegm(&t);
  if (timeSecs < 0 || errno == EOVERFLOW)
    throw std::invalid_argument("Invalid time format (timegm overflow).");
  timeSecs -= zoneOffset.sec;

  uint32_t fracNsec = 0;
  if (matches[7].matched)
  {
    const auto paddedNsec = cras::format("%s%0*d", matches[7].str().c_str(), 9 - matches[7].length(), 0);
    fracNsec = cras::parseUInt32(paddedNsec, 10);
  }

  return {static_cast<uint32_t>(timeSecs), fracNsec};
}

template<> ros::WallTime parseTime(
  const std::string& s, const cras::optional<ros::WallDuration>& timezoneOffset, const ros::WallTime& referenceTime)
{
  return convertTime<ros::WallTime>(parseTime(
    s,
    timezoneOffset.has_value() ? cras::optional{convertDuration<ros::Duration>(*timezoneOffset)} : cras::nullopt,
    convertTime<ros::Time>(referenceTime)));
}

template<> ros::SteadyTime parseTime(
  const std::string& s, const cras::optional<ros::WallDuration>& timezoneOffset, const ros::SteadyTime& referenceTime)
{
  return convertTime<ros::SteadyTime>(parseTime(
    s,
    timezoneOffset.has_value() ? cras::optional{convertDuration<ros::Duration>(*timezoneOffset)} : cras::nullopt,
    convertTime<ros::Time>(referenceTime)));
}

template<> ros::Duration parseDuration(const std::string& s)
{
  // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
  const std::regex delimitersRegex {
    R"((?:(?:(?:(\d+)[:_/-])?(\d+)[:_/-])?(\d+)[Tt _-])?(\d+)[:_/-](\d+)[:_/-](\d+)(?:\.(\d+))?)"};
  std::smatch matches;
  if (!std::regex_match(s, matches, delimitersRegex))
  {
    const std::regex noDelimsRegex {
      R"(((?:\d{2}){1,2})[:_/-]?([01]\d)[:_/-]?([0123]\d)[Tt _-]?([012]\d)[:_/-]?([0-6]\d)[:_/-]?([0-6]\d)(?:\.(\d+))?)"};
    if (!std::regex_match(s, matches, noDelimsRegex))
      throw std::invalid_argument("Invalid duration format");
  }

  const auto year = matches[1].matched ? cras::parseUInt16(matches[1].str(), 10) + 1970 : 1970;
  const auto month = matches[2].matched ? cras::parseUInt16(matches[2].str(), 10) + 1 : 1;
  const auto day = matches[3].matched ? cras::parseUInt16(matches[3].str(), 10) + 1 : 1;
  const auto hour = cras::parseUInt16(matches[4].str(), 10);
  const auto minute = cras::parseUInt16(matches[5].str(), 10);
  const auto second = cras::parseUInt16(matches[6].str(), 10);

  tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  errno = 0;
  const int32_t timeSecs = timegm(&t);
  if (timeSecs < 0 || errno == EOVERFLOW)
    throw std::invalid_argument("Invalid duration format (timegm overflow).");

  int32_t fracNsec = 0;
  if (matches[7].matched)
  {
    const auto paddedNsec = cras::format("%s%0*d", matches[7].str().c_str(), 9 - matches[7].length(), 0);
    fracNsec = cras::parseInt32(paddedNsec, 10);
  }

  return {timeSecs, fracNsec};
}

template<> ros::WallDuration parseDuration(const std::string& s)
{
  return cras::convertDuration<ros::WallDuration>(parseDuration<ros::Duration>(s));
}

}
