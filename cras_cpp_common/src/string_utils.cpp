/**
 * \file
 * \brief Utils for working with strings.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <charconv>
#include <string>
#include <sstream>
#include <vector>

#include <ros/console.h>
#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/string_utils/from_chars.h>

namespace cras
{

void warnLeadingSlash(const std::string& s)
{
  ROS_WARN_STREAM("Found initial slash in " << s);
}

void stripLeading(std::string& s, const char& c)
{
  if (s.length() > 0 && s[0] == c)
    s.erase(0, 1);
}

void stripTrailing(std::string& s, const char& c)
{
  if (s.length() > 0 && s[s.length() - 1] == c)
    s.pop_back();
}

void stripLeadingSlash(std::string& s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn)
      warnLeadingSlash(s);
    s.erase(0, 1);
  }
}

std::string stripLeading(const std::string& s, const char& c)
{
  if (s.length() > 0 && s[0] == c)
    return s.substr(1);

  return s;
}

std::string stripTrailing(const std::string& s, const char& c)
{
  if (s.length() > 0 && s[s.length() - 1] == c)
    return s.substr(0, s.length() - 1);

  return s;
}

std::string stripLeadingSlash(const std::string& s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn)
      warnLeadingSlash(s);
    return s.substr(1);
  }

  return s;
}

std::string removePrefix(const std::string& str, const std::string& prefix, bool* hadPrefix)
{
  const auto hasPrefix = startsWith(str, prefix);
  if (hadPrefix != nullptr)
    *hadPrefix = hasPrefix;

  return hasPrefix ? str.substr(prefix.length()) : str;
}

std::string removeSuffix(const std::string& str, const std::string& suffix, bool* hadSuffix)
{
  const auto hasSuffix = endsWith(str, suffix);
  if (hadSuffix != nullptr)
    *hadSuffix = hasSuffix;

  return hasSuffix ? str.substr(0, str.length() - suffix.length()) : str;
}

std::string prependIfNonEmpty(const std::string& str, const std::string& prefix)
{
  return str.empty() ? str : prefix + str;
}

std::string appendIfNonEmpty(const std::string& str, const std::string& suffix)
{
  return str.empty() ? str : str + suffix;
}

bool startsWith(const std::string& str, const std::string& prefix)
{
  return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
}

bool endsWith(const std::string& str, const std::string& suffix)
{
  return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void replace(std::string& str, const std::string& from, const std::string& to, const ::cras::ReplacePosition& where)
{
  size_t startPos = 0;
  while ((startPos = str.find(from, startPos)) != std::string::npos)
  {
    if (where == cras::ReplacePosition::START && startPos != 0)
      break;
    const auto endPos = startPos + from.length();
    if (where == cras::ReplacePosition::END && endPos != str.length())
    {
      startPos += 1;
      continue;
    }
    str.replace(startPos, from.length(), to);
    startPos += to.length();
  }
}

std::string replace(const std::string& str, const std::string& from, const std::string& to,
  const ::cras::ReplacePosition& where)
{
  std::string s = str;
  cras::replace(s, from, to, where);
  return s;
}

bool contains(const std::string& str, char c)
{
  return str.find_first_of(c) != std::string::npos;
}

bool contains(const std::string& str, const std::string& needle)
{
  return str.length() >= needle.length() && str.find(needle) != std::string::npos;
}

std::vector<std::string> split(const std::string& str, const std::string& delimiter, int maxSplits)
{
  // inspired by https://stackoverflow.com/a/46931770/1076564, CC-BY-SA 4.0
  // renamed some variables, added the maxSplits option
  size_t start{0};
  size_t end;
  size_t delimiterLength{delimiter.length()};
  std::string token;
  std::vector<std::string> result;

  while ((end = str.find(delimiter, start)) != std::string::npos && (maxSplits == -1 || result.size() < maxSplits))
  {
    token = str.substr(start, end - start);
    start = end + delimiterLength;
    result.push_back(token);
  }

  result.push_back(str.substr(start));
  return result;
}

template <typename T, ::std::enable_if_t<::std::is_integral_v<::std::decay_t<T>>, bool> = true>
inline T parseIntegralNumber(const std::string& string)
{
  T result{};
  
  auto cleanString = cras::stripLeading(string, ' ');
  cras::stripLeading(cleanString, '+');
  cras::stripTrailing(cleanString, ' ');
  
  auto noSignString = cleanString;
  cras::stripLeading(noSignString, '-');
  auto base = 10;
  if (noSignString.length() > 2 && noSignString[0] == '0')
  {
    if (noSignString[1] == 'x' || noSignString[1] == 'X')
    {
      base = 16;
      cras::stripLeading(noSignString, '0');
      cras::stripLeading(noSignString, 'x');
      cras::stripLeading(noSignString, 'X');
    }
    else if (noSignString[1] == 'b' || noSignString[1] == 'B')
    {
      base = 2;
      cras::stripLeading(noSignString, '0');
      cras::stripLeading(noSignString, 'b');
      cras::stripLeading(noSignString, 'B');
    }
    else
    {
      base = 8;
      cras::stripLeading(noSignString, '0');
    }
    cleanString = cleanString[0] == '-' ? ("-" + noSignString) : noSignString;
  }
  else if (noSignString.length() > 1 && noSignString[0] == '0')
  {
    base = 8;
    cras::stripLeading(noSignString, '0');
    cleanString = cleanString[0] == '-' ? ("-" + noSignString) : noSignString;
  }

  auto [ptr, ec] = std::from_chars(cleanString.data(), cleanString.data() + cleanString.size(), result, base);

  if (ec == std::errc())
  {
    if (ptr == cleanString.data() + cleanString.size())
      return result;
    throw std::invalid_argument("Passed string contains excess characters: '" + string + "'");
  }
  else if (ec == std::errc::invalid_argument)
  {
    throw std::invalid_argument("Passed string is not a number: '" + string + "'");
  }
  else if (ec == std::errc::result_out_of_range)
  {
    throw std::invalid_argument("Passed string is out of range: '" + string + "'");
  }
  throw std::runtime_error("Unexpected case");
}

int8_t parseInt8(const std::string& string)
{
  return cras::parseIntegralNumber<int8_t>(string);
}

uint8_t parseUInt8(const std::string& string)
{
  return cras::parseIntegralNumber<uint8_t>(string);
}

int16_t parseInt16(const std::string& string)
{
  return cras::parseIntegralNumber<int16_t>(string);
}

uint16_t parseUInt16(const std::string& string)
{
  return cras::parseIntegralNumber<uint16_t>(string);
}

int32_t parseInt32(const std::string& string)
{
  return cras::parseIntegralNumber<int32_t>(string);
}

uint32_t parseUInt32(const std::string& string)
{
  return cras::parseIntegralNumber<uint32_t>(string);
}

int64_t parseInt64(const std::string& string)
{
  return cras::parseIntegralNumber<int64_t>(string);
}

uint64_t parseUInt64(const std::string& string)
{
  return cras::parseIntegralNumber<uint64_t>(string);
}

template <typename T, ::std::enable_if_t<::std::is_floating_point_v<::std::decay_t<T>>, bool> = true>
inline T parseFloatingNumber(const std::string& string)
{
  T result{};
  
  auto cleanString = cras::stripLeading(string, ' ');
  cras::stripLeading(cleanString, '+');
  cras::stripTrailing(cleanString, ' ');

  auto [ptr, ec] = cras::from_chars(cleanString, result);

  if (ec == std::errc())
  {
    if (ptr == cleanString.data() + cleanString.size())
      return result;
    throw std::invalid_argument("Passed string contains excess characters: '" + string + "'");
  }
  else if (ec == std::errc::invalid_argument)
  {
    throw std::invalid_argument("Passed string is not a number: '" + string + "'");
  }
  else if (ec == std::errc::result_out_of_range)
  {
    throw std::invalid_argument("Passed string is out of range: '" + string + "'");
  }
  throw std::runtime_error("Unexpected case");
}

float parseFloat(const std::string& string)
{
  return cras::parseFloatingNumber<float>(string);
}

double parseDouble(const std::string& string)
{
  return cras::parseFloatingNumber<double>(string);
}

};
