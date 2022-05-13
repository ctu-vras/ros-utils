/**
 * \file
 * \brief Utils for working with strings.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */
 
#include <string>
#include <sstream>
#include <vector>

#include <ros/console.h>
#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/string_utils.hpp>

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

void replace(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos)
  {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

std::string replace(const std::string& str, const std::string& from, const std::string& to)
{
  std::string s = str;
  cras::replace(s, from, to);
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

};
