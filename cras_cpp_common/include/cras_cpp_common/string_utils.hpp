#pragma once

#include <cstdarg>
#include <iterator>
#include <map>
#include <ros/duration.h>
#include <ros/time.h>
#include <set>
#include <stdio.h>
#include <string>
#include <sstream>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace cras {

/**
 * \brief Strip leading slash from the given string (if there is one).
 * \param s The string from which slash should be removed.
 * \param warn If true, issue a ROS warning if the string contained the leading slash.
 */
void stripLeadingSlash(std::string& s, bool warn = false);

/**
 * \brief Return a copy of the given string with leading slash removed (if there is one).
 * \param s The string from which slash should be removed.
 * \param warn If true, issue a ROS warning if the string contained the leading slash.
 * \return The string without leading slash.
 */
std::string stripLeadingSlash(const std::string& s, bool warn = false);

/**
 * \brief If `str` is nonempty, returns prefix + str, otherwise empty string.
 * \param str The main string.
 * \param prefix The string's prefix.
 * \return The possibly prefixed string.
 */
std::string prependIfNonEmpty(const std::string& str, const std::string& prefix);

/**
 * \brief If `str` is nonempty, returns str + suffix, otherwise empty string.
 * \param str The main string.
 * \param suffix The string's suffix.
 * \return The possibly suffixed string.
 */
std::string appendIfNonEmpty(const std::string& str, const std::string& suffix);

/**
 * \brief Check whether `prefix` is a prefix of `str`.
 * \param str The string to be searched in.
 * \param prefix The string to be found in `str`.
 * \return Whether `prefix` is a prefix of `str`.
 */
bool startsWith(const std::string& str, const std::string& prefix);

/**
 * \brief Check whether `suffix` is a suffix of `str`.
 * \param str The string to be searched in.
 * \param suffix The string to be found in `str`.
 * \return Whether `suffix` is a suffix of `str`.
 */
bool endsWith(const std::string& str, const std::string& suffix);

template<typename T>
inline std::string to_string(const T& value)
{
  return std::to_string(value);
}

template<>
inline std::string to_string(const bool& value)
{
  return value ? "True" : "False";
}

template<>
inline std::string to_string(const std::string& value)
{
  return value;
}

template<>
inline std::string to_string(const XmlRpc::XmlRpcValue& value)
{
  return value.toXml();
}

template<typename T>
inline std::string to_string(const std::vector<T>& value)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < value.size(); ++i)
  {
    ss << to_string(value[i]);
    if (i + 1 < value.size())
      ss << ", ";
  }
  ss << "]";
  return ss.str();
}

template<typename T>
inline std::string to_string(const std::set<T> &value)
{
  std::stringstream ss;
  ss << "[";
  size_t i = 0;
  for (const auto& v : value)
  {
    ss << to_string(v);
    if (i + 1 < value.size())
      ss << ", ";
    ++i;
  }
  ss << "]";
  return ss.str();
}

template<typename K, typename V>
inline std::string to_string(const std::map<K, V>& value)
{
  std::stringstream ss;
  ss << "{";
  size_t i = 0;
  for (const auto &pair : value)
  {
    ss << "\"" << to_string(pair.first) << "\": \"" << to_string(pair.second) << "\"";
    if (i + 1 < value.size())
      ss << ", ";
    ++i;
  }
  ss << "}";
  return ss.str();
}

template<>
inline std::string to_string(const ros::Time& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::WallTime& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::SteadyTime& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::Duration& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::WallDuration& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

/**
 * printf-like support working with std::string and automatically managing memory.
 * @param format The printf-like format string.
 * @param args Arguments of the format string.
 * @return The formatted string.
 */
inline std::string format(const char* format, va_list args)
{
  constexpr size_t BUF_LEN = 1024u;
  char buf[BUF_LEN];

  const auto len = vsnprintf(buf, BUF_LEN, format, args);

  if (len < BUF_LEN) {
    return std::string(buf);
  } else {
    char* buf2 = new char[len+1];
    vsnprintf(buf2, len+1, format, args);
    const std::string result(buf2);
    delete[] buf2;
    return result;
  }
}

/**
 * printf-like support working with std::string and automatically managing memory.
 * @param format The printf-like format string.
 * @param args Arguments of the format string.
 * @return The formatted string.
 */
inline std::string format(const char* format, ...)
{
  va_list(args);
  va_start(args, format);
  const auto result = ::cras::format(format, args);
  va_end(args);
  return result;
}

/**
 * printf-like support working with std::string and automatically managing memory.
 * @param format The printf-like format string.
 * @param args Arguments of the format string.
 * @return The formatted string.
 */
inline std::string format(std::string format, ...)
{
  va_list(args);
  va_start(args, format);
  const auto result = ::cras::format(format.c_str(), args);
  va_end(args);
  return result;
}

}
