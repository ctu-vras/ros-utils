#pragma once

/**
 * \file
 * \brief Utils for working with strings.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <cstdarg>
#include <functional>
#include <list>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cras_cpp_common/type_utils/string_traits.hpp>

namespace cras
{

/**
 * \brief Strip `c` from the start of the given string (if there is one).
 * \param[in,out] s The string from which c should be removed.
 * \param[in] c The character to remove.
 */
void stripLeading(::std::string& s, const char& c);

/**
 * \brief Strip `c` from the end of the given string (if there is one).
 * \param[in,out] s The string from which c should be removed.
 * \param[in] c The character to remove.
 */
void stripTrailing(::std::string& s, const char& c);

/**
 * \brief Strip leading slash from the given string (if there is one).
 * \param[in,out] s The string from which slash should be removed.
 * \param[in] warn If true, issue a ROS warning if the string contained the leading slash.
 */
void stripLeadingSlash(::std::string& s, bool warn = false);

/**
 * \brief Return a copy of the given string with c removed from its start (if there is one).
 * \param[in] s The string from which c should be removed.
 * \param[in] c The character to remove.
 * \return The string without c at the start.
 */
::std::string stripLeading(const ::std::string& s, const char& c);

/**
 * \brief Return a copy of the given string with c removed from its end (if there is one).
 * \param[in] s The string from which c should be removed.
 * \param[in] c The character to remove.
 * \return The string without c at the end.
 */
::std::string stripTrailing(const ::std::string& s, const char& c);

/**
 * \brief Return a copy of the given string with leading slash removed (if there is one).
 * \param[in] s The string from which slash should be removed.
 * \param[in] warn If true, issue a ROS warning if the string contained the leading slash.
 * \return The string without leading slash.
 */
::std::string stripLeadingSlash(const ::std::string& s, bool warn = false);

/**
 * \brief Remove `prefix` from start of `str` if it contains it, otherwise return `str` unchanged.
 * \param[in] str The string to work on.
 * \param[in] prefix The prefix to find.
 * \param[in] hadPrefix If non-null, will contain information whether `str` starts with `prefix`.
 * \return If `str` starts with `prefix`, it will return `str` with `prefix` removed.
 *         Otherwise, `str` will be returned unchanged.
 */
::std::string removePrefix(const ::std::string& str, const ::std::string& prefix, bool* hadPrefix = nullptr);

/**
 * \brief Remove `suffix` from end of `str` if it contains it, otherwise return `str` unchanged.
 * \param[in] str The string to work on.
 * \param[in] suffix The suffix to find.
 * \param[in] hadSuffix If non-null, will contain information whether `str` ends with `suffix`.
 * \return If `str` ends with `suffix`, it will return `str` with `suffix` removed.
 *         Otherwise, `str` will be returned unchanged.
 */
::std::string removeSuffix(const ::std::string& str, const ::std::string& suffix, bool* hadSuffix = nullptr);

/**
 * \brief If `str` is nonempty, returns prefix + str, otherwise empty string.
 * \param[in] str The main string.
 * \param[in] prefix The string's prefix.
 * \return The possibly prefixed string.
 */
::std::string prependIfNonEmpty(const ::std::string& str, const ::std::string& prefix);

/**
 * \brief If `str` is nonempty, returns str + suffix, otherwise empty string.
 * \param[in] str The main string.
 * \param[in] suffix The string's suffix.
 * \return The possibly suffixed string.
 */
::std::string appendIfNonEmpty(const ::std::string& str, const ::std::string& suffix);

/**
 * \brief Check whether `prefix` is a prefix of `str`.
 * \param[in] str The string to be searched in.
 * \param[in] prefix The string to be found in `str`.
 * \return Whether `prefix` is a prefix of `str`.
 */
bool startsWith(const ::std::string& str, const ::std::string& prefix);

/**
 * \brief Check whether `suffix` is a suffix of `str`.
 * \param[in] str The string to be searched in.
 * \param[in] suffix The string to be found in `str`.
 * \return Whether `suffix` is a suffix of `str`.
 */
bool endsWith(const ::std::string& str, const ::std::string& suffix);

/**
 * \brief Replace all occurrences of `from` in `str` with `to`.
 * \param[in] str The string to replace in.
 * \param[in] from The string to replace.
 * \param[in] to The replacement.
 * \return `str` with all occurrences of `from` replaced with `to`.
 */
::std::string replace(const ::std::string& str, const ::std::string& from, const ::std::string& to);

/**
 * \brief Replace all occurrences of `from` in `str` with `to`.
 * \param[in,out] str The string to replace in.
 * \param[in] from The string to replace.
 * \param[in] to The replacement.
 */
void replace(::std::string& str, const ::std::string& from, const ::std::string& to);

/**
 * \brief Check whether `str` contains character `c`.
 * \param[in] str The string to search in. 
 * \param[in] c The character to search.
 * \return Whether `str` contains character `c`.
 */
bool contains(const ::std::string& str, char c);

/**
 * \brief Check whether `str` contains substring `needle`.
 * \param[in] str The string to search in. 
 * \param[in] needle The substring to search.
 * \return Whether `str` contains substring `needle`.
 */
bool contains(const ::std::string& str, const ::std::string& needle);

/**
 * \brief Split the given string by the given delimiter.
 * \param[in] str The string to split.
 * \param[in] delimiter The delimiter used for splitting.
 * \param[in] maxSplits If >= 0, defines the maximum number of splits.
 * \return A vector of parts of the original string.
 */
::std::vector<::std::string> split(const ::std::string& str, const ::std::string& delimiter, int maxSplits = -1);

/**
 * printf-like support working with std::string and automatically managing memory.
 * \param[in] format The printf-like format string.
 * \param[in] args Arguments of the format string.
 * \return The formatted string.
 */
inline ::std::string format(const char* format, ::va_list args)
{
  constexpr size_t BUF_LEN = 1024u;
  char buf[BUF_LEN];

  ::va_list argsCopy;
  ::va_copy(argsCopy, args);

  const auto len = ::vsnprintf(buf, BUF_LEN, format, args);

  ::std::string result;
  if (len < BUF_LEN)
  {
    result = buf;
  }
  else
  {
    char* buf2 = new char[len + 1];
    ::vsnprintf(buf2, len + 1, format, argsCopy);
    result = buf2;
    delete[] buf2;
  }
  ::va_end(argsCopy);
  return result;
}

/**
 * printf-like support working with std::string and automatically managing memory.
 * \param[in] format The printf-like format string.
 * \param[in] args Arguments of the format string.
 * \return The formatted string.
 */
inline ::std::string format(const char* format, ...)
{
  ::va_list(args);
  ::va_start(args, format);
  const auto result = ::cras::format(format, args);
  ::va_end(args);
  return result;
}

/**
 * printf-like support working with std::string and automatically managing memory.
 * \param[in] format The printf-like format string.
 * \param[in] args Arguments of the format string.
 * \return The formatted string.
 */
inline ::std::string format(::std::string format, ...)
{
  ::va_list(args);
  ::va_start(args, format);
  const auto result = ::cras::format(format.c_str(), args);
  ::va_end(args);
  return result;
}

/**
 * \brief Put `s` in double quotes if `T` is a string type (std::string or char*).
 * \tparam T The type to check.
 * \param[in] s The input string. 
 * \return Either `s` in double quotes if `T` is a string type, or just `s`.
 */
template<typename T, ::std::enable_if_t<!::cras::is_string<::std::decay_t<T>>::value, bool> = true>
inline ::std::string quoteIfStringType(const ::std::string& s, const T&)
{
  return s;
}

/**
 * \brief Put `s` in double quotes if `T` is a string type (std::string or char*).
 * \tparam T The type to check.
 * \param[in] s The input string. 
 * \return Either `s` in double quotes if `T` is a string type, or just `s`.
 */
template<typename T, ::std::enable_if_t<::cras::is_string<::std::decay_t<T>>::value, bool> = true>
inline ::std::string quoteIfStringType(const ::std::string& s, const T&)
{
  return "\"" + s + "\"";
}

/**
 * \brief Convert the given value to a string representation.
 * \tparam T Type of the value.
 * \param[in] value The value to convert.
 * \return The string representation.
 */
template<typename T>
inline decltype(::std::to_string(::std::declval<T>())) to_string(const T& value)
{
  return ::std::to_string(value);
}

/** \brief Type of function that converts anything to a string. */
template<typename T> using ToStringFn = ::std::function<::std::string(const T&)>;

inline ::std::string to_string(const double& value)
{
  return ::cras::format("%g", value);
}

inline ::std::string to_string(const float& value)
{
  return ::cras::format("%g", value);
}

inline ::std::string to_string(const long double& value)
{
  return ::cras::format("%gL", value);
}

inline ::std::string to_string(const char* value)
{
  return {value};
}

inline ::std::string to_string(char* value)
{
  return {value};
}

template<int I>
inline ::std::string to_string(const char value[I])
{
  return {value};
}

template<int I>
inline ::std::string to_string(char value[I])
{
  return {value};
}

inline ::std::string to_string(const bool& value)
{
  return value ? "True" : "False";
}

inline ::std::string to_string(const ::std::string& value)
{
  return value;
}

}

#if __has_include(<Eigen/Core>)
#include "cras_cpp_common/string_utils/eigen.hpp"
#endif

#if __has_include(<tf2/LinearMath/Vector3.h>)
#include "cras_cpp_common/string_utils/tf2.hpp"
#endif

#if __has_include(<ros/ros.h>)
#include "cras_cpp_common/string_utils/ros.hpp"
#endif

#if __has_include(<xmlrpcpp/XmlRpcValue.h>)
#include "cras_cpp_common/string_utils/xmlrpc.hpp"
#endif

namespace cras
{

// forward declarations of to_string(map) so that to_string(vector) can make use of it
template<typename K, typename V>
inline ::std::string to_string(const ::std::map<K, V>& value);

template<typename K, typename V>
inline ::std::string to_string(const ::std::unordered_map<K, V>& value);

#define DECLARE_TO_STRING_VECTOR(vectorType, prefix, suffix) \
  template<typename T> \
  inline ::std::string to_string(const vectorType<T>& value) \
  { \
    ::std::stringstream ss; \
    ss << (prefix); \
    size_t i = 0; \
    for (const auto& v: value) \
    { \
      ss << ::cras::quoteIfStringType(::cras::to_string(v), v); \
      if (i + 1 < value.size()) \
        ss << ", "; \
      ++i; \
    } \
    ss << (suffix); \
    return ss.str(); \
  }

DECLARE_TO_STRING_VECTOR(::std::vector, "[", "]")
DECLARE_TO_STRING_VECTOR(::std::list, "[", "]")
DECLARE_TO_STRING_VECTOR(::std::set, "{", "}")
DECLARE_TO_STRING_VECTOR(::std::unordered_set, "{", "}")

#define DECLARE_TO_STRING_MAP(mapType) \
  template<typename K, typename V> \
  inline ::std::string to_string(const mapType<K, V>& value) \
  { \
    ::std::stringstream ss; \
    ss << "{"; \
    size_t i = 0; \
    for (const auto& pair: value) \
    { \
      ss << ::cras::quoteIfStringType(::cras::to_string(pair.first), pair.first) \
         << ": " \
         << ::cras::quoteIfStringType(::cras::to_string(pair.second), pair.second); \
      if (i + 1 < value.size()) \
        ss << ", "; \
      ++i; \
    } \
    ss << "}"; \
    return ss.str(); \
  }

DECLARE_TO_STRING_MAP(::std::map)
DECLARE_TO_STRING_MAP(::std::unordered_map)

/**
 * \brief Return a string that is a concatenation of elements of `strings` delimited by `delimiter`.
 * \tparam T An iterable type (must support `size()` and foreach).
 * \param[in] strings The elements to put into a string.
 * \param[in] delimiter Delimiter put between elements.
 * \return The concatenated string.
 */
template<typename T>
::std::string join(const T& strings, const ::std::string& delimiter)
{
  const auto numStrings = strings.size();
  if (numStrings == 0)
    return "";

  ::std::stringstream ss;
  size_t i = 0;
  for (const auto& s : strings)
  {
    ss << ::cras::to_string(s);
    if (i < numStrings - 1)
      ss << delimiter;
    i++;
  }
  return ss.str();
}

}
