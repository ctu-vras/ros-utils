/**
 * \file
 * \brief Utils for working with strings.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#if __has_include(<charconv>)
#define HAS_FROM_CHARS 1
#include <charconv>
#else
#define HAS_FROM_CHARS 0
#include <cerrno>
#include <cstdlib>
#endif


#include <algorithm>
#include <cctype>
#include <clocale>
#include <cmath>
#include <iconv.h>
#include <limits>
#include <optional>
#include <regex>
#include <string>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

//#include <ros/console.h>
//#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/string_utils/from_chars.h>

namespace cras
{

void warnLeadingSlash(const std::string& s)
{
  // TODO ROS 2 logging
  //  ROS_WARN_STREAM("Found initial slash in " << s);
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

void strip(std::string& s, const char& c)
{
  stripLeading(s, c);
  stripTrailing(s, c);
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

std::string strip(const std::string& s, const char& c)
{
  return stripLeading(stripTrailing(s, c), c);
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

std::string toUpper(const std::string& str)
{
  // TODO rewrite using libicu-dev
  auto result = str;
  std::transform(str.begin(), str.end(), result.begin(), [](unsigned char c) {return std::toupper(c);});
  return result;
}

std::string toLower(const std::string& str)
{
  // TODO rewrite using libicu-dev
  auto result = str;
  std::transform(str.begin(), str.end(), result.begin(), [](unsigned char c) {return std::tolower(c);});
  return result;
}

#if HAS_FROM_CHARS == 0
template <typename T>
inline void strtoint(const char* str, char** str_end, int base, T& result)
{
  auto tmp = std::strtol(str, str_end, base);
  if (tmp < std::numeric_limits<T>::min() || tmp > std::numeric_limits<T>::max())
    errno = ERANGE;
  else
    result = static_cast<T>(tmp);
}

template<>
inline void strtoint(const char* str, char** str_end, int base, uint32_t& result)
{
  const auto tmp = std::strtoul(str, str_end, base);
  if (str[0] == '-' || tmp > std::numeric_limits<uint32_t>::max())
    errno = ERANGE;
  else
    result = static_cast<uint32_t>(tmp);
}

template<>
inline void strtoint(const char* str, char** str_end, int base, int64_t& result)
{
  result = std::strtoll(str, str_end, base);
}

template<>
inline void strtoint(const char* str, char** str_end, int base, uint64_t& result)
{
  result = std::strtoull(str, str_end, base);
  if (str[0] == '-')
    errno = ERANGE;
}
#endif

template <typename T, ::std::enable_if_t<::std::is_integral_v<::std::decay_t<T>>, bool> = true>
inline T parseIntegralNumber(const std::string& string, const uint8_t base)
{
  T result{};

  auto cleanString = string;
  while (!cleanString.empty() && cleanString[0] == ' ')
    cras::stripLeading(cleanString, ' ');
  while (!cleanString.empty() && cleanString[cleanString.length() - 1] == ' ')
    cras::stripTrailing(cleanString, ' ');
  cras::stripLeading(cleanString, '+');
  while (cleanString.length() > 1 && cleanString[0] == '0')
    cras::stripLeading(cleanString, '0');

#if HAS_FROM_CHARS == 1
  auto [ptr, ec] = std::from_chars(cleanString.data(), cleanString.data() + cleanString.size(), result, base);
#else
  char* ptr;
  ::std::errc ec{};
  errno = 0;
  strtoint(cleanString.data(), &ptr, base, result);
  if (errno == ERANGE)
  {
    ec = std::errc::result_out_of_range;
  }
  else if (errno != 0)
  {
    ec = std::errc::invalid_argument;
  }
#endif

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

template <typename T, ::std::enable_if_t<::std::is_integral_v<::std::decay_t<T>>, bool> = true>
inline T parseIntegralNumber(const std::string& string)
{
  auto cleanString = string;
  while (!cleanString.empty() && cleanString[0] == ' ')
    cras::stripLeading(cleanString, ' ');
  while (!cleanString.empty() && cleanString[cleanString.length() - 1] == ' ')
    cras::stripTrailing(cleanString, ' ');
  cras::stripLeading(cleanString, '+');

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

  return parseIntegralNumber<T>(cleanString, base);
}

int8_t parseInt8(const std::string& string)
{
  return cras::parseIntegralNumber<int8_t>(string);
}

int8_t parseInt8(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<int8_t>(string, base);
}

uint8_t parseUInt8(const std::string& string)
{
  return cras::parseIntegralNumber<uint8_t>(string);
}

uint8_t parseUInt8(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<uint8_t>(string, base);
}

int16_t parseInt16(const std::string& string)
{
  return cras::parseIntegralNumber<int16_t>(string);
}

int16_t parseInt16(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<int16_t>(string, base);
}

uint16_t parseUInt16(const std::string& string)
{
  return cras::parseIntegralNumber<uint16_t>(string);
}

uint16_t parseUInt16(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<uint16_t>(string, base);
}

int32_t parseInt32(const std::string& string)
{
  return cras::parseIntegralNumber<int32_t>(string);
}

int32_t parseInt32(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<int32_t>(string, base);
}

uint32_t parseUInt32(const std::string& string)
{
  return cras::parseIntegralNumber<uint32_t>(string);
}

uint32_t parseUInt32(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<uint32_t>(string, base);
}

int64_t parseInt64(const std::string& string)
{
  return cras::parseIntegralNumber<int64_t>(string);
}

int64_t parseInt64(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<int64_t>(string, base);
}

uint64_t parseUInt64(const std::string& string)
{
  return cras::parseIntegralNumber<uint64_t>(string);
}

uint64_t parseUInt64(const std::string& string, const uint8_t base)
{
  return cras::parseIntegralNumber<uint64_t>(string, base);
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

const std::regex NAME_LEGAL_CHARS_P {R"(^[~/]?[A-Za-z][a-zA-Z0-9/]*$)"};

bool isLegalName(const std::string& name)
{
  // empty string is a legal name as it resolves to namespace
  if (name.empty() || name == "/" || name == "~")
    return true;

  if (cras::contains(name, "//"))
    return false;

  return std::regex_match(name, NAME_LEGAL_CHARS_P);
}

const std::regex BASE_NAME_LEGAL_CHARS_P {R"(^[A-Za-z][A-Za-z0-9_]*$)"};

bool isLegalBaseName(const std::string& name)
{
  return std::regex_match(name,  BASE_NAME_LEGAL_CHARS_P);
}

TempLocale::TempLocale(const int category, const char* newLocale) :
  category(category), oldLocale(setlocale(category, nullptr))
{
  setlocale(category, newLocale);
}

TempLocale::~TempLocale()
{
  setlocale(this->category, this->oldLocale);
}

namespace
{
template <class T> inline void hash_combine(size_t &seed, T const &v)
{
  seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct pair_hash
{
  template <class T1, class T2> size_t operator()(const std::pair<T1, T2> &p) const
  {
    size_t seed = 0;
    hash_combine(seed, p.first);
    hash_combine(seed, p.second);
    return seed;
  }
};

thread_local std::unordered_map<std::pair<std::string, std::string>, iconv_t, pair_hash> iconvDescriptors;
}

std::string iconvConvert(const std::string& toEncoding, const std::string& fromEncoding, const std::string& inText,
                         const bool translit, bool ignore,
                         const double initialOutbufSizeScale, const double outbufEnlargeCoef,
                         const std::optional<std::string>& localeName)
{
  if (outbufEnlargeCoef <= 1.0)
    throw std::invalid_argument("outbufEnlargeCoef has to be strictly larger than 1.0");

  auto toEnc = toEncoding;
  if (translit && !cras::contains(toEncoding, "//TRANSLIT"))
    toEnc += "//TRANSLIT";
  if (ignore && !cras::contains(toEncoding, "//IGNORE"))
    toEnc += "//IGNORE";
  ignore = cras::contains(toEnc, "//IGNORE") || (cras::contains(toEnc, "//") && cras::contains(toEnc, ",IGNORE"));

  iconv_t convDesc;
  if (iconvDescriptors.find({fromEncoding, toEnc}) == iconvDescriptors.end())
  {
    errno = 0;
    convDesc = iconv_open(toEnc.c_str(), fromEncoding.c_str());
    if (convDesc == reinterpret_cast<iconv_t>(-1))
      throw std::invalid_argument(cras::format(
        "Could not create conversion descriptor from encoding '%s' to '%s': Error %s",
          fromEncoding.c_str(), toEnc.c_str(), strerror(errno)));
    iconvDescriptors[{fromEncoding, toEnc}] = convDesc;
  }
  else
  {
    convDesc = iconvDescriptors[{fromEncoding, toEnc}];
    iconv(convDesc, nullptr, nullptr, nullptr, nullptr);
  }

  std::vector<char> inbufData(std::begin(inText), std::end(inText));
  size_t inbufUnreadSize = inbufData.size();
  char* inbuf = inbufData.data();

  size_t outbufLen = static_cast<size_t>(inText.size() * initialOutbufSizeScale);
  std::vector<char> outbufData(outbufLen);
  size_t outbufUnusedSize = outbufData.size();
  char* outbuf = outbufData.data();

  // Read the input until there is something to read
  while (inbufUnreadSize > 0)
  {
    // iconv transliteration doesn't work with the default C locale, we need a UTF-8 one
    TempLocale tempLocale(LC_CTYPE, localeName.value_or("en_US.UTF-8").c_str());
    errno = 0;
    if (iconv(convDesc, &inbuf, &inbufUnreadSize, &outbuf, &outbufUnusedSize) != static_cast<size_t>(-1))
    {
      // Clean up the conversion descriptor and flush possible "shift sequences"
      errno = 0;
      iconv(convDesc, nullptr, nullptr, &outbuf, &outbufUnusedSize);
    }
    else
    {
      // The output buffer is too small; increase its size and try the conversion again
      if (errno == E2BIG)
      {
        inbuf = inbufData.data();
        inbufUnreadSize = inbufData.size();

        outbufLen = static_cast<size_t>(std::ceil(outbufLen * outbufEnlargeCoef));  // Enlarge the output buffer size
        outbufData.resize(outbufLen);
        outbuf = outbufData.data();
        outbufUnusedSize = outbufData.size();
      }
      // Invalid byte sequence encountered or cannot transliterate to output
      else
      {
        const auto resErrno = errno;
        // Reset the conversion descriptor as we'll be ignoring some bytes, so all context is lost
        errno = 0;
        iconv(convDesc, nullptr, nullptr, nullptr, nullptr);

        if (!ignore)
          throw std::invalid_argument(cras::format("Could not convert %s from encoding %s to %s. Error %s",
            inText.c_str(), fromEncoding.c_str(), toEncoding.c_str(), strerror(resErrno)));

        // Ignore invalid input byte sequences or sequences we can't transliterate
        if (resErrno == EILSEQ && inbufUnreadSize > 1)
        {
          inbufUnreadSize -= 1;
          inbuf = inbufData.data() + inText.size() - inbufUnreadSize;
        }
        // EINVAL means invalid byte sequence at the end of input, just throw it away
        // inbufUnreadSize == 0 means ignore is True, some chars were ignored, but otherwise, we have success
        else
          break;
      }
    }
  }

  return {outbufData.data(), outbufLen - outbufUnusedSize};
}

std::string transliterateToAscii(const std::string& text)
{
  return iconvConvert("ASCII", "UTF-8", text, true, true);
}

std::string toValidRosName(
  const std::string& text, const bool baseName, const std::optional<std::string>& fallbackName)
{
  if ((baseName && isLegalBaseName(text)) || (!baseName && isLegalName(text)))
    return text;

  if (text.empty())
  {
    if (!fallbackName.has_value())
      throw std::invalid_argument("Empty name is not allowed");
    return *fallbackName;
  }

  auto name = transliterateToAscii(text);
  std::string prefix;
  if (baseName)
  {
    name = std::regex_replace(name, std::regex("[^a-zA-Z0-9_]"), "_");
  }
  else
  {
    if (name[0] == '~')
    {
      prefix = "~";
      name = name.substr(1);
    }
    else if (name[0] == '/')
    {
      prefix = "/";
      name = name.substr(1);
    }
    name = std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]"), "_");
  }

  while (cras::contains(name, "__"))
    cras::replace(name, "__", "_");

  name = std::regex_replace(name, std::regex("^[^a-zA-Z]*"), "");
  if (name.empty())
  {
    if (!fallbackName.has_value())
        throw std::invalid_argument(cras::format("Name '%s' cannot be converted to valid ROS name", name.c_str()));
    return *fallbackName;
  }

  name = prefix + name;
  if ((baseName && !isLegalBaseName(name)) || (!baseName && !isLegalName(name)))
  {
    if (!fallbackName.has_value())
      throw std::invalid_argument(cras::format("Name '%s' cannot be converted to valid ROS name", name.c_str()));
    return *fallbackName;
  }

  return name;
}

};
