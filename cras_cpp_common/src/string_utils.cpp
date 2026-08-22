// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Utils for working with strings.
 * \author Martin Pecka
 */

#include <iconv.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <clocale>
#include <cmath>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// #include <ros/console.h>
// #include <rosconsole/macros_generated.h>

#include <cras_cpp_common/format.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/string_utils/from_chars.hpp>

namespace cras {

void warnLeadingSlash(const std::string& s) {
  // TODO ROS 2 logging
  // ROS_WARN_STREAM("Found initial slash in " << s);
}

void stripLeading(std::string& s, const char& c) {
  if (s.length() > 0 && s[0] == c) {
    s.erase(0, 1);
  }
}

void stripTrailing(std::string& s, const char& c) {
  if (s.length() > 0 && s[s.length() - 1] == c) {
    s.pop_back();
  }
}

void strip(std::string& s, const char& c) {
  stripLeading(s, c);
  stripTrailing(s, c);
}

void stripLeadingSlash(std::string& s, const bool warn) {
  if (s.length() > 0 && s[0] == '/') {
    if (warn) {
      warnLeadingSlash(s);
    }
    s.erase(0, 1);
  }
}

std::string stripLeading(const std::string& s, const char& c) {
  if (s.length() > 0 && s[0] == c) {
    return s.substr(1);
  }

  return s;
}

std::string stripTrailing(const std::string& s, const char& c) {
  if (s.length() > 0 && s[s.length() - 1] == c) {
    return s.substr(0, s.length() - 1);
  }

  return s;
}

std::string strip(const std::string& s, const char& c) {
  return stripLeading(stripTrailing(s, c), c);
}

std::string stripLeadingSlash(const std::string& s, const bool warn) {
  if (s.length() > 0 && s[0] == '/') {
    if (warn) {
      warnLeadingSlash(s);
    }
    return s.substr(1);
  }

  return s;
}

std::string removePrefix(const std::string& str, const std::string& prefix, bool* had_prefix) {
  const auto has_prefix = startsWith(str, prefix);
  if (had_prefix != nullptr) {
    *had_prefix = has_prefix;
  }

  return has_prefix ? str.substr(prefix.length()) : str;
}

std::string removeSuffix(const std::string& str, const std::string& suffix, bool* had_suffix) {
  const auto has_suffix = endsWith(str, suffix);
  if (had_suffix != nullptr) {
    *had_suffix = has_suffix;
  }

  return has_suffix ? str.substr(0, str.length() - suffix.length()) : str;
}

std::string prependIfNonEmpty(const std::string& str, const std::string& prefix) {
  return str.empty() ? str : prefix + str;
}

std::string appendIfNonEmpty(const std::string& str, const std::string& suffix) {
  return str.empty() ? str : str + suffix;
}

bool startsWith(const std::string& str, const std::string& prefix) {
  return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
}

bool endsWith(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void replace(std::string& str, const std::string& from, const std::string& to, const ::cras::ReplacePosition& where) {
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    if (where == cras::ReplacePosition::START && start_pos != 0) {
      break;
    }
    const auto end_pos = start_pos + from.length();
    if (where == cras::ReplacePosition::END && end_pos != str.length()) {
      start_pos += 1;
      continue;
    }
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

std::string replace(
    const std::string& str, const std::string& from, const std::string& to, const ::cras::ReplacePosition& where) {
  std::string s = str;
  cras::replace(s, from, to, where);
  return s;
}

bool contains(const std::string& str, char c) {
  return str.find_first_of(c) != std::string::npos;
}

bool contains(const std::string& str, const std::string& needle) {
  return str.length() >= needle.length() && str.find(needle) != std::string::npos;
}

std::vector<std::string> split(const std::string& str, const std::string& delimiter, const int max_splits) {
  // inspired by https://stackoverflow.com/a/46931770/1076564, CC-BY-SA 4.0
  // renamed some variables, added the maxSplits option
  size_t start{0};
  size_t end;
  size_t delimiter_length{delimiter.length()};
  std::string token;
  std::vector<std::string> result;

  while ((end = str.find(delimiter, start)) != std::string::npos && (max_splits == -1 || result.size() < max_splits)) {
    token = str.substr(start, end - start);
    start = end + delimiter_length;
    result.push_back(token);
  }

  result.push_back(str.substr(start));
  return result;
}

std::string toUpper(const std::string& str) {
  // TODO rewrite using libicu-dev
  auto result = str;
  std::transform(str.begin(), str.end(), result.begin(), [](unsigned char c) {return std::toupper(c);});
  return result;
}

std::string toLower(const std::string& str) {
  // TODO rewrite using libicu-dev
  auto result = str;
  std::transform(str.begin(), str.end(), result.begin(), [](unsigned char c) {return std::tolower(c);});
  return result;
}

template<typename T, ::std::enable_if_t<::std::is_integral_v<::std::decay_t<T>>, bool> = true>
inline T parseIntegralNumber(const std::string& string, const uint8_t base) {
  T result{};

  auto clean_string = string;
  while (!clean_string.empty() && clean_string[0] == ' ') {
    cras::stripLeading(clean_string, ' ');
  }
  while (!clean_string.empty() && clean_string[clean_string.length() - 1] == ' ') {
    cras::stripTrailing(clean_string, ' ');
  }
  cras::stripLeading(clean_string, '+');
  while (clean_string.length() > 1 && clean_string[0] == '0') {
    cras::stripLeading(clean_string, '0');
  }

  auto [ptr, ec] = std::from_chars(clean_string.data(), clean_string.data() + clean_string.size(), result, base);
  if (ec == std::errc()) {
    if (ptr == clean_string.data() + clean_string.size()) {
      return result;
    }
    throw std::invalid_argument("Passed string contains excess characters: '" + string + "'");
  } else if (ec == std::errc::invalid_argument) {
    throw std::invalid_argument("Passed string is not a number: '" + string + "'");
  } else if (ec == std::errc::result_out_of_range) {
    throw std::invalid_argument("Passed string is out of range: '" + string + "'");
  }
  throw std::runtime_error("Unexpected case");
}

template<typename T, ::std::enable_if_t<::std::is_integral_v<::std::decay_t<T>>, bool> = true>
inline T parseIntegralNumber(const std::string& string) {
  auto clean_string = string;
  while (!clean_string.empty() && clean_string[0] == ' ') {
    cras::stripLeading(clean_string, ' ');
  }
  while (!clean_string.empty() && clean_string[clean_string.length() - 1] == ' ') {
    cras::stripTrailing(clean_string, ' ');
  }
  cras::stripLeading(clean_string, '+');

  auto no_sign_string = clean_string;
  cras::stripLeading(no_sign_string, '-');
  auto base = 10;
  if (no_sign_string.length() > 2 && no_sign_string[0] == '0') {
    if (no_sign_string[1] == 'x' || no_sign_string[1] == 'X') {
      base = 16;
      cras::stripLeading(no_sign_string, '0');
      cras::stripLeading(no_sign_string, 'x');
      cras::stripLeading(no_sign_string, 'X');
    } else if (no_sign_string[1] == 'b' || no_sign_string[1] == 'B') {
      base = 2;
      cras::stripLeading(no_sign_string, '0');
      cras::stripLeading(no_sign_string, 'b');
      cras::stripLeading(no_sign_string, 'B');
    } else {
      base = 8;
      cras::stripLeading(no_sign_string, '0');
    }
    clean_string = clean_string[0] == '-' ? ("-" + no_sign_string) : no_sign_string;
  } else if (no_sign_string.length() > 1 && no_sign_string[0] == '0') {
    base = 8;
    cras::stripLeading(no_sign_string, '0');
    clean_string = clean_string[0] == '-' ? ("-" + no_sign_string) : no_sign_string;
  }

  return parseIntegralNumber<T>(clean_string, base);
}

int8_t parseInt8(const std::string& string) {
  return cras::parseIntegralNumber<int8_t>(string);
}

int8_t parseInt8(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<int8_t>(string, base);
}

uint8_t parseUInt8(const std::string& string) {
  return cras::parseIntegralNumber<uint8_t>(string);
}

uint8_t parseUInt8(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<uint8_t>(string, base);
}

int16_t parseInt16(const std::string& string) {
  return cras::parseIntegralNumber<int16_t>(string);
}

int16_t parseInt16(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<int16_t>(string, base);
}

uint16_t parseUInt16(const std::string& string) {
  return cras::parseIntegralNumber<uint16_t>(string);
}

uint16_t parseUInt16(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<uint16_t>(string, base);
}

int32_t parseInt32(const std::string& string) {
  return cras::parseIntegralNumber<int32_t>(string);
}

int32_t parseInt32(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<int32_t>(string, base);
}

uint32_t parseUInt32(const std::string& string) {
  return cras::parseIntegralNumber<uint32_t>(string);
}

uint32_t parseUInt32(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<uint32_t>(string, base);
}

int64_t parseInt64(const std::string& string) {
  return cras::parseIntegralNumber<int64_t>(string);
}

int64_t parseInt64(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<int64_t>(string, base);
}

uint64_t parseUInt64(const std::string& string) {
  return cras::parseIntegralNumber<uint64_t>(string);
}

uint64_t parseUInt64(const std::string& string, const uint8_t base) {
  return cras::parseIntegralNumber<uint64_t>(string, base);
}

template<typename T, ::std::enable_if_t<::std::is_floating_point_v<::std::decay_t<T>>, bool> = true>
inline T parseFloatingNumber(const std::string& string) {
  T result{};

  auto clean_string = cras::stripLeading(string, ' ');
  cras::stripLeading(clean_string, '+');
  cras::stripTrailing(clean_string, ' ');

  auto [ptr, ec] = cras::from_chars(clean_string, result);

  if (ec == std::errc()) {
    if (ptr == clean_string.data() + clean_string.size()) {
      return result;
    }
    throw std::invalid_argument("Passed string contains excess characters: '" + string + "'");
  } else if (ec == std::errc::invalid_argument) {
    throw std::invalid_argument("Passed string is not a number: '" + string + "'");
  } else if (ec == std::errc::result_out_of_range) {
    throw std::invalid_argument("Passed string is out of range: '" + string + "'");
  }
  throw std::runtime_error("Unexpected case");
}

float parseFloat(const std::string& string) {
  return cras::parseFloatingNumber<float>(string);
}

double parseDouble(const std::string& string) {
  return cras::parseFloatingNumber<double>(string);
}

const std::regex NAME_LEGAL_CHARS_P {R"(^[~/]?[A-Za-z][a-zA-Z0-9/]*$)"};

bool isLegalName(const std::string& name) {
  // empty string is a legal name as it resolves to namespace
  if (name.empty() || name == "/" || name == "~") {
    return true;
  }

  if (cras::contains(name, "//")) {
    return false;
  }

  return std::regex_match(name, NAME_LEGAL_CHARS_P);
}

const std::regex BASE_NAME_LEGAL_CHARS_P {R"(^[A-Za-z][A-Za-z0-9_]*$)"};

bool isLegalBaseName(const std::string& name) {
  return std::regex_match(name, BASE_NAME_LEGAL_CHARS_P);
}

TempLocale::TempLocale(const int category, const char* new_locale)
    : category_(category), old_locale_(setlocale(category, nullptr)) {
  setlocale(category, new_locale);
}

TempLocale::~TempLocale() {
  setlocale(category_, old_locale_);
}

namespace {
template<class T> inline void hash_combine(size_t& seed, T const& v) {
  seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct pair_hash {
  template<class T1, class T2> size_t operator()(const std::pair<T1, T2>& p) const {
    size_t seed = 0;
    hash_combine(seed, p.first);
    hash_combine(seed, p.second);
    return seed;
  }
};

thread_local std::unordered_map<std::pair<std::string, std::string>, iconv_t, pair_hash> g_iconv_descriptors;
}

std::string iconvConvert(
    const std::string& to_encoding, const std::string& from_encoding, const std::string& in_text, const bool translit,
    bool ignore, const double initial_outbuf_size_scale, const double outbuf_enlarge_coef,
    const std::optional<std::string>& locale_name) {
  if (outbuf_enlarge_coef <= 1.0) {
    throw std::invalid_argument("outbufEnlargeCoef has to be strictly larger than 1.0");
  }

  auto to_enc = to_encoding;
  if (translit && !cras::contains(to_encoding, "//TRANSLIT")) {
    to_enc += "//TRANSLIT";
  }
  if (ignore && !cras::contains(to_encoding, "//IGNORE")) {
    to_enc += "//IGNORE";
  }
  ignore = cras::contains(to_enc, "//IGNORE") || (cras::contains(to_enc, "//") && cras::contains(to_enc, ",IGNORE"));

  iconv_t conv_desc;
  if (!g_iconv_descriptors.contains({from_encoding, to_enc})) {
    errno = 0;
    conv_desc = iconv_open(to_enc.c_str(), from_encoding.c_str());
    if (conv_desc == reinterpret_cast<iconv_t>(-1)) {
      throw std::invalid_argument(cras::format(
        "Could not create conversion descriptor from encoding '{}' to '{}': Error {}",
        from_encoding, to_enc, strerror(errno)));
    }
    g_iconv_descriptors[{from_encoding, to_enc}] = conv_desc;
  } else {
    conv_desc = g_iconv_descriptors[{from_encoding, to_enc}];
    iconv(conv_desc, nullptr, nullptr, nullptr, nullptr);
  }

  std::vector<char> inbuf_data(std::begin(in_text), std::end(in_text));
  size_t inbuf_unread_size = inbuf_data.size();
  char* inbuf = inbuf_data.data();

  size_t outbuf_len = static_cast<size_t>(in_text.size() * initial_outbuf_size_scale);
  std::vector<char> outbuf_data(outbuf_len);
  size_t outbuf_unused_size = outbuf_data.size();
  char* outbuf = outbuf_data.data();

  // Read the input until there is something to read
  while (inbuf_unread_size > 0) {
    // iconv transliteration doesn't work with the default C locale, we need a UTF-8 one
    TempLocale temp_locale(LC_CTYPE, locale_name.value_or("en_US.UTF-8").c_str());
    errno = 0;
    if (iconv(conv_desc, &inbuf, &inbuf_unread_size, &outbuf, &outbuf_unused_size) != static_cast<size_t>(-1)) {
      // Clean up the conversion descriptor and flush possible "shift sequences"
      errno = 0;
      iconv(conv_desc, nullptr, nullptr, &outbuf, &outbuf_unused_size);
    } else {
      // The output buffer is too small; increase its size and try the conversion again
      if (errno == E2BIG) {
        inbuf = inbuf_data.data();
        inbuf_unread_size = inbuf_data.size();

        // Enlarge the output buffer size
        outbuf_len = static_cast<size_t>(std::ceil(outbuf_len * outbuf_enlarge_coef));
        outbuf_data.resize(outbuf_len);
        outbuf = outbuf_data.data();
        outbuf_unused_size = outbuf_data.size();
      } else {
        // Invalid byte sequence encountered or cannot transliterate to output
        const auto res_errno = errno;
        // Reset the conversion descriptor as we'll be ignoring some bytes, so all context is lost
        errno = 0;
        iconv(conv_desc, nullptr, nullptr, nullptr, nullptr);

        if (!ignore) {
          throw std::invalid_argument(cras::format("Could not convert {} from encoding {} to {}. Error {}",
                                                   in_text, from_encoding, to_encoding, strerror(res_errno)));
        }

        // Ignore invalid input byte sequences or sequences we can't transliterate
        if (res_errno == EILSEQ && inbuf_unread_size > 1) {
          inbuf_unread_size -= 1;
          inbuf = inbuf_data.data() + in_text.size() - inbuf_unread_size;
        } else {
          // EINVAL means invalid byte sequence at the end of input, just throw it away
          // inbufUnreadSize == 0 means ignore is True, some chars were ignored, but otherwise, we have success
          break;
        }
      }
    }
  }

  return {outbuf_data.data(), outbuf_len - outbuf_unused_size};
}

std::string transliterateToAscii(const std::string& text) {
  return iconvConvert("ASCII", "UTF-8", text, true, true);
}

std::string toValidRosName(
    const std::string& text, const bool base_name, const std::optional<std::string>& fallback_name) {
  if ((base_name && isLegalBaseName(text)) || (!base_name && isLegalName(text))) {
    return text;
  }

  if (text.empty()) {
    if (!fallback_name.has_value()) {
      throw std::invalid_argument("Empty name is not allowed");
    }
    return *fallback_name;
  }

  auto name = transliterateToAscii(text);
  std::string prefix;
  if (base_name) {
    name = std::regex_replace(name, std::regex("[^a-zA-Z0-9_]"), "_");
  } else {
    if (name[0] == '~') {
      prefix = "~";
      name = name.substr(1);
    } else if (name[0] == '/') {
      prefix = "/";
      name = name.substr(1);
    }
    name = std::regex_replace(name, std::regex("[^a-zA-Z0-9_/]"), "_");
  }

  while (cras::contains(name, "__")) {
    cras::replace(name, "__", "_");
  }

  name = std::regex_replace(name, std::regex("^[^a-zA-Z]*"), "");
  if (name.empty()) {
    if (!fallback_name.has_value()) {
      throw std::invalid_argument(cras::format("Name '{}' cannot be converted to valid ROS name", name));
    }
    return *fallback_name;
  }

  name = prefix + name;
  if ((base_name && !isLegalBaseName(name)) || (!base_name && !isLegalName(name))) {
    if (!fallback_name.has_value()) {
      throw std::invalid_argument(cras::format("Name '{}' cannot be converted to valid ROS name", name));
    }
    return *fallback_name;
  }

  return name;
}

}  // namespace cras
