/**
 * \file
 * \brief Utilities for working with C++ types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <regex>
#include <string>
#include <typeinfo>

#if __has_include(<cxxabi.h>)
#define HAS_CXX_ABI 1
#include <cxxabi.h>
#endif

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>

namespace cras {

std::regex g_char_traits_regex("(.*), std::char_traits<\\1>");  // NOLINT
std::regex g_allocator_regex("(.*), std::allocator<\\1>");  // NOLINT
std::regex g_map_regex("<(.*), (.*), std::less<\\1>, std::allocator<std::pair<\\1( const)?, \\2>>");  // NOLINT
std::regex g_unordered_map_regex(  // NOLINT
    "<(.*), (.*), std::hash<\\1>, std::equal_to<\\1>, std::allocator<std::pair<\\1( const)?, \\2>>");  // NOLINT
std::regex g_set_regex("<(.*), std::less<\\1>");  // NOLINT
std::regex g_unordered_set_regex("<(.*), std::hash<\\1>, std::equal_to<\\1>");  // NOLINT

std::string cleanTypeName(const std::string& type_name) {
  auto result = type_name;

  cras::replace(result, " >", ">");
  cras::replace(result, "::__cxx11", "");

  result = std::regex_replace(result, g_map_regex, "<$1, $2");
  result = std::regex_replace(result, g_unordered_map_regex, "<$1, $2");
  result = std::regex_replace(result, g_set_regex, "<$1");
  result = std::regex_replace(result, g_unordered_set_regex, "<$1");

  while (std::regex_search(result, g_char_traits_regex) || std::regex_search(result, g_allocator_regex)) {
    result = std::regex_replace(result, g_char_traits_regex, "$1");
    result = std::regex_replace(result, g_allocator_regex, "$1");
  }

  cras::replace(result, "basic_string", "string");
  cras::replace(result, "string<char>", "string");

  return result;
}

std::string demangle(const std::string& mangled) {
#if HAS_CXX_ABI
  int status;
  const auto demangled = abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &status);
  if (demangled && status == 0) {
    std::string result {demangled};
    std::free(demangled);
    return result;
  } else {
    return mangled;
  }
#else
  #if defined(__clang__)
  #warning Install package libc++abi-dev to enable name demangling.
#else
  #warning Demangling is not supported for this compiler.
#endif
  return mangled;
#endif
}

std::string getTypeName(const std::type_info& type_info) {
  return cras::cleanTypeName(cras::demangle(type_info.name()));
}

}  // namespace cras
