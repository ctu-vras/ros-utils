/**
 * \file
 * \brief Utilities for working with C++ types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <regex>
#include <string>

#if __has_include(<cxxabi.h>)
#define HAS_CXX_ABI 1
#include <cxxabi.h>
#endif

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{

std::regex charTraitsRegex("(.*), std::char_traits<\\1>");  // NOLINT
std::regex allocatorRegex("(.*), std::allocator<\\1>");  // NOLINT
std::regex mapRegex("<(.*), (.*), std::less<\\1>, std::allocator<std::pair<\\1( const)?, \\2>>");  // NOLINT
std::regex unorderedMapRegex(  // NOLINT
	"<(.*), (.*), std::hash<\\1>, std::equal_to<\\1>, std::allocator<std::pair<\\1( const)?, \\2>>");  // NOLINT
std::regex setRegex("<(.*), std::less<\\1>");  // NOLINT
std::regex unorderedSetRegex("<(.*), std::hash<\\1>, std::equal_to<\\1>");  // NOLINT

std::string cleanTypeName(const std::string& typeName)
{
  auto result = typeName;

	cras::replace(result, " >", ">");
	cras::replace(result, "::__cxx11", "");

	result = std::regex_replace(result, mapRegex, "<$1, $2");
	result = std::regex_replace(result, unorderedMapRegex, "<$1, $2");
	result = std::regex_replace(result, setRegex, "<$1");
	result = std::regex_replace(result, unorderedSetRegex, "<$1");

	while (std::regex_search(result, charTraitsRegex) || std::regex_search(result, allocatorRegex))
	{
		result = std::regex_replace(result, charTraitsRegex, "$1");
		result = std::regex_replace(result, allocatorRegex, "$1");
	}

	cras::replace(result, "basic_string", "string");
	cras::replace(result, "string<char>", "string");

  return result;
}

std::string demangle(const std::string& mangled)
{
#if HAS_CXX_ABI
  int status;
  const auto demangled = abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &status);
  if (demangled && status == 0)
  {
    std::string result {demangled};
    std::free(demangled);
    return result;
  }
  else
  {
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

std::string getTypeName(const std::type_info& typeInfo)
{
  return cras::cleanTypeName(cras::demangle(typeInfo.name()));
}

}