/**
 * \file
 * \brief Utilities for working with C++ types.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */
 
#include <cras_cpp_common/type_utils.hpp>

#if __has_include(<cxxabi.h>)
#define HAS_CXX_ABI 1
#include <cxxabi.h>
#endif

#include <cras_cpp_common/string_utils.hpp>

namespace cras
{

std::string cleanTypeName(const std::string& typeName)
{
  auto result = typeName;
  ::cras::replace(result, "::__cxx11", "");
  ::cras::replace(result, "basic_string", "string");
  ::cras::replace(result, "string<char>", "string");
  ::cras::replace(result, " >", ">");
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