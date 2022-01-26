#pragma once

#include <cstddef>
#include <cras_cpp_common/string_utils.hpp>

/**
 * This operator allows you to write size_t literals like 5_sz.
 */
inline size_t operator "" _sz(unsigned long long int x)
{
  return x;
}

namespace cras
{

template <typename T>
inline std::string getTypeName()
{
#if defined(__clang__)
  const std::string prefix = "[T = ";
  const std::string suffix = "]";
  const std::string function = __PRETTY_FUNCTION__;
#elif defined(__GNUC__)
  const std::string prefix = "with T = ";
  const std::string suffix = "; ";
  const std::string function = __PRETTY_FUNCTION__;
#elif defined(__MSC_VER)
  const std::string prefix = "get_type_name<";
  const std::string suffix = ">(void)";
  const std::string function = __FUNCSIG__;
#else
  const std::string prefix = "";
  const std::string suffix = "";
  const std::string function = typeid(T).name();
#endif

  const auto start = function.find(prefix) + prefix.size();
  const auto end = function.find(suffix);
  const auto size = end - start;

  auto result {function.substr(start, size)};
  ::cras::replace(result, "::__cxx11", "");
  ::cras::replace(result, "basic_string", "string");
  ::cras::replace(result, "string<char>", "string");
  ::cras::replace(result, " >", ">");
  return result;
}

}