#pragma once

#include <cstddef>
#include <type_traits>
#include <cras_cpp_common/string_utils.hpp>

#if __has_include(<cxxabi.h>)
#define HAS_CXX_ABI 1
#include <cxxabi.h>
#endif

/**
 * This operator allows you to write size_t literals like 5_sz.
 */
inline size_t operator "" _sz(unsigned long long int x)
{
  return x;
}

namespace cras
{

/**
 * \brief Remove not-so-nice parts of demangled C++ type names.
 * \param[in] typeName Name of a demangled C++ type.
 * \return A better name.
 */
inline ::std::string cleanTypeName(const ::std::string& typeName)
{
  auto result = typeName;
  ::cras::replace(result, "::__cxx11", "");
  ::cras::replace(result, "basic_string", "string");
  ::cras::replace(result, "string<char>", "string");
  ::cras::replace(result, " >", ">");
  return result;
}

/**
 * \brief Demangle the given mangle C++ type identifier.
 * \param[in] mangled The mangled name.
 * \return The demangled name if demangling succeeded. Otherwise, return `mangled`.
 */
inline ::std::string demangle(const ::std::string& mangled)
{
#if HAS_CXX_ABI
  int status;
  const auto demangled = ::abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &status);
  if (demangled && status == 0)
  {
    ::std::string result {demangled};
    ::std::free(demangled);
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

/**
 * \brief Get a human-readable name of T.
 * \tparam T Type to get info about.
 * \return Human-readable name.
 */
template <typename T>
inline ::std::string getTypeName()
{
#if defined(__clang__)
  const ::std::string prefix = "[T = ";
  const ::std::string suffix = "]";
  const ::std::string function = __PRETTY_FUNCTION__;
#elif defined(__GNUC__)
  const ::std::string prefix = "with T = ";
  const ::std::string suffix = "; ";
  const ::std::string function = __PRETTY_FUNCTION__;
#elif defined(__MSC_VER)
  const ::std::string prefix = "get_type_name<";
  const ::std::string suffix = ">(void)";
  const ::std::string function = __FUNCSIG__;
#else
  const ::std::string prefix = "";
  const ::std::string suffix = "";
  const ::std::string function = ::cras::demangle(typeid(T).name());
#endif

  const auto start = function.find(prefix) + prefix.size();
  const auto end = function.find(suffix);
  const auto size = end - start;

  return ::cras::cleanTypeName(function.substr(start, size));
}

/**
 * \brief Get a human-readable name of a type represented by the given typeinfo.
 * \param[in] typeInfo Info about the type.
 * \return Human-readable name.
 */
inline ::std::string getTypeName(const ::std::type_info& typeInfo)
{
  return ::cras::cleanTypeName(::cras::demangle(typeInfo.name()));
}

template<typename T>
struct is_c_string : public std::false_type {};

template<>
struct is_c_string<char*> : public std::true_type {};

template<>
struct is_c_string<char* const> : public std::true_type {};

template<>
struct is_c_string<const char*> : public std::true_type {};

template<>
struct is_c_string<const char* const> : public std::true_type {};

template<int I>
struct is_c_string<char[I]> : public std::true_type {};

template<int I>
struct is_c_string<const char[I]> : public std::true_type {};

}