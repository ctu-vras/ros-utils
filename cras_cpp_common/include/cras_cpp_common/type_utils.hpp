#pragma once

#include <cstddef>
#include <string>
#include <type_traits>

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
::std::string cleanTypeName(const ::std::string& typeName);

/**
 * \brief Demangle the given mangle C++ type identifier.
 * \param[in] mangled The mangled name.
 * \return The demangled name if demangling succeeded. Otherwise, return `mangled`.
 */
::std::string demangle(const ::std::string& mangled);

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
::std::string getTypeName(const ::std::type_info& typeInfo);

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

template<typename T, typename = void>
struct is_string : public std::false_type {};

template<typename T>
struct is_string<T, ::std::enable_if_t<::cras::is_c_string<T>::value>> : public std::true_type {};

template<typename T>
struct is_string<T, ::std::enable_if_t<::std::is_same<T, ::std::string>::value>> : public std::true_type {};

}