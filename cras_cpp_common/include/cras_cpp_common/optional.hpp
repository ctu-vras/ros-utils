#pragma once

/**
 * \file
 * \brief A C++14 shim for std::optional. It doesn't work as a fully-fledged substitute and a compiler warning is
 *        printed when the C++14 version is used.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#if __has_include(<optional>) && __cplusplus >= 201703L
#include <optional>
namespace cras
{
  using ::std::optional;
  using ::std::bad_optional_access;
  using ::std::nullopt;
}
#else

#warning Using C++14 fallback implementation of std::optional. This may result in various problems. Consider building \
this package with C++17.

#include <memory>

namespace cras
{
class bad_optional_access : public std::exception {
public:
  bad_optional_access() = default;
  const char *what() const noexcept { return "Optional has no value"; }
};

struct nullopt_t {
  struct do_not_use {};
  constexpr explicit nullopt_t(do_not_use, do_not_use) noexcept {}
};

static constexpr nullopt_t nullopt {
  nullopt_t::do_not_use{}, nullopt_t::do_not_use{}
};

template <class T>
class optional
{
public:
  optional() noexcept = default;
  optional(nullopt_t) noexcept {}

  optional(const T& v)
  {
    this->stored_value = std::make_unique<T>(v);
  }
  
  virtual ~optional()
  {
  }

  inline optional(const optional &rhs) noexcept
  {
    this->operator=(rhs);
  };

  inline optional(optional&& rhs) = default;

  template <class U>
  inline explicit optional(const optional<U>& rhs) noexcept : optional(rhs.value()) {}
  
  inline bool has_value() const noexcept { return this->stored_value != nullptr; }
  
  inline explicit operator bool() const noexcept
  {
    return this->has_value();
  }
  
  T& value() & {
    if (this->has_value())
      return *this->stored_value;
    throw bad_optional_access();
  }
  
  const T& value() const & {
    if (this->has_value())
      return *this->stored_value;
    throw bad_optional_access();
  }
  
  void reset() noexcept {
    this->stored_value.reset();
  }
  
  inline optional& operator=(nullopt_t) noexcept {
    this->stored_value.reset();
    return *this;
  }

  inline optional& operator=(const optional& rhs)
  {
    if (this->has_value())
      this->stored_value.reset();
    if (rhs.has_value())
      this->stored_value = std::make_unique<T>(rhs.value());
    return *this;
  }

  template <class U>
  inline optional& operator=(const optional<U>& rhs)
  {
    if (this->has_value())
      this->stored_value.reset();
    if (rhs.has_value())
      this->stored_value = std::make_unique<T>(rhs.value());
    return *this;
  }

  template <class U>
  inline optional& operator=(const U& rhs)
  {
    this->stored_value = std::make_unique<T>(rhs);
    return *this;
  }
  
private:
  std::unique_ptr<T> stored_value;
};

template <class T, class U>
inline bool operator==(const optional<T>& lhs, const optional<U>& rhs) {
  return lhs.has_value() == rhs.has_value() && (!lhs.has_value() || lhs.value() == rhs.value());
}

template <class T, class U>
inline bool operator!=(const optional<T>& lhs, const optional<U>& rhs) {
  return lhs.has_value() != rhs.has_value() || (lhs.has_value() && lhs.value() != rhs.value());
}

template <class T>
inline bool operator==(const optional<T>& lhs, nullopt_t) noexcept {
  return !lhs.has_value();
}

template <class T>
inline bool operator==(nullopt_t, const optional<T>& rhs) noexcept {
  return !rhs.has_value();
}

template <class T>
inline bool operator!=(const optional<T>& lhs, nullopt_t) noexcept {
  return lhs.has_value();
}

template <class T>
inline bool operator!=(nullopt_t, const optional<T>& rhs) noexcept {
  return rhs.has_value();
}

}

#endif

namespace cras
{
/**
 * \brief Type trait determining whether type T is cras::optional or not.
 * \tparam T The type to test.
 */
template<typename T>
struct is_optional : public ::std::false_type {};

/**
 * \brief Type trait determining whether type T is cras::optional or not.
 * \tparam T The type to test.
 */
template<typename T>
struct is_optional<::cras::optional<T>> : public ::std::true_type {};
}