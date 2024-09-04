#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Simple map implemented on top of a std::list<std::pair>. The map is append-only, with lock-free reads and
 *  mutex-protected insert.
 * \author Martin Pecka
 */

#include <algorithm>
#include <list>
#include <mutex>
#include <utility>

namespace cras
{

/**
 * \brief Simple map implemented on top of a std::list<std::pair>. The map is append-only, with lock-free reads and
 *        mutex-protected insert.
 *
 * This map is suitable for storing a small number of elements, as it uses non-sorted linear lookup of the values.
 * However, if this condition is met, the map is reasonably fast and efficient.
 *
 * \tparam K Type of the map keys.
 * \tparam V Type of the map values (should support empty constructor).
 */
template<typename K, typename V>
class SmallMap
{
public:
  /**
   * \brief Find (or insert a default-constructed value) the value for the given key.
   * \param key The key to find.
   * \return Reference to the value stored for the given key.
   */
  V& operator[](const K& key)
  {
    return this->insertIfNew(key);
  }

  /**
   * \brief Get the stored value for the given key.
   * \param key The key to find.
   * \return Const reference to the value stored for the given key.
   * \throws std::out_of_range if the key is not stored in this map..
   */
  const V& at(const K& key) const
  {
    const auto searchFn = [&key](const auto& item) {return item.first == key;};
    auto it = ::std::find_if(this->data.begin(), this->data.end(), searchFn);
    if (__builtin_expect(it == this->data.end(), 0))
      throw ::std::out_of_range("Key not found");
    return it->second;
  }

  /**
   * \brief Check whether the given key is stored in this map.
   * \param key The key to find.
   * \return Whether the key is stored in this map.
   */
  bool contains(const K& key) const
  {
    const auto searchFn = [&key](const auto& item) {return item.first == key;};
    auto it = ::std::find_if(this->data.begin(), this->data.end(), searchFn);
    return it != this->data.end();
  }

  /**
   * \brief Search this map for the key. If it is not found, store the key with a value constructed from \ref args.
   * \param key The key to find.
   * \param args Arguments for the constructor of the inserted value.
   * \return Reference to the value stored for the given key.
   */
  template<typename... Args>
  V& insertIfNew(const K& key, Args&&... args)
  {
    const auto searchFn = [&key](const auto& item) {return item.first == key;};
    auto it = ::std::find_if(this->data.begin(), this->data.end(), searchFn);
    if (__builtin_expect(it == this->data.end(), 0))
    {
      ::std::unique_lock<::std::mutex> lock(this->mutex);
      // Check once again with the lock; if key is still not there, insert it, otherwise find it
      it = ::std::find_if(this->data.begin(), this->data.end(), searchFn);
      if (__builtin_expect(it == this->data.end(), 1))
        // Do not use emplace_back - its reference-returning variant is C++17 only
        it = this->data.emplace(this->data.end(), key, V{::std::forward<Args>(args)...});
    }
    return it->second;
  }

  /**
   * \brief Return the number of elements of this map.
   * \return The number of elements.
   */
  size_t size() const
  {
    return this->data.size();
  }

  /**
   * \brief Return whether this map is empty.
   * \return Whether the map is empty.
   */
  bool empty() const
  {
    return this->data.empty();
  }

private:
  ::std::list<::std::pair<K, V>> data;
  ::std::mutex mutex;
};


/**
 * \brief Simple set implemented on top of a std::list. The set is append-only, with lock-free reads and
 *        mutex-protected insert.
 *
 * This set is suitable for storing a small number of elements, as it uses non-sorted linear lookup of the values.
 * However, if this condition is met, the set is reasonably fast and efficient.
 *
 * \tparam K Type of the map keys.
 */
template<typename K>
class SmallSet
{
public:
  /**
   * \brief Check whether the given key is stored in this set.
   * \param key The key to find.
   * \return Whether the key is stored in this set.
   */
  bool contains(const K& key) const
  {
    return ::std::find(this->data.begin(), this->data.end(), key) != this->data.end();
  }

  /**
   * \brief Search this set for the key. If it is not found, store the key.
   * \param key The key to find.
   * \return Whether the key was missing and was inserted.
   */
  bool insert(const K& key)
  {
    auto it = std::find(this->data.begin(), this->data.end(), key);
    if (__builtin_expect(it == this->data.end(), 0))
    {
      std::unique_lock<std::mutex> lock(this->mutex);
      // Check once again with the lock; if key is still not there, insert it, otherwise find it
      it = std::find(this->data.begin(), this->data.end(), key);
      if (__builtin_expect(it == this->data.end(), 1))
      {
        this->data.emplace(this->data.end(), key);
        return true;
      }
    }
    return false;
  }

  /**
   * \brief Return the number of elements of this map.
   * \return The number of elements.
   */
  size_t size() const
  {
    return this->data.size();
  }

  /**
   * \brief Return whether this map is empty.
   * \return Whether the map is empty.
   */
  bool empty() const
  {
    return this->data.empty();
  }

private:
  ::std::list<K> data;
  ::std::mutex mutex;
};

}
