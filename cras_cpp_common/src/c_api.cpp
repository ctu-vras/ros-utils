// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support definitions for declaration of a C API of modules
 * \author Martin Pecka
 */

#include <cras_cpp_common/c_api.hpp>

#include <cstring>
#include <string>
#include <vector>

namespace cras {

char* outputString(const cras::allocator_t allocator, const char* string, const size_t length) {
  const auto buffer = static_cast<char*>(allocator(length));
  strncpy(buffer, string, length);
  return buffer;
}

char* outputString(const allocator_t allocator, const std::string& string) {
  return outputString(allocator, string.c_str(), string.size() + 1);
}

uint8_t* outputByteBuffer(const allocator_t allocator, const uint8_t* bytes, const size_t length) {
  const auto buffer = allocator(length);
  return static_cast<uint8_t*>(memcpy(buffer, bytes, length));
}

uint8_t* outputByteBuffer(const allocator_t allocator, const std::vector<uint8_t>& bytes) {
  return outputByteBuffer(allocator, bytes.data(), bytes.size());
}

}  // namespace cras
