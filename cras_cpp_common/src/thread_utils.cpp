/**
 * \file
 * \brief Utilities for working with threads.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <pthread.h>
#include <string>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/thread_utils.h>

namespace cras
{

std::string getThreadName()
{
  char buf[16];
  auto res = pthread_getname_np(pthread_self(), buf, 16);
  if (res != 0)
    return "";
  return {buf};
}

void setThreadName(const std::string& name)
{
  char nameBuf[16];

  if (name.length() <= 15)
  {
    memcpy(nameBuf, name.c_str(), name.length());
    nameBuf[name.length()] = '\0';
  }
  else
  {
    memcpy(nameBuf, name.c_str(), 7);
    memset(nameBuf + 7, '.', 1);
    memcpy(nameBuf + 8, name.c_str() + (name.length() - 7), 7);
    nameBuf[15] = '\0';
  }
  pthread_setname_np(pthread_self(), nameBuf);
}

}
