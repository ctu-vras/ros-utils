#include <cras_cpp_common/nodelet_utils.hpp>

#include <pthread.h>

namespace cras
{

void Nodelet::updateThreadName() const
{
  char nameBuf[16];
  const auto& name = stripLeadingSlash(this->getName());

  if (name.length() <= 15) {
    memcpy(nameBuf, name.c_str(), name.length());
    nameBuf[name.length()] = '\0';
  } else {
    memcpy(nameBuf, name.c_str(), 7);
    memset(nameBuf + 7, '.', 1);
    memcpy(nameBuf + 8, name.c_str() + (name.length() - 7), 7);
    nameBuf[15] = '\0';
  }
  pthread_setname_np(pthread_self(), nameBuf);
}

}