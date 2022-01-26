#pragma once

#include <xmlrpcpp/XmlRpcValue.h>

namespace cras {

inline std::string to_string(const XmlRpc::XmlRpcValue& value)
{
  return value.toXml();
}

}
