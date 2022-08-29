#include <memory>
#include <string>

#include <ros/exceptions.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <XmlRpcValue.h>

#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>

namespace cras
{

NodeHandleGetParamAdapter::NodeHandleGetParamAdapter(const ::ros::NodeHandle& nh) noexcept : nh(nh)
{
}

NodeHandleGetParamAdapter::~NodeHandleGetParamAdapter() = default;

bool NodeHandleGetParamAdapter::getParam(const ::std::string& name, ::XmlRpc::XmlRpcValue& v) const noexcept
{
  try
  {
    // NodeHandle::getParam("/") gets "cleaned" to NodeHandle::getParam("~") which we don't want
    if (name != "/")
      return this->nh.getParam(name, v);
    else
      return ros::param::get(name, v);
  }
  catch (const ::ros::InvalidNameException&)
  {
    return false;
  }
}

::std::string NodeHandleGetParamAdapter::getNamespace() const noexcept
{
  return this->nh.getNamespace();
}

bool NodeHandleGetParamAdapter::hasParam(const ::std::string& name) const noexcept
{
  try
  {
    return nh.hasParam(name);
  }
  catch (const ::ros::InvalidNameException&)
  {
    return false;
  }
}

::std::shared_ptr<::cras::GetParamAdapter>
NodeHandleGetParamAdapter::getNamespaced(const ::std::string& ns) const noexcept(false)
{
  return ::std::make_shared<NodeHandleGetParamAdapter>(::ros::NodeHandle(this->nh, ns));
}

}