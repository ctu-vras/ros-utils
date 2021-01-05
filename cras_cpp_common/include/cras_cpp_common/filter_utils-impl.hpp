#pragma once

#include <string>
#include <rosconsole/macros_generated.h>
#include <filters/filter_base.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils.hpp>

/**
 * This is non-interesting code supporting the integration of ParamHelper and FilterBase.
 */

namespace cras
{

/**
 * Log helper redirecting the logging calls to ROS_ macros.
 */
class FilterLogHelper : public LogHelper {
protected:
  void printDebug(const std::string &text) const override { ROS_DEBUG("%s", text.c_str()); }
  void printInfo(const std::string &text) const override { ROS_INFO("%s", text.c_str()); }
  void printWarn(const std::string &text) const override { ROS_WARN("%s", text.c_str()); }
  void printError(const std::string &text) const override { ROS_ERROR("%s", text.c_str()); }
  void printFatal(const std::string &text) const override { ROS_FATAL("%s", text.c_str()); }
};

template<typename F>
class FilterBase;

template<typename F>
struct FilterRawGetParamAdapter : public RawGetParamAdapter
{
  explicit FilterRawGetParamAdapter(cras::FilterBase<F>* filter) : filter(filter) {}
  virtual ~FilterRawGetParamAdapter() = default;

  // overloads directly implemented in filters::FilterBase

  bool getParam(const std::string& name, bool& v) const override { return filter->getParamRaw(name, v); }
  bool getParam(const std::string& name, int& v) const override { return filter->getParamRaw(name, v); }
  bool getParam(const std::string& name, double& v) const override { return filter->getParamRaw(name, v); }
  bool getParam(const std::string& name, std::string& v) const override { return filter->getParamRaw(name, v); }
  bool getParam(const std::string& name, std::vector<double>& v) const override { return filter->getParamRaw(name, v); }
  bool getParam(const std::string& name, std::vector<std::string>& v) const override { return filter->getParamRaw(name, v); }
  bool getParam(const std::string& name, XmlRpc::XmlRpcValue& v) const override { return filter->getParamRaw(name, (XmlRpc::XmlRpcValue&)v); }

  // other overloads that need custom implementation

  bool getParam(const std::string& name, float& v) const override {
    double d;
    const auto success = this->filter->getParamRaw(name, d);
    if (success)
      v = static_cast<float>(d);
    return success;
  }

  template<typename T>
  bool getParamVector(const std::string& name, std::vector<T>& v, const XmlRpc::XmlRpcValue::Type type) const {
    XmlRpc::XmlRpcValue xml;
    const auto success = this->getParam(name, xml);

    if (!success)
      return false;

    v.clear();

    if(xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return false;

    for (size_t i = 0; i < xml.size(); ++i) {
      if(xml[i].getType() != type &&
         !(type == XmlRpc::XmlRpcValue::TypeBoolean && xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt) &&
         !(type == XmlRpc::XmlRpcValue::TypeBoolean && xml[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) &&
         !(type == XmlRpc::XmlRpcValue::TypeDouble &&  xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        return false;
      }
      v.push_back(xml[i]);
    }

    return true;
  }

  bool getParam(const std::string& name, std::vector<bool>& v) const override {
    return this->getParamVector(name, v, XmlRpc::XmlRpcValue::TypeBoolean);
  }
  bool getParam(const std::string& name, std::vector<int>& v) const override {
    return this->getParamVector(name, v, XmlRpc::XmlRpcValue::TypeInt);
  }
  bool getParam(const std::string& name, std::vector<float>& v) const override {
    std::vector<double> vec;
    const auto success = this->getParam(name, vec);
    if (success) {
      v.clear();
      for (const auto x : vec)
        v.push_back(static_cast<float>(x));
    }
    return success;
  }

  template<typename T>
  bool getParamMap(const std::string& name, std::map<std::string, T>& v, const XmlRpc::XmlRpcValue::Type type) const {
    XmlRpc::XmlRpcValue xml;
    const auto success = this->getParam(name, xml);

    if (!success)
      return false;

    v.clear();

    if(xml.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      return false;

    for (auto it = xml.begin(); it != xml.end(); ++it) {
      auto pair = *it;
      if(pair.second.getType() != type &&
         !(type == XmlRpc::XmlRpcValue::TypeBoolean && pair.second.getType() == XmlRpc::XmlRpcValue::TypeInt) &&
         !(type == XmlRpc::XmlRpcValue::TypeBoolean && pair.second.getType() == XmlRpc::XmlRpcValue::TypeDouble) &&
         !(type == XmlRpc::XmlRpcValue::TypeDouble && pair.second.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        return false;
      }
      v[pair.first] = (T)pair.second;
    }

    return true;
  }

  bool getParam(const std::string& name, std::map<std::string, bool>& v) const override {
    return this->getParamMap(name, v, XmlRpc::XmlRpcValue::TypeBoolean);
  }
  bool getParam(const std::string& name, std::map<std::string, int>& v) const override {
    return this->getParamMap(name, v, XmlRpc::XmlRpcValue::TypeInt);
  }
  bool getParam(const std::string& name, std::map<std::string, double>& v) const override {
    return this->getParamMap(name, v, XmlRpc::XmlRpcValue::TypeDouble);
  }
  bool getParam(const std::string& name, std::map<std::string, float>& v) const override {
    std::map<std::string, double> vec;
    const auto success = this->getParam(name, vec);
    if (success) {
      v.clear();
      for (const auto x : vec)
        v[x.first] = static_cast<float>(x.second);
    }
    return success;
  }
  bool getParam(const std::string& name, std::map<std::string, std::string>& v) const override {
    return this->getParamMap(name, v, XmlRpc::XmlRpcValue::TypeString);
  }

  std::string getNamespace() const override { return this->filter->filter_name_; }
  bool hasParam(const std::string& name) const override { return this->filter->hasParam(name); }
  std::shared_ptr<RawGetParamAdapter> getNamespaced(const std::string &ns) const override {
    throw std::runtime_error("Filters do not offer namespaced parameter adapters.");
  }
protected:
  cras::FilterBase<F>* filter;
};


}
