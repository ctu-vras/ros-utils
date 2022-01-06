#pragma once

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/xmlrpc_value_traits.hpp>

#include <list>
#include <set>
#include <unordered_set>
#include <unordered_map>

/**
 * This file provides helper methods easing access to parameters passed to nodes, nodelets and filters.
 */

namespace cras {

class BoundParamHelper;
typedef std::shared_ptr<BoundParamHelper> BoundParamHelperPtr;

struct RawGetParamAdapter {
  virtual bool getParam(const std::string &, bool&) const = 0;
  virtual bool getParam(const std::string &, int&) const = 0;
  virtual bool getParam(const std::string &, float&) const = 0;
  virtual bool getParam(const std::string &, double&) const = 0;
  virtual bool getParam(const std::string &, std::string&) const = 0;
  virtual bool getParam(const std::string &, std::vector<bool>&) const = 0;
  virtual bool getParam(const std::string &, std::vector<int>&) const = 0;
  virtual bool getParam(const std::string &, std::vector<float>&) const = 0;
  virtual bool getParam(const std::string &, std::vector<double>&) const = 0;
  virtual bool getParam(const std::string &, std::vector<std::string>&) const = 0;
  virtual bool getParam(const std::string &, std::map<std::string, bool>&) const = 0;
  virtual bool getParam(const std::string &, std::map<std::string, int>&) const = 0;
  virtual bool getParam(const std::string &, std::map<std::string, float>&) const = 0;
  virtual bool getParam(const std::string &, std::map<std::string, double>&) const = 0;
  virtual bool getParam(const std::string &, std::map<std::string, std::string>&) const = 0;
  virtual bool getParam(const std::string &, XmlRpc::XmlRpcValue&) const = 0;
  virtual std::string getNamespace() const = 0;
  virtual bool hasParam(const std::string&) const = 0;
  virtual std::shared_ptr<RawGetParamAdapter> getNamespaced(const std::string& ns) const = 0;

  // these would normally be written as specializations in ParamHelper, but C++ forbids partial function specialization
  template<typename T>
  bool getParam(const std::string& name, std::set<T>& set) const {
    std::vector<T> vector;
    const auto success = this->getParam(name, vector);
    if (!success)
      return false;
    set.clear();
    set.insert(vector.begin(), vector.end());
    return true;
  }

  template<typename T>
  bool getParam(const std::string& name, std::unordered_set<T>& set) const {
    std::vector<T> vector;
    const auto success = this->getParam(name, vector);
    if (!success)
      return false;
    set.clear();
    set.insert(vector.begin(), vector.end());
    return true;
  }

  template<typename T>
  bool getParam(const std::string& name, std::list<T>& list) const {
    std::vector<T> vector;
    const auto success = this->getParam(name, vector);
    if (!success)
      return false;
    list.clear();
    list.insert(vector.begin(), vector.end());
    return true;
  }

  template<typename K, typename V>
  bool getParam(const std::string& name, std::unordered_map<K, V>& map) const {
    std::map<K, V> tmp;
    const auto success = this->getParam(name, tmp);
    if (!success)
      return false;
    map.clear();
    map.insert(tmp.begin(), tmp.end());
    return true;
  }
};

struct NodeRawGetParamAdapter : public RawGetParamAdapter
{
  explicit NodeRawGetParamAdapter(const ros::NodeHandle &nh) : nh(nh) {}
  virtual ~NodeRawGetParamAdapter() = default;

  bool getParam(const std::string& name, bool& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, int& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, float& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, double& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::string& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::vector<bool>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::vector<int>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::vector<float>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::vector<double>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::vector<std::string>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::map<std::string, bool>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::map<std::string, int>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::map<std::string, float>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::map<std::string, double>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, std::map<std::string, std::string>& v) const override { return nh.getParam(name, v); }
  bool getParam(const std::string& name, XmlRpc::XmlRpcValue& v) const override { return nh.getParam(name, v); }

  std::string getNamespace() const override { return this->nh.getNamespace(); }
  bool hasParam(const std::string& name) const override { return this->nh.hasParam(name); }
  std::shared_ptr<RawGetParamAdapter> getNamespaced(const std::string &ns) const override {
    return std::make_shared<NodeRawGetParamAdapter>(ros::NodeHandle(this->nh, ns));
  }
protected:
  ros::NodeHandle nh;
};

template<typename T>
T getParam(ros::NodeHandle&, const std::string&, const T& = T(), const std::string& = "");
std::string getParam(ros::NodeHandle&, const std::string&, const char *, const std::string& = "");

/**
 * This class provides a unified experience for both nodes, nodelets and filters for getting ROS parameter values.
 * Each parameter has to be provided a default value, and each parameter read is logged - specified parameters with INFO
 * verbosity level, defaulted parameters with WARN level. There are also lots of template specializations for builtin
 * ROS types or unsigned values which ease the process of reading the parameters correctly.
 */
class ParamHelper {
public:
  /**
   * Create the param helper using the given log helper for logging messages.
   * @param log The log helper to use for logging.
   */
  explicit ParamHelper(const std::shared_ptr<LogHelper>& log) : log(log) {}
  virtual ~ParamHelper() = default;

  template<typename T>
  friend T getParam(ros::NodeHandle&, const std::string&, const T&, const std::string&);
  friend std::string getParam(ros::NodeHandle&, const std::string&, const char*, const std::string&);


protected:
  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value, and print out a
   *        ROS info/warning message with the loaded values.
   * \tparam T Param type (the C++ type; various specializations make it possible to convert different parameter server
   *         values to the corresponding C++ type if the conversion is non-trivial).
   * \param node The node handle to read the param value from.
   * \param name Name of the parameter.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
   * \return The loaded param value.
   */
  template<typename T>
  inline T getParam(const RawGetParamAdapter& param, const std::string &name,
                    const T &defaultValue = T(), const std::string &unit = "") const
  {
    T value;
    if (param.getParam(name, value))
    {
      this->log->logInfo("%s: Found parameter: %s, value: %s%s",
          param.getNamespace().c_str(), name.c_str(), cras::to_string(value).c_str(),
          cras::prependIfNonEmpty(unit, " ").c_str());
      return value;
    }
    else if (param.hasParam(name))
    {
      XmlRpc::XmlRpcValue v;
      param.getParam(name, v);
      this->log->logError("%s: Parameter %s found, but has wrong type. Expected XmlRpc type %s, got %s. Assigning default: %s%s",
        param.getNamespace().c_str(), name.c_str(),
        cras::XmlRpcValueTraits<T>::stringType, cras::to_string(v.getType()).c_str(),
        cras::to_string(defaultValue).c_str(), cras::prependIfNonEmpty(unit, " ").c_str());
      return defaultValue;
    }
    else
    {
      this->log->logWarn("%s: Cannot find value for parameter: %s, assigning default: %s%s",
           param.getNamespace().c_str(), name.c_str(), cras::to_string(defaultValue).c_str(),
           cras::prependIfNonEmpty(unit, " ").c_str());
      return defaultValue;
    }
  }

  // std::string - char interop specializations
  inline std::string getParam(const RawGetParamAdapter& param, const std::string &name, const char *defaultValue, const std::string &unit = "") const
  {
    return this->getParam<std::string>(param, name, std::string(defaultValue), unit);
  }

  // getParam helper for unsigned values
  template <typename Unsigned, typename Signed>
  inline Unsigned getParamUnsigned(const RawGetParamAdapter& param, const std::string& name, const Unsigned& defaultValue, const std::string& unit = "") const
  {
    const Signed signedValue = this->getParam(param, name, static_cast<Signed>(defaultValue), unit);
    if (signedValue < 0)
    {
      this->log->logError("%s: Value %i of unsigned parameter %s is negative.",
          param.getNamespace().c_str(), signedValue, name.c_str());
      throw std::invalid_argument(name);
    }
    return static_cast<Unsigned>(signedValue);
  }

  // converting getParam()
  template <typename TargetType, typename OrigType>
  inline TargetType getParamConvert(const RawGetParamAdapter& param, const std::string& name, const OrigType& defaultValue, const std::string& unit, std::function<TargetType(const OrigType&)> toTarget) const
  {
    const OrigType paramValue = this->getParam(param, name, defaultValue, unit);
    return toTarget(paramValue);
  }

  // converting getParam()
  template <typename TargetType, typename OrigType>
  inline TargetType getParamConvert(const RawGetParamAdapter& param, const std::string& name, const TargetType& defaultValue, const std::string& unit, std::function<TargetType(const OrigType&)> toTarget, std::function<OrigType(const TargetType&)> toOrig) const
  {
    const OrigType paramValue = this->getParam(param, name, toOrig(defaultValue), unit);
    return toTarget(paramValue);
  }

  // generic casting getParam()
  template <typename CastType, typename OrigType>
  inline CastType getParamCast(const RawGetParamAdapter& param, const std::string& name, const OrigType& defaultValue, const std::string& unit = "") const
  {
    return this->getParamConvert<CastType, OrigType>(param, name, defaultValue, unit,
      [](const OrigType& o) { return static_cast<CastType>(o); });
  }

  // generic casting getParam()
  template <typename CastType, typename OrigType>
  inline CastType getParamCast(const RawGetParamAdapter& param, const std::string& name, const CastType& defaultValue, const std::string& unit = "") const
  {
    return this->getParamConvert<CastType, OrigType>(param, name, defaultValue, unit,
      [](const OrigType& o) { return static_cast<CastType>(o); },
      [](const CastType& o) { return static_cast<OrigType>(o); });
  }

  std::shared_ptr<LogHelper> log; //!< The log helper to use for logging parameter read messages.
};

// getParam specializations for unsigned values

// try out the _sz operator from type_utils.hpp!
template<> inline size_t
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const size_t& defaultValue, const std::string& unit) const {
  return this->getParamUnsigned<size_t, int>(param, name, defaultValue, unit);
}

template<> inline unsigned int
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const unsigned int& defaultValue, const std::string& unit) const {
  return this->getParamUnsigned<unsigned int, int>(param, name, defaultValue, unit);
}

// ROS types specializations

template<> inline ros::Time
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const ros::Time& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::Time, double>(param, name, defaultValue.toSec(), unit);
}

template<> inline ros::Duration
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const ros::Duration& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::Duration, double>(param, name, defaultValue.toSec(), unit.empty() ? "s" : unit);
}

template<> inline ros::Rate
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const ros::Rate& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::Rate, double>(param, name, 1.0 / defaultValue.expectedCycleTime().toSec(), unit.empty() ? "Hz" : unit);
}

template<> inline ros::WallTime
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const ros::WallTime& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::WallTime, double>(param, name, defaultValue.toSec(), unit);
}

template<> inline ros::WallDuration
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const ros::WallDuration& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::WallDuration, double>(param, name, defaultValue.toSec(), unit.empty() ? "s" : unit);
}

template<> inline ros::WallRate
ParamHelper::getParam(const RawGetParamAdapter& param, const std::string& name, const ros::WallRate& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::WallRate, double>(param, name, 1.0 / defaultValue.expectedCycleTime().toSec(), unit.empty() ? "Hz" : unit);
}

/**
 * Bound param helper (allows omitting the param adapter in each getParam call).
 */
class BoundParamHelper : public ParamHelper
{
public:
  /**
   * Create the bound param helper.
   * @param log The log helper to use for logging parameter read messages.
   * @param param The raw parameter adapter to bind to.
   */
  BoundParamHelper(const std::shared_ptr<LogHelper>& log, const std::shared_ptr<RawGetParamAdapter>& param) :
    ParamHelper(log), param(param) {}

  /**
   * \brief Get the value of the given parameter, falling back to the specified default value, and print out a
   *      ROS info/warning message with the loaded values.
   * \tparam T Param type (the C++ type; various specializations make it possible to convert different parameter server
   *       values to the corresponding C++ type if the conversion is non-trivial).
   * \param name Name of the parameter.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
   * \return The loaded param value.
   */
  template<typename T>
  inline T getParam(const std::string &name, const T &defaultValue = T(), const std::string &unit = "") const {
    return ParamHelper::getParam(*this->param, name, defaultValue, unit);
  }

  // std::string - char interop specializations
  inline std::string getParam(const std::string &name, const char *defaultValue, const std::string &unit = "") const {
    return ParamHelper::getParam(*this->param, name, defaultValue, unit);
  }

  inline bool hasParam(const std::string& name) const { return this->param->hasParam(name); }

  inline std::shared_ptr<BoundParamHelper> paramsInNamespace(const std::string& ns) const
  {
    return std::make_shared<BoundParamHelper>(this->log, this->param->getNamespaced(ns));
  }

protected:
  std::shared_ptr<RawGetParamAdapter> param; //!< The bound parameter adapter.
};

}
