#pragma once

#include <cras_cpp_common/log_utils.h>

namespace cras {

class BoundParamHelper;
typedef std::shared_ptr<BoundParamHelper> BoundParamHelperPtr;

/**
 * This class provides a unified experience for both nodes and nodelets for getting ROS parameter values.
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

  /**
   * Creates a version of this param helper "bound" to the given node handle, so that it is not needed to specify the
   * node handle in the subsequent getParam calls.
   * @param node The node to bind to.
   * @return The bound param helper.
   */
  inline BoundParamHelperPtr paramsForNodeHandle(ros::NodeHandle& node) const;

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
  inline T getParam(ros::NodeHandle &node, const std::string &name,
                    const T &defaultValue = T(), const std::string &unit = "") const
  {
    T value;
    if (node.getParam(name, value))
    {
      this->log->logInfo("%s: Found parameter: %s, value: %s%s",
          node.getNamespace().c_str(), name.c_str(), cras::to_string(value).c_str(),
          cras::prependIfNonEmpty(unit, " ").c_str());
      return value;
    }
    else
    {
      this->log->logWarn("%s: Cannot find value for parameter: %s, assigning default: %s%s",
           node.getNamespace().c_str(), name.c_str(), cras::to_string(defaultValue).c_str(),
           cras::prependIfNonEmpty(unit, " ").c_str());
      return defaultValue;
    }
  }

  // std::string - char interop specializations
  inline std::string getParam(ros::NodeHandle &node, const std::string &name, const char *defaultValue, const std::string &unit = "") const
  {
    return this->getParam<std::string>(node, name, std::string(defaultValue), unit);
  }

protected:

  // getParam helper for unsigned values
  template <typename Unsigned, typename Signed>
  inline Unsigned getParamUnsigned(ros::NodeHandle& node, const std::string& name, const Unsigned& defaultValue, const std::string& unit = "") const
  {
    const Signed signedValue = this->getParam(node, name, static_cast<Signed>(defaultValue), unit);
    if (signedValue < 0)
    {
      this->log->logError("%s: Value %i of unsigned parameter %s is negative.",
          node.getNamespace().c_str(), signedValue, name.c_str());
      throw std::invalid_argument(name);
    }
    return static_cast<Unsigned>(signedValue);
  }

  // converting getParam()
  template <typename TargetType, typename OrigType>
  inline TargetType getParamConvert(ros::NodeHandle& node, const std::string& name, const OrigType& defaultValue, const std::string& unit, std::function<TargetType(const OrigType&)> toTarget) const
  {
    const OrigType paramValue = this->getParam(node, name, defaultValue, unit);
    return toTarget(paramValue);
  }

  // converting getParam()
  template <typename TargetType, typename OrigType>
  inline TargetType getParamConvert(ros::NodeHandle& node, const std::string& name, const TargetType& defaultValue, const std::string& unit, std::function<TargetType(const OrigType&)> toTarget, std::function<OrigType(const TargetType&)> toOrig) const
  {
    const OrigType paramValue = this->getParam(node, name, toOrig(defaultValue), unit);
    return toTarget(paramValue);
  }

  // generic casting getParam()
  template <typename CastType, typename OrigType>
  inline CastType getParamCast(ros::NodeHandle& node, const std::string& name, const OrigType& defaultValue, const std::string& unit = "") const
  {
    return this->getParamConvert<CastType, OrigType>(node, name, defaultValue, unit,
      [](const OrigType& o) { return static_cast<CastType>(o); });
  }

  // generic casting getParam()
  template <typename CastType, typename OrigType>
  inline CastType getParamCast(ros::NodeHandle& node, const std::string& name, const CastType& defaultValue, const std::string& unit = "") const
  {
    return this->getParamConvert<CastType, OrigType>(node, name, defaultValue, unit,
      [](const OrigType& o) { return static_cast<CastType>(o); },
      [](const CastType& o) { return static_cast<OrigType>(o); });
  }

  std::shared_ptr<LogHelper> log; //!< The log helper to use for logging parameter read messages.
};

// getParam specializations for unsigned values

// try out the _sz operator from type_utils.hpp!
template<> inline size_t
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const size_t& defaultValue, const std::string& unit) const {
  return this->getParamUnsigned<size_t, int>(node, name, defaultValue, unit);
}

template<> inline unsigned int
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const unsigned int& defaultValue, const std::string& unit) const {
  return this->getParamUnsigned<unsigned int, int>(node, name, defaultValue, unit);
}

// ROS types specializations

template<> inline ros::Time
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const ros::Time& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::Time, double>(node, name, defaultValue.toSec(), unit);
}

template<> inline ros::Duration
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const ros::Duration& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::Duration, double>(node, name, defaultValue.toSec(), unit.empty() ? "s" : unit);
}

template<> inline ros::Rate
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const ros::Rate& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::Rate, double>(node, name, 1.0 / defaultValue.expectedCycleTime().toSec(), unit.empty() ? "Hz" : unit);
}

template<> inline ros::WallTime
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const ros::WallTime& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::WallTime, double>(node, name, defaultValue.toSec(), unit);
}

template<> inline ros::WallDuration
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const ros::WallDuration& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::WallDuration, double>(node, name, defaultValue.toSec(), unit.empty() ? "s" : unit);
}

template<> inline ros::WallRate
ParamHelper::getParam(ros::NodeHandle& node, const std::string& name, const ros::WallRate& defaultValue, const std::string& unit) const {
  return this->getParamCast<ros::WallRate, double>(node, name, 1.0 / defaultValue.expectedCycleTime().toSec(), unit.empty() ? "Hz" : unit);
}

/**
 * Bound param helper (allows omitting the nodehandle in each getParam call).
 */
class BoundParamHelper : public ParamHelper
{
public:
  /**
   * Create the bound param helper.
   * @param log The log helper to use for logging parameter read messages.
   * @param node The node handle to bind to.
   */
  BoundParamHelper(const std::shared_ptr<LogHelper> &log, ros::NodeHandle& node) : ParamHelper(log), node(node) {}

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value, and print out a
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
    return ParamHelper::getParam(this->node, name, defaultValue, unit);
  }

  // std::string - char interop specializations
  inline std::string getParam(const std::string &name, const char *defaultValue, const std::string &unit = "") const {
    return ParamHelper::getParam(this->node, name, defaultValue, unit);
  }

  inline bool hasParam(const std::string& param) const { return this->node.hasParam(param); }

  inline std::shared_ptr<BoundParamHelper> paramsInNamespace(const std::string& ns) const
  {
    ros::Subscriber s;
    ros::NodeHandle nh(this->node, ns);
    return this->paramsForNodeHandle(nh);
  }

protected:
  mutable ros::NodeHandle node; //!< The bound node handle.
};

inline std::shared_ptr<BoundParamHelper> ParamHelper::paramsForNodeHandle(ros::NodeHandle& node) const
{
  return std::make_shared<BoundParamHelper>(this->log, node);
}

}
