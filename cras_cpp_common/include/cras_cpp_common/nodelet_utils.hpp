#ifndef CRAS_CPP_COMMON_NODELET_UTILS_HPP
#define CRAS_CPP_COMMON_NODELET_UTILS_HPP

#include <string>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include <nodelet/nodelet.h>

#include "cras_cpp_common/string_utils.hpp"

namespace cras {

class Nodelet : public ::nodelet::Nodelet {

protected:

  /**
   * \brief Set custom name of the current thread to this nodelet's name.
   *
   * \note The name will be automatically shortened if longer than 15 chars.
   * \note You can see the custom names in htop when you enable display of
   *       custom thread names in options.
   * \note This function doesn't reset the name back to the original.
   */
  void updateThreadName() const;

  /**
   * \brief Get the value of the given ROS parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with the loaded values.
   *
   * \tparam T Param type.
   * \param node The node handle to read the param value from.
   * \param name Name of the parameter.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
   * \return The loaded param value.
   */
  template <typename T>
  inline T getParam(ros::NodeHandle &node, const std::string &name,
                    const T &defaultValue = T(),
                    const std::string &unit = "") {
    T value;
    if (node.getParam(name, value)) {
      NODELET_INFO_STREAM(node.getNamespace()
                          << ": Found parameter: " << name
                          << ", value: " << cras::to_string(value)
                          << cras::prependIfNonEmpty(unit, " "));
      return value;
    } else {
      NODELET_WARN_STREAM(node.getNamespace()
                          << ": Cannot find value for parameter: " << name
                          << ", assigning default: "
                          << cras::to_string(defaultValue)
                          << cras::prependIfNonEmpty(unit, " "));
    }
    return defaultValue;
  }

  // std::string - char interop specializations

  inline std::string getParam(ros::NodeHandle &node,
                              const std::string &name,
                              const char *defaultValue,
                              const std::string &unit = "") {
    return this->getParam<std::string>(node, name, std::string(defaultValue),
                                       unit);
  }

private:

  template <typename Result, typename Param>
  inline Result getParamUnsigned(ros::NodeHandle &node,
                                 const std::string &name,
                                 const Result &defaultValue,
                                 const std::string &unit = "") {
    const Param signedValue =
        this->getParam(node, name, static_cast<Param>(defaultValue), unit);
    if (signedValue < 0) {
      NODELET_ERROR_STREAM(node.getNamespace() << ": Value " << signedValue
                                               << " of unsigned parameter "
                                               << name << " is negative.");
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signedValue);
  }

  // generic casting getParam()
  template <typename Result, typename Param>
  inline Result
  getParamCast(ros::NodeHandle &node, const std::string &name,
               const Param &defaultValue, const std::string &unit = "") {
    const Param paramValue = this->getParam(node, name, defaultValue, unit);
    return Result(paramValue);
  }

};

// getParam specializations for unsigned values

template<> inline uint64_t
Nodelet::getParam(ros::NodeHandle &node, const std::string &name,
                  const uint64_t &defaultValue, const std::string &unit) {
  return this->getParamUnsigned<uint64_t, int>(node, name, defaultValue, unit);
}

template<> inline unsigned int
Nodelet::getParam(ros::NodeHandle &node, const std::string &name,
                  const unsigned int &defaultValue, const std::string &unit) {
  return this->getParamUnsigned<unsigned int, int>(node, name, defaultValue, unit);
}

// ROS types specializations

template<> inline ros::Duration
Nodelet::getParam(ros::NodeHandle &node, const std::string &name,
                  const ros::Duration &defaultValue, const std::string &unit) {
  return this->getParamCast<ros::Duration, double>(node, name, defaultValue.toSec(), unit);
}

}

#endif //CRAS_CPP_COMMON_NODELET_UTILS_HPP