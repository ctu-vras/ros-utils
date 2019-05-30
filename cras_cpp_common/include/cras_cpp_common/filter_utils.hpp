#ifndef CRAS_CPP_COMMON_FILTER_UTILS_HPP
#define CRAS_CPP_COMMON_FILTER_UTILS_HPP

#include <functional>

#include <rosconsole/macros_generated.h>
#include <filters/filter_base.h>
#include <filters/filter_chain.h>
#include <nodelet/nodelet.h>

#include "cras_cpp_common/string_utils.hpp"

namespace cras
{

template<typename F>
class FilterChain : public filters::FilterChain<F>
{
public:
  /**
   * \brief Callback to be called after each filter processes the data.
   *
   * \param data The data after application of the filter.
   * \param filterNum The number of the filter in the filtering chain.
   * \param name Name of the filter that processed the data.
   * \param type Type of the filter that processed the data.
   */
  typedef std::function<void(const F& data, const size_t filterNum, const std::string& name, const std::string& type)> FilterCallback;

  /**
   * \brief Construct a filter chain.
   * \param dataType Textual representation of the data type.
   * \param filterCallback Optional callback to be called after each filter finishes its work.
   */
  explicit FilterChain(const std::string &dataType, const FilterCallback& filterCallback = {})
      : filters::FilterChain<F>(dataType), filterCallback(filterCallback) {};
  void setNodelet(const nodelet::Nodelet* nodelet);
  bool update(const F& data_in, F& data_out);
protected:
   FilterCallback filterCallback;

   void callCallback(const F& data, size_t filterNum);
};

template<typename F>
class FilterBase : public filters::FilterBase<F>
{

public:
  void setNodelet(const nodelet::Nodelet* nodelet)
  {
    this->nodelet = nodelet;
  }

protected:

  const nodelet::Nodelet* nodelet;

  /**
   * \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Param type.
   * \param name Name of the parameter.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
   * \return The loaded param value.
   */
  template< typename T>
  T getParamVerbose(const std::string &name, const T &defaultValue = T(),
             const std::string &unit = "")
  {
    T value;
    if (filters::FilterBase<F>::getParam(name, value))
    {
      ROS_INFO_STREAM(this->getName() << ": Found parameter: " << name <<
        ", value: " << cras::to_string(value) <<
        cras::prependIfNonEmpty(unit, " "));
      return value;
    }
    else
    {
      ROS_WARN_STREAM(this->getName() << ": Cannot find value for parameter: "
        << name << ", assigning default: " << cras::to_string(defaultValue)
        << cras::prependIfNonEmpty(unit, " "));
    }
    return defaultValue;
  }

  std::string getParamVerbose(const std::string &name, const char* defaultValue,
                              const std::string &unit = "")
  {
    return this->getParamVerbose(name, std::string(defaultValue), unit);
  }



  // getParam specializations for unsigned values

  uint64_t getParamVerbose(const std::string &name, const uint64_t &defaultValue,
                           const std::string &unit = "")
  {
    return this->getParamUnsigned<uint64_t, int>(name, defaultValue, unit);
  }

  // there actually is an unsigned int implementation of FilterBase::getParam,
  // but it doesn't tell you when the passed value is negative - instead it just
  // returns false
  unsigned int getParamVerbose(const std::string &name,
                               const unsigned int &defaultValue,
                               const std::string &unit = "")
  {
    return this->getParamUnsigned<unsigned int, int, F>(name, defaultValue,
      unit);
  }

  // ROS types specializations

  ros::Duration getParamVerbose(const std::string &name,
                                const ros::Duration &defaultValue,
                                const std::string &unit = "")
  {
    return this->getParamCast<ros::Duration, double>(name, defaultValue.toSec(),
      unit);
  }

private:

  template<typename Result, typename Param>
  Result getParamUnsigned(const std::string &name, const Result &defaultValue,
                          const std::string &unit = "")
  {
    const Param signedValue = this->getParamVerbose(name,
      static_cast<Param>(defaultValue), unit);
    if (signedValue < 0)
    {
      ROS_ERROR_STREAM(this->getName() << ": Value " << signedValue <<
        " of unsigned parameter " << name << " is negative.");
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signedValue);
  }

  // generic casting getParam()
  template<typename Result, typename Param>
  Result getParamCast(const std::string &name, const Param &defaultValue,
                      const std::string &unit = "")
  {
    const Param paramValue = this->getParamVerbose(name, defaultValue, unit);
    return Result(paramValue);
  }

};

};
#endif //CRAS_CPP_COMMON_FILTER_UTILS_HPP
