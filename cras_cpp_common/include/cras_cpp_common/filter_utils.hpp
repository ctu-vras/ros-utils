#pragma once

#include <functional>
#include <unordered_set>

#include <rosconsole/macros_generated.h>
#include <filters/filter_base.h>
#include <filters/filter_chain.h>
#include <nodelet/nodelet.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/filter_utils-impl.hpp>

/**
 * This file contains helpers for working with filters based on filter::FilterBase and filter chains.
 */

namespace cras
{

template<typename F> struct FilterChainPrivate;

/**
 * This filterchain implementation allows for selectively disabling/enabling filters during run time.
 * It also adds a callback with the result of each filter run, so that you can e.g. publish the output of each filter.
 * @tparam F Type of the filtered data.
 */
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
  explicit FilterChain(const std::string &dataType, const FilterCallback& filterCallback = {});
  virtual ~FilterChain();
  void setNodelet(const nodelet::Nodelet* nodelet);
  bool update(const F& data_in, F& data_out);
  void disableFilter(const std::string& name);
  void enableFilter(const std::string& name);
  void setDisabledFilters(std::unordered_set<std::string> filters);
protected:
  void callCallback(const F& data, size_t filterNum);
  void updateActiveFilters();
private:
  std::unique_ptr<FilterChainPrivate<F>> data;
};


/**
 * This FilterBase implementation adds access to the templated getParam() methods from param_utils. It also adds
 * possibility to inform the filter about the nodelet it is running in (if it is running in one).
 * @tparam F
 */
template<typename F>
class FilterBase : public filters::FilterBase<F>, public ParamHelper
{

public:
  FilterBase() : ParamHelper(std::make_shared<FilterLogHelper>()), params(this) {}

  /**
   * Inform this filter that it is running inside the passed nodelet.
   * This should be called after configure().
   * @param nodelet The nodelet running this filter.
   */
  void setNodelet(const nodelet::Nodelet* nodelet)
  {
    this->nodelet = nodelet;
  }

  friend struct FilterRawGetParamAdapter<F>;

protected:

  const nodelet::Nodelet* nodelet;
  FilterRawGetParamAdapter<F> params;

  bool getParamRaw(const std::string& name, bool& v) const { return filters::FilterBase<F>::getParam(name, (bool&)v); }
  bool getParamRaw(const std::string& name, int& v) const { return filters::FilterBase<F>::getParam(name, (int&)v); }
  bool getParamRaw(const std::string& name, double& v) const { return filters::FilterBase<F>::getParam(name, (double&)v); }
  bool getParamRaw(const std::string& name, std::string& v) const { return filters::FilterBase<F>::getParam(name, (std::string&)v); }
  bool getParamRaw(const std::string& name, std::vector<double>& v) const { return filters::FilterBase<F>::getParam(name, (std::vector<double>&)v); }
  bool getParamRaw(const std::string& name, std::vector<std::string>& v) const { return filters::FilterBase<F>::getParam(name, (std::vector<std::string>&)v); }
  bool getParamRaw(const std::string& name, XmlRpc::XmlRpcValue& v) const { return filters::FilterBase<F>::getParam(name, (XmlRpc::XmlRpcValue&)v); }

  bool hasParam(const std::string& name) const
  {
    return this->params_.find(name) != this->params_.end();
  }

  template<typename T>
  inline T getParam(const std::string& name, const T& defaultValue = T(), const std::string& unit = "") const {
    return ParamHelper::getParam(this->params, name, defaultValue, unit);
  }

  inline std::string getParam(const std::string &name, const char *defaultValue, const std::string &unit = "") const {
    return ParamHelper::getParam(this->params, name, defaultValue, unit);
  }

};

}
