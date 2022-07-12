#if HAS_FILTERCHAIN_GET_FILTERS!=1
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private protected
#include <filters/filter_chain.h>
#undef private
#endif

#include <cras_cpp_common/filter_utils.hpp>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace cras
{

template <typename F>
struct FilterChainPrivate
{
  F buffer0;
  F buffer1;
  typename FilterChain<F>::FilterCallback filterCallback;
  std::unordered_set<std::string> disabledFilters;
#if HAS_FILTERCHAIN_GET_FILTERS==1
  std::vector<std::shared_ptr<filters::FilterBase<F> > > activeFilters;
#else
  std::vector<boost::shared_ptr<filters::FilterBase<F> > > activeFilters;
#endif
};

template <typename F>
void FilterChain<F>::setNodelet(const nodelet::Nodelet* nodelet) {
#if HAS_FILTERCHAIN_GET_FILTERS==1
  for (const auto& filter : this->getFilters()) {
    if (std::dynamic_pointer_cast<cras::FilterBase<F>>(filter)) {
      std::dynamic_pointer_cast<cras::FilterBase<F>>(filter)->setNodelet(nodelet);
    }
  }
#else
  for (const auto& filter : this->reference_pointers_) {
    if (boost::dynamic_pointer_cast<cras::FilterBase<F>>(filter)) {
      boost::dynamic_pointer_cast<cras::FilterBase<F>>(filter)->setNodelet(nodelet);
    }
  }
#endif
}

template <typename F>
bool FilterChain<F>::update(const F &data_in, F &data_out) {

#if HAS_FILTERCHAIN_GET_FILTERS==1
  if (this->data->activeFilters.size() + this->data->disabledFilters.size() != this->getFilters().size())
#else
  if (this->data->activeFilters.size() + this->data->disabledFilters.size() != this->reference_pointers_.size())
#endif
    this->updateActiveFilters();

  unsigned int list_size = this->data->activeFilters.size();
  bool result;
  if (list_size == 0)
  {
    data_out = data_in;
    result = true;
  }
  else if (list_size == 1)
  {
    result = this->data->activeFilters[0]->update(data_in, data_out);
    if (result) this->callCallback(data_out, 0);
  }
  else if (list_size == 2)
  {
    result = this->data->activeFilters[0]->update(data_in, this->data->buffer0);
    if (result == false) {return false; };//don't keep processing on failure
    if (this->data->filterCallback)
      this->callCallback(this->data->buffer0, 0);
    result = result && this->data->activeFilters[1]->update(this->data->buffer0, data_out);
    if (result) this->callCallback(data_out, 1);
  }
  else
  {
    result = this->data->activeFilters[0]->update(data_in, this->data->buffer0);  //first copy in
    if (result == false) {return false; }; //don't keep processing on failure
    this->callCallback(this->data->buffer0, 0);
    for (unsigned int i = 1; i <  this->data->activeFilters.size() - 1; i++) // all but first and last (never called if size=2)
    {
      if (i %2 == 1) {
        result = result && this->data->activeFilters[i]->update(this->data->buffer0, this->data->buffer1);
        if (result) this->callCallback(this->data->buffer1, i);
      }
      else {
        result = result && this->data->activeFilters[i]->update(this->data->buffer1, this->data->buffer0);
        if (result) this->callCallback(this->data->buffer0, i);
      }

      if (result == false) {return false; }; //don't keep processing on failure
    }
    if (list_size % 2 == 1) // odd number last deposit was in this->buffer1
      result = result && this->data->activeFilters.back()->update(this->data->buffer1, data_out);
    else
      result = result && this->data->activeFilters.back()->update(this->data->buffer0, data_out);

    if (result) this->callCallback(data_out, this->data->activeFilters.size() - 1);
  }
  return result;
}

template <typename F>
void FilterChain<F>::callCallback(const F &data, size_t filterNum) {
  if (this->data->filterCallback)
  {
#if HAS_FILTERCHAIN_GET_FILTERS==1
    const auto& filters = this->getFilters();
    const auto& filter = filters[filterNum];
#else
    const auto& filter = this->reference_pointers_[filterNum];
#endif
    this->data->filterCallback(data, filterNum, filter->getName(), filter->getType());
  }
}

template<typename F>
void FilterChain<F>::disableFilter(const std::string &name)
{
  this->data->disabledFilters.insert(name);
  ROS_DEBUG("Disabled filter %s", name.c_str());
}

template<typename F>
void FilterChain<F>::enableFilter(const std::string &name)
{
  this->data->disabledFilters.erase(name);
  ROS_DEBUG("Enabled filter %s", name.c_str());
}

template<typename F>
void FilterChain<F>::updateActiveFilters()
{
  this->data->activeFilters.clear();
#if HAS_FILTERCHAIN_GET_FILTERS==1
  for (const auto& filter : this->getFilters())
#else
  for (const auto& filter : this->reference_pointers_)
#endif
  {
    if (this->data->disabledFilters.find(filter->getName()) == this->data->disabledFilters.end())
      this->data->activeFilters.push_back(filter);
  }
}

template<typename F>
void FilterChain<F>::setDisabledFilters(std::unordered_set<std::string> filters)
{
  this->data->disabledFilters = std::move(filters);
}

template<typename F>
FilterChain<F>::FilterChain(const std::string &dataType,
                            const FilterChain::FilterCallback &filterCallback)
  : filters::FilterChain<F>(dataType), data(new FilterChainPrivate<F>())
{
  this->data->filterCallback = filterCallback;
}

template<typename F>
FilterChain<F>::~FilterChain()
{
}

template class FilterChain<sensor_msgs::LaserScan>;
template class FilterChain<sensor_msgs::PointCloud2>;

}
