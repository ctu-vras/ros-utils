#pragma once

/**
 * \file
 * \brief Definitions of cras::FilterChain methods.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ros/common.h>
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_base.hpp>
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_base.h>
#include <filters/filter_chain.h>
#endif
#include <nodelet/nodelet.h>

#include <cras_cpp_common/filter_utils/filter_chain.hpp>
#include <cras_cpp_common/filter_utils/filter_base.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/nodelet.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>

namespace cras
{

template<typename F>
FilterChain<F>::FilterChain(
  const ::std::string& dataType, const FilterChain::FilterFinishedCallback& filterFinishedCallback,
  const FilterChain::FilterStartCallback& filterStartCallback, const ::cras::LogHelperPtr& logHelper) :
    ::filters::FilterChain<F>(dataType), ::cras::HasLogger(logHelper), filterStartCallback(filterStartCallback),
    filterFinishedCallback(filterFinishedCallback)
{
}

template <typename F>
void FilterChain<F>::setNodelet(const ::nodelet::Nodelet* nodelet)
{
  for (const auto& filter : this->getFilters())
  {
    auto crasFilter = ::std::dynamic_pointer_cast<::cras::FilterBase<F>>(filter);
    if (crasFilter != nullptr)
      crasFilter->setNodelet(nodelet);
  }
}

template <typename F>
void FilterChain<F>::setFilterStartCallback(const FilterChain::FilterStartCallback& callback)
{
  this->filterStartCallback = callback;
}

template <typename F>
void FilterChain<F>::setFilterFinishedCallback(const FilterChain::FilterFinishedCallback& callback)
{
  this->filterFinishedCallback = callback;
}

template <typename F>
bool FilterChain<F>::update(const F &data_in, F &data_out)
{
  if (!this->initialized || this->activeFilters.size() + this->disabledFilters.size() != this->getFilters().size())
  {
    this->updateActiveFilters();
    this->initialized = true;
  }

  ::std::lock_guard<::std::mutex> lock(this->activeFiltersMutex);

  size_t listSize = this->activeFilters.size();
  bool result;

  if (listSize == 0)
  {
    data_out = data_in;
    result = true;
  }
  else if (listSize == 1)
  {
    this->callStartCallback(data_in, 0);
    result = this->activeFilters[0]->update(data_in, data_out);
    this->callFinishedCallback(data_out, 0, result);
  }
  else if (listSize == 2)
  {
    this->callStartCallback(data_in, 0);
    result = this->activeFilters[0]->update(data_in, this->buffer0);
    this->callFinishedCallback(this->buffer0, 0, result);
    if (result == false) return false;  // don't keep processing on failure

    this->callStartCallback(this->buffer0, 1);
    result = result && this->activeFilters[1]->update(this->buffer0, data_out);
    this->callFinishedCallback(data_out, 1, result);
  }
  else
  {
    this->callStartCallback(data_in, 0);
    result = this->activeFilters[0]->update(data_in, this->buffer0);  // first copy in
    this->callFinishedCallback(this->buffer0, 0, result);
    if (result == false) return false;  // don't keep processing on failure

    // all but first and last (never called if size=2)
    for (size_t i = 1; i <  this->activeFilters.size() - 1; i++)
    {
      if (i % 2 == 1)
      {
        this->callStartCallback(this->buffer0, i);
        result = result && this->activeFilters[i]->update(this->buffer0, this->buffer1);
        this->callFinishedCallback(this->buffer1, i, result);
      }
      else
      {
        this->callStartCallback(this->buffer1, i);
        result = result && this->activeFilters[i]->update(this->buffer1, this->buffer0);
        this->callFinishedCallback(this->buffer0, i, result);
      }

      if (result == false) return false;  // don't keep processing on failure
    }
    if (listSize % 2 == 1)  // odd number last deposit was in this->buffer1
    {
      this->callStartCallback(this->buffer1, this->activeFilters.size() - 1);
      result = result && this->activeFilters.back()->update(this->buffer1, data_out);
    }
    else
    {
      this->callStartCallback(this->buffer0, this->activeFilters.size() - 1);
      result = result && this->activeFilters.back()->update(this->buffer0, data_out);
    }
    this->callFinishedCallback(data_out, this->activeFilters.size() - 1, result);
  }
  return result;
}

template <typename F>
void FilterChain<F>::callStartCallback(const F& data, const size_t filterNum)
{
  if (!this->filterStartCallback)
    return;
  // activeFilters mutex should be locked by the calling function
  const auto& filter = this->activeFilters[filterNum];
  this->filterStartCallback(data, filterNum, filter->getName(), filter->getType());
}

template <typename F>
void FilterChain<F>::callFinishedCallback(const F& data, const size_t filterNum, const bool succeeded)
{
  if (!this->filterFinishedCallback)
    return;
  // activeFilters mutex should be locked by the calling function
  const auto& filter = this->activeFilters[filterNum];
  this->filterFinishedCallback(data, filterNum, filter->getName(), filter->getType(), succeeded);
}

template<typename F>
void FilterChain<F>::disableFilter(const ::std::string& name)
{
  this->disabledFilters.insert(name);
  CRAS_DEBUG("Disabled filter %s", name.c_str());
  this->updateActiveFilters();
}

template<typename F>
void FilterChain<F>::enableFilter(const ::std::string& name)
{
  this->disabledFilters.erase(name);
  CRAS_DEBUG("Enabled filter %s", name.c_str());
  this->updateActiveFilters();
}

template<typename F>
void FilterChain<F>::updateActiveFilters()
{
  ::std::lock_guard<::std::mutex> lock(this->activeFiltersMutex);

  this->activeFilters.clear();
  for (const auto& filter : this->getFilters())
  {
    if (this->disabledFilters.find(filter->getName()) == this->disabledFilters.end())
      this->activeFilters.push_back(filter);
  }
}

template<typename F>
void FilterChain<F>::setDisabledFilters(::std::unordered_set<::std::string> filters)
{
  this->disabledFilters = ::std::move(filters);
  CRAS_DEBUG("Disabled filters updated. Currently disabled are: %s",
    this->disabledFilters.size() > 0 ? ::cras::join(this->disabledFilters, ", ").c_str() : "none");
  this->updateActiveFilters();
}

template<typename F>
::std::unordered_set<::std::string> FilterChain<F>::getDisabledFilters() const
{
  return this->disabledFilters;
}

template<typename F>
::std::vector<::std::shared_ptr<::filters::FilterBase<F>>> FilterChain<F>::getActiveFilters() const
{
  ::std::lock_guard<::std::mutex> lock(this->activeFiltersMutex);
  return this->activeFilters;
}

template<typename F>
bool FilterChain<F>::clear()
{
  const auto result = ::filters::FilterChain<F>::clear();
  this->disabledFilters.clear();
  {
    ::std::lock_guard<::std::mutex> lock(this->activeFiltersMutex);
    this->activeFilters.clear();
  }
  this->initialized = false;
  CRAS_DEBUG("Filters cleared");
  return result;
}

}
