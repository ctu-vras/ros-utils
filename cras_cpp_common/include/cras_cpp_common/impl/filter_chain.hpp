#pragma once

/**
 * \file
 * \brief Definitions of cras::FilterChain methods.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <rosconsole/macros_generated.h>

#include <cras_cpp_common/filter_chain.hpp>
#include <cras_cpp_common/filter_base.hpp>

namespace cras
{

template<typename F>
FilterChain<F>::FilterChain(
  const ::std::string& dataType, const FilterChain::FilterCallback& filterCallback)
  : ::filters::FilterChain<F>(dataType), filterCallback(filterCallback)
{
}

template <typename F>
void FilterChain<F>::setNodelet(const ::nodelet::Nodelet* nodelet)
{
  for (const auto& filter : this->getFilters())
  {
    if (::boost::dynamic_pointer_cast<::cras::FilterBase<F>>(filter))
    {
      ::boost::dynamic_pointer_cast<::cras::FilterBase<F>>(filter)->setNodelet(nodelet);
    }
  }
}

template <typename F>
bool FilterChain<F>::update(const F &data_in, F &data_out)
{

  if (this->activeFilters.size() + this->disabledFilters.size() != this->getFilters().size())
    this->updateActiveFilters();

  size_t listSize = this->activeFilters.size();
  bool result;

  if (listSize == 0)
  {
    data_out = data_in;
    result = true;
  }
  else if (listSize == 1)
  {
    result = this->activeFilters[0]->update(data_in, data_out);
    if (result) this->callCallback(data_out, 0);
  }
  else if (listSize == 2)
  {
    result = this->activeFilters[0]->update(data_in, this->buffer0);
    if (result == false) return false;  //don't keep processing on failure
    if (this->filterCallback) this->callCallback(this->buffer0, 0);
    result = result && this->activeFilters[1]->update(this->buffer0, data_out);
    if (result) this->callCallback(data_out, 1);
  }
  else
  {
    result = this->activeFilters[0]->update(data_in, this->buffer0);  //first copy in
    if (result == false) return false;  //don't keep processing on failure

    this->callCallback(this->buffer0, 0);
    // all but first and last (never called if size=2)
    for (size_t i = 1; i <  this->activeFilters.size() - 1; i++)
    {
      if (i % 2 == 1)
      {
        result = result && this->activeFilters[i]->update(this->buffer0, this->buffer1);
        if (result) this->callCallback(this->buffer1, i);
      }
      else
      {
        result = result && this->activeFilters[i]->update(this->buffer1, this->buffer0);
        if (result) this->callCallback(this->buffer0, i);
      }

      if (result == false) return false;  //don't keep processing on failure
    }
    if (listSize % 2 == 1) // odd number last deposit was in this->buffer1
      result = result && this->activeFilters.back()->update(this->buffer1, data_out);
    else
      result = result && this->activeFilters.back()->update(this->buffer0, data_out);

    if (result) this->callCallback(data_out, this->activeFilters.size() - 1);
  }
  return result;
}

template <typename F>
void FilterChain<F>::callCallback(const F &data, size_t filterNum)
{
  const auto& filter = this->getFilters()[filterNum];
  if (this->filterCallback)
    this->filterCallback(data, filterNum, filter->getName(), filter->getType());
}

template<typename F>
void FilterChain<F>::disableFilter(const ::std::string &name)
{
  this->disabledFilters.insert(name);
  ROS_DEBUG("Disabled filter %s", name.c_str());
}

template<typename F>
void FilterChain<F>::enableFilter(const ::std::string &name)
{
  this->disabledFilters.erase(name);
  ROS_DEBUG("Enabled filter %s", name.c_str());
}

template<typename F>
void FilterChain<F>::updateActiveFilters()
{
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
}

}