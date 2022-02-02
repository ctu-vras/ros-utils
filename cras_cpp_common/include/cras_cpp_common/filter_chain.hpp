#pragma once

/**
 * \file
 * \brief Helpers for working with filter chains based on filters::FilterChain.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/common.h>
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_base.hpp>
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_base.h>
#include <filters/filter_chain.h>
#endif

#include <nodelet/nodelet.h>

namespace cras
{

/**
 * This filter chain implementation allows for selectively disabling/enabling filters during run time.
 * It also adds a callback with the result of each filter run, so that you can e.g. publish the output of each filter.
 * @tparam F Type of the filtered data.
 */
template<typename F>
class FilterChain : public ::filters::FilterChain<F>
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
  typedef std::function<
    void(const F& data, const size_t filterNum, const ::std::string& name, const ::std::string& type)> FilterCallback;

  /**
   * \brief Construct a filter chain.
   * \param dataType Textual representation of the data type.
   * \param filterCallback Optional callback to be called after each filter finishes its work.
   */
  explicit FilterChain(const ::std::string& dataType, const FilterCallback& filterCallback = {});
  
  /**
   * \brief Inform this chain that it is running in the given nodelet, so that it can do appropriate optimizations.
   *        This should be done after the filters are configured.
   * \param[in] nodelet The nodelet this chain is running in.
   *                    Setting to nullptr will inform the chain it is not running inside a nodelet. 
   */
  void setNodelet(const ::nodelet::Nodelet* nodelet);
  
  /**
   * \brief Set the filter callback.
   * \param callback The callback to set.
   */
  void setFilterCallback(const FilterCallback& callback);
  
  /**
   * \brief Do the filtering.
   * This function intentionally shadows filters::FilterChain::update() which is non-virtual.
   * \param[in] data_in Input data. 
   * \param[out] data_out The filtered data.
   * \return Whether the filtering succeeded. If false, data_out should not be considered valid.
   */
  bool update(const F& data_in, F& data_out);
  
  /**
   * \brief Temporarily disable the filter with the given name.
   * \param[in] name Name of the filter to disable. If the filter is not found, nothing happens.
   */
  void disableFilter(const ::std::string& name);

  /**
   * \brief Enable the temporarily disabled filter with the given name.
   * \param[in] name Name of the filter to enable. If the filter is not found, nothing happens.
   */
  void enableFilter(const ::std::string& name);
  
  /**
   * \brief Set which filters are temporarily disabled. This overrides any previous calls to
   *        disableFilter() and enableFilter().
   * \param[in] filters The filters to disable. 
   */
  void setDisabledFilters(::std::unordered_set<::std::string> filters);
  
  /**
   * \brief Clear all filters from this chain.
   * This function intentionally shadows filters::FilterChain::clear() which is non-virtual.
   * \return Always true.
   */
  bool clear();

protected:
  /**
   * \brief If filterCallback is set, call it.
   * \param data The filtered data to pass to the callback.
   * \param filterNum Number of the filter to pass to the callback.
   */
  void callCallback(const F& data, size_t filterNum);

  /**
   * \brief Update the contents of activeFilters with just the filters that have not been disabled.
   */
  void updateActiveFilters();
  
  /**
   * \brief Get the list of filters configured for this chain.
   * \return The filters.
   */
  ::std::vector<::boost::shared_ptr<::filters::FilterBase<F>>>& getFilters();

  /**
   * \brief Get the list of filters configured for this chain.
   * \return The filters.
   */
  const ::std::vector<::boost::shared_ptr<::filters::FilterBase<F>>>& getFilters() const; 

  //! \brief The optional callback to call when a filter finishes its work.
  FilterCallback filterCallback;
  
  //! \brief A set of filters that have been temporarily disabled.
  ::std::unordered_set<::std::string> disabledFilters;
  
  //! \brief A list of filters that should be treated as active and should act on the input data.
  ::std::vector<::boost::shared_ptr<::filters::FilterBase<F>>> activeFilters;
  
  //! \brief Mutex protecting activeFilters access.
  ::std::mutex activeFiltersMutex;

  //! \brief Whether this filter chain has been initialized (gets set by first `update()` and cleared by `clear()`).
  bool initialized {false};

  // Parent class buffers are private, so we create our own

  //! \brief A temporary intermediate buffer
  F buffer0;

  //! \brief A temporary intermediate buffer
  F buffer1;
};

}

#include <cras_cpp_common/impl/filter_chain.hpp>