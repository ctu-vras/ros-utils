#pragma once

/**
 * \file
 * \brief Helper for allocating boost::shared_ptr instances in a boost shared pool. This should be useful if you publish
 *        shared_ptr messages and want to save some time during allocation of the new messages. The default pool size
 *        is 32 messages. Note that when publishing messages containing arrays, these functions only handle allocation
 *        of the message itself. Allocation of the array elements is done via the allocator specified in the message
 *        type.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <boost/make_shared.hpp>
#include <boost/pool/pool_alloc.hpp>

namespace cras
{

/**
 * \brief Make a shared instance of an object from Boost fast pool (suitable for e.g. elements of a std::list or single
 *        object allocations).
 * \tparam T Type of the object to allocate.
 * \tparam Args Constructor argument types.
 * \param[in] args Constructor arguments.
 * \return The shared instance.
 * \note Usage in ROS code:
 *   <code>msg = cras::make_shared_from_fast_pool&lt;sensor_msgs::PointCloud2&gt;();</code>
 */
template <typename T, class... Args>
::boost::shared_ptr<T> make_shared_from_fast_pool(Args && ... args)
{
  using allocator_t = ::boost::fast_pool_allocator<::boost::shared_ptr<T>>;
  return ::boost::allocate_shared<T, allocator_t, Args...>(allocator_t(), ::boost::detail::sp_forward<Args>( args )...);
}

/**
 * \brief Make a shared instance of an object from Boost pool (suitable for e.g. elements of a std::vector or
 *        allocations of multiple objects at once).
 * \tparam T Type of the object to allocate.
 * \tparam Args Constructor argument types.
 * \param[in] args Constructor arguments.
 * \return The shared instance.
 * \note Usage in ROS code:
 *   <code>msg = cras::make_shared_from_pool&lt;sensor_msgs::PointCloud2&gt;();</code>
 */
template <typename T, class... Args>
::boost::shared_ptr<T> make_shared_from_pool(Args && ... args)
{
  using allocator_t = ::boost::pool_allocator<::boost::shared_ptr<T>>;
  return ::boost::allocate_shared<T, allocator_t, Args...>(allocator_t(), ::boost::detail::sp_forward<Args>( args )...);
}

}