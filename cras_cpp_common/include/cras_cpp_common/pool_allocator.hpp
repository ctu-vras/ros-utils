#pragma once

#include <boost/pool/pool_alloc.hpp>

namespace cras
{

template <typename T, class... Args>
boost::shared_ptr<T> make_shared_from_fast_pool(Args && ... args) {
  using allocator_t = boost::fast_pool_allocator<boost::shared_ptr<T>>;
  return boost::allocate_shared<T, allocator_t, Args...>(
    allocator_t(), boost::detail::sp_forward<Args>( args )...);
}

template <typename T, class... Args>
boost::shared_ptr<T> make_shared_from_pool(Args && ... args) {
  using allocator_t = boost::pool_allocator<boost::shared_ptr<T>>;
  return boost::allocate_shared<T, allocator_t, Args...>(
    allocator_t(), boost::detail::sp_forward<Args>( args )...);
}

}