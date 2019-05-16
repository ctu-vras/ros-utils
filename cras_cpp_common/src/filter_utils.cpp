#include <sstream>  // has to be there, otherwise we encounter build problems
#define private protected
#include <filters/filter_chain.h>
#undef private

#include <cras_cpp_common/filter_utils.hpp>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace cras
{

template <typename F>
void FilterChain<F>::setNodelet(const nodelet::Nodelet* nodelet) {
  for (const auto& filter : this->reference_pointers_) {
    if (boost::dynamic_pointer_cast<cras::FilterBase<F>>(filter)) {
      boost::dynamic_pointer_cast<cras::FilterBase<F>>(filter)->setNodelet(nodelet);
    }
  }
}

template class FilterChain<sensor_msgs::LaserScan>;
template class FilterChain<sensor_msgs::PointCloud2>;

}