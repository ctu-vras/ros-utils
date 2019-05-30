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

template <typename F>
bool FilterChain<F>::update(const F &data_in, F &data_out) {

  unsigned int list_size = this->reference_pointers_.size();
  bool result;
  if (list_size == 0)
  {
    data_out = data_in;
    result = true;
  }
  else if (list_size == 1)
  {
    result = this->reference_pointers_[0]->update(data_in, data_out);
    if (result) this->callCallback(data_out, 0);
  }
  else if (list_size == 2)
  {
    result = this->reference_pointers_[0]->update(data_in, this->buffer0_);
    if (result == false) {return false; };//don't keep processing on failure
    if (this->filterCallback)
      this->callCallback(this->buffer0_, 0);
    result = result && this->reference_pointers_[1]->update(this->buffer0_, data_out);
    if (result) this->callCallback(data_out, 1);
  }
  else
  {
    result = this->reference_pointers_[0]->update(data_in, this->buffer0_);  //first copy in
    if (result == false) {return false; }; //don't keep processing on failure
    this->callCallback(this->buffer0_, 0);
    for (unsigned int i = 1; i <  this->reference_pointers_.size() - 1; i++) // all but first and last (never called if size=2)
    {
      if (i %2 == 1) {
        result = result && this->reference_pointers_[i]->update(this->buffer0_, this->buffer1_);
        if (result) this->callCallback(this->buffer1_, i);
      }
      else {
        result = result && this->reference_pointers_[i]->update(this->buffer1_, this->buffer0_);
        if (result) this->callCallback(this->buffer0_, i);
      }

      if (result == false) {return false; }; //don't keep processing on failure
    }
    if (list_size % 2 == 1) // odd number last deposit was in this->buffer1
      result = result && this->reference_pointers_.back()->update(this->buffer1_, data_out);
    else
      result = result && this->reference_pointers_.back()->update(this->buffer0_, data_out);

    if (result) this->callCallback(data_out, this->reference_pointers_.size() - 1);
  }
  return result;
}

template <typename F>
void FilterChain<F>::callCallback(const F &data, size_t filterNum) {
  if (this->filterCallback)
    this->filterCallback(data, filterNum, this->reference_pointers_[filterNum]->getName(), this->reference_pointers_[filterNum]->getType());
}

template class FilterChain<sensor_msgs::LaserScan>;
template class FilterChain<sensor_msgs::PointCloud2>;

}