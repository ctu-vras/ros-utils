#pragma once

#include <Eigen/Core>

namespace cras {

template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
inline std::string to_string(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& value)
{
  std::stringstream ss;
  ss << value.format({6, Eigen::DontAlignCols, ", ", "; ", "[", "]", "[", "]"});
  return ss.str();
}

}
