#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Log helper redirecting the logging calls to NODELET_ macros.
 * \author Martin Pecka
 */

#include <functional>
#include <string>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/nodelet_utils/log_macros.h>

namespace cras
{

/**
 * Log helper delegating the logging calls to the NODELET_ macros.
 */
class NodeletLogHelper : public ::cras::RosconsoleLogHelper
{
public:
  //! \brief Type of the function returning the name of the nodelet.
  typedef ::std::function<const ::std::string&()> GetNameFn;

  /**
   * Create the log helper reporting as the nodelet of name returned by getNameFn.
   * @param getNameFn A function returning the name of the nodelet.
   */
  explicit NodeletLogHelper(const GetNameFn& getNameFn);

  void initializeLogLocation(::ros::console::LogLocation* loc, const ::std::string& name,
    ::ros::console::Level level) const override;

protected:
  //! \brief Function returning the name of the nodelet.
  GetNameFn getNameFn;
};

}
