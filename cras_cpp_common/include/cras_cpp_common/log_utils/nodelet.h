#pragma once

/**
 * \file
 * \brief Log helper redirecting the logging calls to NODELET_ macros.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <string>

#include <cras_cpp_common/log_utils.h>

namespace cras
{
/**
 * Log helper delegating the logging calls to the NODELET_ macros.
 */
  class NodeletLogHelper : public ::cras::LogHelper
{
public:
  
  //! \brief Type of the function returning the name of the nodelet.
  typedef ::std::function<const ::std::string&()> GetNameFn;
  
  /**
   * Create the log helper reporting as the nodelet of name returned by getNameFn.
   * @param getNameFn A function returning the name of the nodelet.
   */
  explicit NodeletLogHelper(const GetNameFn& getNameFn);
  ~NodeletLogHelper() override;

protected:
  /**
   * \brief Used by the NODELET_ logging macros.
   * \return Name of the bound nodelet.
   */
  const ::std::string& getName() const;

  void printDebug(const ::std::string& text) const override;
  void printInfo(const ::std::string& text) const override;
  void printWarn(const ::std::string& text) const override;
  void printError(const ::std::string& text) const override;
  void printFatal(const ::std::string& text) const override;

  //! \brief Function returning the name of the nodelet.
  GetNameFn getNameFn;
};

}