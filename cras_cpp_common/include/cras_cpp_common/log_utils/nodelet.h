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

#include <nodelet/nodelet.h>

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

  void printDebug(const ::std::string& text) const override
	{
		NODELET_DEBUG("%s", text.c_str());
	}
	
  void printInfo(const ::std::string& text) const override
	{
		NODELET_INFO("%s", text.c_str());
	}
	
  void printWarn(const ::std::string& text) const override
	{
		NODELET_WARN("%s", text.c_str());
	}
	
  void printError(const ::std::string& text) const override
	{
		NODELET_ERROR("%s", text.c_str());
	}
	
  void printFatal(const ::std::string& text) const override
	{
		NODELET_FATAL("%s", text.c_str());
	}

  //! \brief Function returning the name of the nodelet.
  GetNameFn getNameFn;
};

}