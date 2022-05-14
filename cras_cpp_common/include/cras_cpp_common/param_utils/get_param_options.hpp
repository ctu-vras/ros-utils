#pragma once

/**
 * \file
 * \brief Options for getParam() calls.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <functional>
#include <list>
#include <string>

#include <XmlRpcValue.h>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>

namespace cras
{

/**
 * \brief Default function for converting param server types to result types.
 * \tparam ResultType Type of getParam() result.
 * \tparam ParamServerType Type of the intermediate value to which the XmlRpcValue param is converted.
 * \note Create a specialization of this struct in case you need a different implementation of the conversion.
 */
template<typename ResultType, typename ParamServerType>
struct DefaultToResultFn
{
	/**
   * \brief Function converting ParamServerType values to ResultType.
   * \param[in] value The value to convert.
   * \return The converted value.
   * \throws std::runtime_error If the conversion failed.
   */
	static ResultType toResult(const ParamServerType& value){ return static_cast<ResultType>(value); };
};

/**
 * \brief Default ParamServerType for the given ResultType.
 * \tparam ResultType Type of the getParam() result.
 * \note Create a specialization of this struct in case you want to use a different intermediate ParamServerType for
 *       reading values of type ResultType.
 */
template<typename ResultType>
struct DefaultParamServerType
{
	//! \brief The ParamServerType to be used as intermediate type when reading values of type ResultType.
	typedef ResultType type;
};

/**
 * \brief Options specifying behavior of getParam() calls.
 * \note For easy usage in getParam() calls, use the braced initializer syntax, e.g. `{.printMessages = false}`.
 * \tparam ResultType Type of getParam() result.
 * \tparam ParamServerType Type of the intermediate value to which the XmlRpcValue should be converted.
 */
template<typename ResultType, typename ParamServerType = typename ::cras::DefaultParamServerType<ResultType>::type> 
struct GetParamOptions
{
  /**
   * \brief Function converting ParamServerType values to ResultType.
   * \param[in] value The value to convert.
   * \return The converted value.
   * \throws std::runtime_error If the conversion failed.
   */
  typedef ::std::function<ResultType(const ParamServerType& value)> ToResultFn;

  /**
   * \brief Function converting XmlRpcValue to an intermediate value of type ParamServerType.
   * \param[in] xmlValue The XmlRpcValue read from parameter server.
   * \param[out] value The converted value. It is not valid if this function returns false.
   * \param[in] skipNonConvertible If true and the target value is a container, all non-convertible items will be
   *                               skipped. If false, non-convertible values mean failure of the whole conversion.
   *                               If all items of a container are skipped (and there were some),
   *                               the conversion also fails.
   * \param[out] errors If non-null, any error messages coming from the conversion can be appended to this list.
   * \return Whether the conversion succeeded.
   * \note This function should not throw.
   */
  typedef ::std::function<bool(
      const ::XmlRpc::XmlRpcValue& xmlValue,
      ParamServerType& value,
      const bool skipNonConvertible,
      ::std::list<::std::string>* errors
    )> ToParamFn;
  
  //! \brief Whether to print error messages to log.
  bool printMessages {true};
  
  //! \brief Whether defaulted parameters are reported as warning or info level messages.
  bool printDefaultAsWarn {false};
  
  //! \brief Throw GetParamException if any conversion fails. If false, the default value is used instead of a value
  //! that failed to convert. In such case, the log message is of error level.
  bool throwIfConvertFails {false};
  
  //! \brief Allow parameters of form a/b/c (leads to recursive parsing of the sub-namespaces).
  bool allowNestedParams {true};
  
  //! \brief The human-friendly namespace to be reported in log messages.
  ::std::string origNamespace {};
  
  //! \brief The human-friendly parameter name to be reported in log messages.
  ::std::string origParamName {};
  
  //! \brief A function that converts ParamServerType values to string for use in log messages.
  ::cras::ToStringFn<ParamServerType> paramToStr = [](const ParamServerType& s){ return ::cras::to_string(s); };

  //! \brief A function that converts ResultType values to string for use in log messages.
  ::cras::ToStringFn<ResultType> resultToStr = [](const ResultType& s){ return ::cras::to_string(s); };

  //! \brief A function converting ParamServerType values to ResultType.
  ToResultFn toResult = &::cras::DefaultToResultFn<ResultType, ParamServerType>::toResult;
  
  //! \brief A function converting XmlRpcValue to an intermediate value of type ParamServerType.
  ToParamFn toParam = [](
      const ::XmlRpc::XmlRpcValue& x,
      ParamServerType& v,
      bool skipNonConvertible = false,
      ::std::list<::std::string>* errors = nullptr
    ) -> bool
		{
      return ::cras::convert(x, v, skipNonConvertible, errors);
    };
  
  /**
   * \brief Assign from options of a different type. Only the non-function members are copied!
   * \tparam R1 Other ResultType.
   * \tparam P1 Other ParamServerType.
   * \param other The options to copy from.
   * \return This.
   */
  template<typename R1, typename P1>
  GetParamOptions& operator=(const ::cras::GetParamOptions<R1, P1>& other)
  {
    this->printMessages = other.printMessages;
    this->printDefaultAsWarn = other.printDefaultAsWarn;
    this->throwIfConvertFails = other.throwIfConvertFails;
    this->allowNestedParams = other.allowNestedParams;
    this->origNamespace = other.origNamespace;
    this->origParamName = other.origParamName;
    return *this;
  }

  /**
   * \brief Assign from options of the same type. Non-function members are also copied.
   * \param other The options to copy from.
   * \return This.
   */
  GetParamOptions& operator=(const ::cras::GetParamOptions<ResultType, ParamServerType>& other)
  {
    this->printMessages = other.printMessages;
    this->printDefaultAsWarn = other.printDefaultAsWarn;
    this->throwIfConvertFails = other.throwIfConvertFails;
    this->allowNestedParams = other.allowNestedParams;
    this->origNamespace = other.origNamespace;
    this->origParamName = other.origParamName;
    this->paramToStr = other.paramToStr;
    this->resultToStr = other.resultToStr;
    this->toResult = other.toResult;
    this->toParam = other.toParam;
    return *this;
  }
  
  /**
   * \brief Convert this options object to a similar object with a different ParamServerType.
   * \tparam NewParamServerType The new ParamServerType.
   * \param[in] newToResult New toResult function.
   * \param[in] newParamToStr New paramToStr function.
   * \param[in] newToParam New toParam function.
   * \return The new options object.
   */
  template<typename NewParamServerType>
  ::cras::GetParamOptions<ResultType, NewParamServerType> asType(
    typename ::cras::GetParamOptions<ResultType, NewParamServerType>::ToResultFn newToResult =
			&::cras::DefaultToResultFn<ResultType, NewParamServerType>::toResult,
    ::cras::ToStringFn<NewParamServerType> newParamToStr =
			[](const NewParamServerType& s){ return ::cras::to_string(s); },
    typename ::cras::GetParamOptions<ResultType, NewParamServerType>::ToParamFn newToParam =
      [](const ::XmlRpc::XmlRpcValue& x, NewParamServerType& v, bool skipNonConvertible = false,
          ::std::list<::std::string>* errors = nullptr) -> bool {
        return ::cras::convert(x, v, skipNonConvertible, errors);
      }
    ) const
  {
    // We can't initialize the object with the default values of toParam and toResult because they are ill-formed
    ::cras::GetParamOptions<ResultType, NewParamServerType> options =
			{{}, {}, {}, {}, {}, {}, {}, {}, newToResult, newToParam};
    options = *this;
    options.resultToStr = this->resultToStr;
    options.paramToStr = newParamToStr;
    return options;
  }
};

}