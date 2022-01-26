#pragma once

/**
 * \brief This file contains a set of classes that make work with nodelets easier.
 * \param[in] node The node to bind to.
 * \param[in] ns If nonempty, returns just the parameters in the given namespace.
 * \return The bound param helper.
 * 
 * \details
 * First, there are a few helper "mixins", which are not derived from Nodelet, but are expected to be its sister classes.
 *
 * NodeletParamHelper provides the conveniently templatized getParam() methods similar to those in node_utils.
 *
 * StatefulNodelet adds state to the nodelets, so that you can query isOk() to determine whether the nodelet is not
 * shutting down.
 *
 * ThreadNameUpdatingNodelet adds function updateThreadName() which you can call in your callbacks to signal to the OS
 * which nodelet is currently being run in the particular nodelet manager's thread.
 *
 * The NodeletSharingTfBuffer adds functions setBuffer(), getBuffer() and usesSharedBuffer(), which allow the nodelet
 * manager to pass a pointer to a shared TF buffer. Note that standard nodelet manager cannot make use of this feature,
 * so to utilize it, you should launch nodelet_manager_sharing_tf_buffer from this package instead.
 *
 * Finally, NodeletBase<BaseNodelet> template class adds all these mixins to the provided BaseNodelet class to create
 * a nodelet where you can use all the features added by the mentioned mixins. Use this template if you need to
 * derive your nodelet from a different class than nodelet::Nodelet (like PCLNodelet).
 *
 * If you just want to build your nodelet from scratch, you should start by deriving it from the cras::Nodelet class.
 *
 * NodeletAwareTFBuffer is a class that is able to correctly end canTransform() calls in case the nodelet is asked to
 * unload while the canTransform() call is waiting.
 */

#include <string>
#include <utility>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/log_utils/nodelet.h>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>

#define NODELET_DEBUG_DELAYED_THROTTLE(rate, ...) ROS_DEBUG_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_INFO_DELAYED_THROTTLE(rate, ...) ROS_INFO_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_WARN_DELAYED_THROTTLE(rate, ...) ROS_WARN_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_ERROR_DELAYED_THROTTLE(rate, ...) ROS_ERROR_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)

#define NODELET_DEBUG_STREAM_DELAYED_THROTTLE(rate, ...) ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM_DELAYED_THROTTLE(rate, ...) ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM_DELAYED_THROTTLE(rate, ...) ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_DELAYED_THROTTLE(rate, ...) ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)

namespace cras {

/**
 * \brief Make this class a public base of your Nodelet and it will allow you to call the getParam() helpers.
 */
class NodeletParamHelper : public ::cras::ParamHelper
{
public:
  NodeletParamHelper() :
    ::cras::ParamHelper(::std::make_shared<::cras::NodeletLogHelper>(::std::bind(&NodeletParamHelper::getName, this)))
  {
  }

  // we need at least one virtual function so that we can use dynamic cast in implementation
  ~NodeletParamHelper() override = default;

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
   *                         If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamServerType = ResultType,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const ::cras::optional<ResultType>& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParamVerbose(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  template<typename ResultType, typename ParamServerType = ResultType,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ::cras::GetParamResult<ResultType> getParamVerbose(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const ResultType& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParamVerbose(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
   *                         If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamServerType = ResultType,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ResultType getParam(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const ::cras::optional<ResultType>& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParam(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \tparam ResultType Param type (the C++ type). It is converted from the intermediate ParamServerType
   *                    using options.toResult function (which defaults to static_cast).
   * \tparam ParamServerType Intermediate type to which the XmlRpcValue read from parameter server is converted. The
   *                         conversion is done using options.toParam function (which defaults to cras::convert). Most
   *                         overloads of cras::convert are in xmlrpc_value_utils.hpp, but you can add your own.
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  template<typename ResultType, typename ParamServerType = ResultType,
    ::cras::check_get_param_types<ResultType, ParamServerType>* = nullptr>
  inline ResultType getParam(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const ResultType& defaultValue = ResultType(),
    const ::std::string& unit = "",
    const ::cras::GetParamOptions<ResultType, ParamServerType>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParam(param, name, defaultValue, unit, options);
  }

  // std::string - char interop specializations

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
   *                         If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  inline ::cras::GetParamResult<::std::string> getParamVerbose(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const ::cras::optional<const char*>& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParamVerbose(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return A wrapper containing the loaded parameter value and details about the function execution.
   */
  inline ::cras::GetParamResult<::std::string> getParamVerbose(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const char* defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParamVerbose(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value (if not nullopt),
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use. If std::nullopt, then the parameter is required.
   *                         If a required param is not found, a GetParamException is thrown.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  inline ::std::string getParam(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const ::cras::optional<const char*>& defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParam(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Get the value of the given ROS parameter, falling back to the specified default value,
   *        and print out a ROS log message with the loaded values (if specified).
   * \details This is a variant allowing use of C-string instead of std::string. 
   * \param[in] node The node handle from which parameters are read.
   * \param[in] name Name of the parameter.
   * \param[in] defaultValue The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages
   *                 more informative.
   * \param[in] options Options specifying detailed behavior of this function. Use the braced initializer syntax for
   *                    comfortable writing, e.g. `{.throwIfConvertFails = true, .allowNestedParams = false}`.
   * \return The loaded parameter value.
   */
  inline ::std::string getParam(
    const ::ros::NodeHandle& node, const ::std::string& name,
    const char* defaultValue, const ::std::string& unit = "",
    const ::cras::GetParamOptions<::std::string>& options = {})
  {
    const auto param = ::cras::NodeHandleGetParamAdapter(node);
    return ::cras::ParamHelper::getParam(param, name, defaultValue, unit, options);
  }

  /**
   * \brief Creates a version of this param helper "bound" to the given node handle, so that it is not needed to
   *        specify the node handle in the subsequent getParam() calls.
   * \param node[in] The node to bind to.
   * \param ns[in] If nonempty, returns just the parameters in the given namespace.
   * \return The bound param helper.
   */
  ::cras::BoundParamHelperPtr params(::ros::NodeHandle& node, const ::std::string& ns = "") const;

  /**
   * \brief Creates a version of this param helper "bound" to the private nodelet parameters, so that it is not needed
   *        to specify the node handle in the subsequent getParam() calls.
   * \param ns[in] If nonempty, returns just the parameters in the given namespace.
   * \return The bound param helper.
   */
  ::cras::BoundParamHelperPtr privateParams(const ::std::string& ns = "") const;

  /**
   * \brief Creates a version of this param helper "bound" to the public nodelet parameters, so that it is not needed to
   *        specify the node handle in the subsequent getParam() calls.
   * \param ns[in] If nonempty, returns just the parameters in the given namespace.
   * \return The bound param helper.
   */
  ::cras::BoundParamHelperPtr publicParams(const ::std::string& ns = "") const;

  /**
   * \brief Creates a version of this param helper "bound" to the given node handle, so that it is not needed to
   *        specify the node handle in the subsequent getParam() calls.
   * \param node[in] The node to bind to.
   * \return The bound param helper.
   */
  ::cras::BoundParamHelperPtr paramsForNodeHandle(::ros::NodeHandle& node);

protected:
  //! \brief This name is used for NodeletParamHelper instances that are wrongly not a sister class of nodelet::Nodelet. 
  ::std::string defaultName = "NodeletParamHelper_has_to_be_a_sister_class_of_Nodelet";

  /**
   * \brief This function is passed to the logging helper.
   * \return Name of the sister nodelet.
   */
  const ::std::string& getName() const;

};

/** \brief This is a nodelet which can tell you when it is being unloaded. */
class StatefulNodelet
{
public:
  virtual ~StatefulNodelet();

  /**
   * \brief Similar to ros::ok(). Becomes false when nodelet unload is requested.
   * \return Whether it is ok to continue nodelet operations.
   */
  bool ok() const;

protected:
  void shutdown();

private:
  bool isUnloading = false;
};

/** \brief A nodelet that provides a function to update the OS name of the thread it is currently executing in. */
class ThreadNameUpdatingNodelet
{
public:
  // we need at least one virtual function so that we can use dynamic cast in implementation
  virtual ~ThreadNameUpdatingNodelet() = default;

protected:
  /**
   * \brief Set custom name of the current thread to this nodelet's name.
   *
   * \note The name will be automatically shortened if longer than 15 chars.
   * \note You can see the custom names in htop when you enable display of
   *       custom thread names in options.
   * \note This function doesn't reset the name back to the original.
   * \note You should call this function at the beginning of all your callbacks.
   */
  void updateThreadName() const;
};

struct NodeletWithSharedTfBuffer
{
public:
  NodeletWithSharedTfBuffer();
  virtual ~NodeletWithSharedTfBuffer();  // we need to be polymorphic
  void setBuffer(const std::shared_ptr<tf2_ros::Buffer>& buffer);
  tf2_ros::Buffer& getBuffer() const;
  bool usesSharedBuffer() const;

private:
  const std::string& getName() const;
  struct NodeletWithSharedTfBufferPrivate;
  std::unique_ptr<NodeletWithSharedTfBufferPrivate> data;
};

struct NodeletWithDiagnostics
{
public:
  NodeletWithDiagnostics();
  virtual ~NodeletWithDiagnostics();  // we need to be polymorphic
  diagnostic_updater::Updater& getDiagUpdater() const;
  void startDiagTimer(ros::NodeHandle& nh) const;
  void stopDiagTimer() const;

private:
  struct NodeletWithDiagnosticsPrivate;
  std::unique_ptr<NodeletWithDiagnosticsPrivate> data;
};

/** \brief Base template which adds all defined mixins to BaseNodelet class. */
template <typename BaseNodelet>
class NodeletBase : public BaseNodelet, public NodeletParamHelper, public StatefulNodelet,
    public ThreadNameUpdatingNodelet, public NodeletWithSharedTfBuffer, public NodeletWithDiagnostics
{
protected:
  template<typename T>
  typename std::enable_if<!ros::message_traits::HasHeader<T>::value, std::unique_ptr<HeaderlessDiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ros::NodeHandle nh, const std::string& topic, size_t queueSize, const std::string& paramNamespace,
    const ros::Rate& defaultRate, const ros::Rate& defaultMinRate, const ros::Rate& defaultMaxRate)
  {
    return std::make_unique<HeaderlessDiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->paramsForNodeHandle(this->getPrivateNodeHandle())->paramsInNamespace(paramNamespace),
      defaultRate, defaultMinRate, defaultMaxRate);
  }

  template<typename T>
  typename std::enable_if<!ros::message_traits::HasHeader<T>::value, std::unique_ptr<HeaderlessDiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ros::NodeHandle nh, const std::string& topic, size_t queueSize, const std::string& paramNamespace,
    const ros::Rate& defaultRate)
  {
    return std::make_unique<HeaderlessDiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->paramsForNodeHandle(this->getPrivateNodeHandle())->paramsInNamespace(paramNamespace),
      defaultRate);
  }

  template<typename T>
  typename std::enable_if<!ros::message_traits::HasHeader<T>::value, std::unique_ptr<HeaderlessDiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ros::NodeHandle nh, const std::string& topic, size_t queueSize, const std::string& paramNamespace)
  {
    return std::make_unique<HeaderlessDiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->paramsForNodeHandle(this->getPrivateNodeHandle())->paramsInNamespace(paramNamespace));
  }

  template<typename T>
  typename std::enable_if<ros::message_traits::HasHeader<T>::value, std::unique_ptr<DiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ros::NodeHandle nh, const std::string& topic, size_t queueSize, const std::string& paramNamespace,
    const ros::Rate& defaultRate, const ros::Rate& defaultMinRate, const ros::Rate& defaultMaxRate)
  {
    return std::make_unique<DiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->paramsForNodeHandle(this->getPrivateNodeHandle())->paramsInNamespace(paramNamespace),
      defaultRate, defaultMinRate, defaultMaxRate);
  }

  template<typename T>
  typename std::enable_if<ros::message_traits::HasHeader<T>::value, std::unique_ptr<DiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ros::NodeHandle nh, const std::string& topic, size_t queueSize, const std::string& paramNamespace,
    const ros::Rate& defaultRate)
  {
    return std::make_unique<DiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->paramsForNodeHandle(this->getPrivateNodeHandle())->paramsInNamespace(paramNamespace),
      defaultRate);
  }

  template<typename T>
  typename std::enable_if<ros::message_traits::HasHeader<T>::value, std::unique_ptr<DiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ros::NodeHandle nh, const std::string& topic, size_t queueSize, const std::string& paramNamespace)
  {
    return std::make_unique<DiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->paramsForNodeHandle(this->getPrivateNodeHandle())->paramsInNamespace(paramNamespace));
  }
};

/** A convenient base class for all nodelets. */
class Nodelet : public NodeletBase<::nodelet::Nodelet> {

  // To disambiguate between nodelet::Nodelet::getName() and NodeletParamHelper::getName().
  protected: using ::nodelet::Nodelet::getName;

};

/** Provides overrides of canTransform() that correctly end when the nodelet is asked to unload. */
class NodeletAwareTFBuffer : public tf2_ros::Buffer {
public:
  explicit NodeletAwareTFBuffer(const StatefulNodelet* nodelet);

  bool canTransform(const std::string& target_frame,
      const std::string& source_frame, const ros::Time& time,
      ros::Duration timeout, std::string *errstr) const override;

  bool canTransform(const std::string &target_frame, const ros::Time &target_time,
      const std::string &source_frame, const ros::Time &source_time,
      const std::string &fixed_frame, ros::Duration timeout,
      std::string *errstr) const override;

protected:
  const StatefulNodelet* nodelet;
};

}
