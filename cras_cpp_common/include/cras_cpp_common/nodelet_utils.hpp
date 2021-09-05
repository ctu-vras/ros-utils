#pragma once

#include <string>
#include <utility>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils.hpp>

/**
 * This file contains a set of classes that make work with nodelets easier.
 *
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
 * Log helper delegating the logging calls to the NODELET_ macros.
 */
class NodeletLogHelper : public LogHelper
{
public:
  /**
   * Create the log helper reporting as the nodelet of name returned by getNameFn.
   * @param getNameFn A function returning the name of the nodelet.
   */
  NodeletLogHelper(const std::function<const std::string&()> &getNameFn) : getNameFn(getNameFn) {}
  virtual ~NodeletLogHelper() = default;

protected:
  /**
   * Used by the NODELET_ logging macros.
   * @return Name of the bound nodelet.
   */
  const std::string& getName() const { return getNameFn(); }

  void printDebug(const std::string &text) const override { NODELET_DEBUG("%s", text.c_str()); }
  void printInfo(const std::string &text) const override { NODELET_INFO("%s", text.c_str()); }
  void printWarn(const std::string &text) const override { NODELET_WARN("%s", text.c_str()); }
  void printError(const std::string &text) const override { NODELET_ERROR("%s", text.c_str()); }
  void printFatal(const std::string &text) const override { NODELET_FATAL("%s", text.c_str()); }

  std::function<const std::string&()> getNameFn; //!< Function returning the name of the nodelet.
};

/** \brief Make this class a public base of your Nodelet and it will allow you to call the getParam() helpers. */
class NodeletParamHelper : public ParamHelper
{
public:
  NodeletParamHelper() :
    ParamHelper(std::make_shared<NodeletLogHelper>(std::bind(&NodeletParamHelper::getName, this)))
  {
  }

  // we need at least one virtual function so that we can use dynamic cast in implementation
  virtual ~NodeletParamHelper() = default;

  template<typename T>
  inline T getParam(ros::NodeHandle& nh, const std::string& name, const T& defaultValue = T(),
                    const std::string& unit = "") const {
    const auto param = cras::NodeRawGetParamAdapter(nh);
    return ParamHelper::getParam(param, name, defaultValue, unit);
  }

  inline std::string getParam(ros::NodeHandle& nh, const std::string &name, const char *defaultValue,
                              const std::string &unit = "") const {
    const auto param = cras::NodeRawGetParamAdapter(nh);
    return ParamHelper::getParam(param, name, defaultValue, unit);
  }

  /**
   * Creates a version of this param helper "bound" to the given node handle, so that it is not needed to specify the
   * node handle in the subsequent getParam calls.
   * @param node The node to bind to.
   * @return The bound param helper.
   */
  inline BoundParamHelperPtr paramsForNodeHandle(ros::NodeHandle& node) const {
    const auto param = std::make_shared<NodeRawGetParamAdapter>(node);
    return std::make_shared<BoundParamHelper>(this->log, param);
  }

protected:
  std::string defaultName = "NodeletParamHelper_has_to_be_a_sister_class_of_Nodelet";

  /**
   * This function is passed to the logging helper.
   * @return Name of the nodelet.
   */
  const std::string& getName() const;

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
