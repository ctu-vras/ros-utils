#ifndef CRAS_CPP_COMMON_NODELET_UTILS_HPP
#define CRAS_CPP_COMMON_NODELET_UTILS_HPP

#include <string>
#include <utility>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "cras_cpp_common/string_utils.hpp"

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

namespace cras {

/** \brief Make this class a public base of your Nodelet and it will allow you to call the getParam() helpers. */
class NodeletParamHelper
{
public:
  NodeletParamHelper() = default;
  // we need at least one virtual function so that we can use dynamic cast in implementation
  virtual ~NodeletParamHelper() = default;

protected:
  std::string defaultName = "NodeletParamHelper_has_to_be_a_sister_class_of_Nodelet";

  // we need to provide this for the NODELET_* logging macros to work
  const std::string& getName() const;

  /**
   * \brief Get the value of the given ROS parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with the loaded values.
   *
   * \tparam T Param type.
   * \param node The node handle to read the param value from.
   * \param name Name of the parameter.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
   * \return The loaded param value.
   */
  template <typename T>
  inline T getParam(ros::NodeHandle &node, const std::string &name,
                    const T &defaultValue = T(),
                    const std::string &unit = "") {
    T value;
    if (node.getParam(name, value)) {
      NODELET_INFO_STREAM(node.getNamespace()
                              << ": Found parameter: " << name
                              << ", value: " << cras::to_string(value)
                              << cras::prependIfNonEmpty(unit, " "));
      return value;
    } else {
      NODELET_WARN_STREAM(node.getNamespace()
                              << ": Cannot find value for parameter: " << name
                              << ", assigning default: "
                              << cras::to_string(defaultValue)
                              << cras::prependIfNonEmpty(unit, " "));
    }
    return defaultValue;
  }

  // std::string - char interop specializations

  inline std::string getParam(ros::NodeHandle &node,
                              const std::string &name,
                              const char *defaultValue,
                              const std::string &unit = "") {
    return this->getParam<std::string>(node, name, std::string(defaultValue),
                                       unit);
  }

private:
  template <typename Result, typename Param>
  inline Result getParamUnsigned(ros::NodeHandle &node,
                                 const std::string &name,
                                 const Result &defaultValue,
                                 const std::string &unit = "") {
    const Param signedValue =
        this->getParam(node, name, static_cast<Param>(defaultValue), unit);
    if (signedValue < 0) {
      NODELET_ERROR_STREAM(node.getNamespace() << ": Value " << signedValue
                                               << " of unsigned parameter "
                                               << name << " is negative.");
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signedValue);
  }

  // generic casting getParam()
  template <typename Result, typename Param>
  inline Result
  getParamCast(ros::NodeHandle &node, const std::string &name,
               const Param &defaultValue, const std::string &unit = "") {
    const Param paramValue = this->getParam(node, name, defaultValue, unit);
    return Result(paramValue);
  }
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

private:
  struct NodeletWithDiagnosticsPrivate;
  std::unique_ptr<NodeletWithDiagnosticsPrivate> data;
};

/** \brief Base template which adds all defined mixins to BaseNodelet class. */
template <typename BaseNodelet>
class NodeletBase : public BaseNodelet, public NodeletParamHelper, public StatefulNodelet,
    public ThreadNameUpdatingNodelet, public NodeletWithSharedTfBuffer, public NodeletWithDiagnostics
{
};

/** A convenient base class for all nodelets. */
class Nodelet : public NodeletBase<::nodelet::Nodelet> {

  // To disambiguate between nodelet::Nodelet::getName() and NodeletParamHelper::getName().
  protected: using ::nodelet::Nodelet::getName;

};

// getParam specializations for unsigned values

template<> inline uint64_t
NodeletParamHelper::getParam(ros::NodeHandle &node, const std::string &name,
                  const uint64_t &defaultValue, const std::string &unit) {
  return this->getParamUnsigned<uint64_t, int>(node, name, defaultValue, unit);
}

template<> inline unsigned int
NodeletParamHelper::getParam(ros::NodeHandle &node, const std::string &name,
                  const unsigned int &defaultValue, const std::string &unit) {
  return this->getParamUnsigned<unsigned int, int>(node, name, defaultValue, unit);
}

// ROS types specializations

template<> inline ros::Duration
NodeletParamHelper::getParam(ros::NodeHandle &node, const std::string &name,
                  const ros::Duration &defaultValue, const std::string &unit) {
  return this->getParamCast<ros::Duration, double>(node, name, defaultValue.toSec(), unit);
}

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

#endif //CRAS_CPP_COMMON_NODELET_UTILS_HPP