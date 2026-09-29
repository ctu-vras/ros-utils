#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport wrapper that allows using its modern API even on older distros.
 * \author Martin Pecka
 */

#include <functional>
#include <memory>
#include <string>

#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/transport_hints.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

#ifndef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include <image_transport/node_interfaces.hpp>
#else
#include <rclcpp/node.hpp>
#endif

namespace cras {

class ImageTransport : public ::image_transport::ImageTransport {
public:
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  using RequiredInterfaces = ::rclcpp::node_interfaces::NodeInterfaces<
      ::rclcpp::node_interfaces::NodeBaseInterface,
      ::rclcpp::node_interfaces::NodeParametersInterface,
      ::rclcpp::node_interfaces::NodeLoggingInterface,
      ::rclcpp::node_interfaces::NodeTimersInterface,
      ::rclcpp::node_interfaces::NodeTopicsInterface
  >;
#else
  using RequiredInterfaces = ::image_transport::RequiredInterfaces;
#endif

  /**
   * \param[in] required_interfaces Node interfaces used by the image transport.
   */
  explicit ImageTransport(::cras::ImageTransport::RequiredInterfaces required_interfaces);

  ~ImageTransport();

  /**
   * \brief Advertise the image topics for all registered transports.
   * \param[in] base_topic Name of the raw topic.
   * \param[in] custom_qos QoS of the image publishers.
   * \param[in] options Publisher options.
   * \return The publisher object that can be used to publish images.
   */
  ::image_transport::Publisher advertise(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos, ::rclcpp::PublisherOptions options = {});

  /**
   * \brief Advertise the camera info and image topics for all registered transports.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image publishers.
   * \param[in] options Publisher options.
   * \note This assumes the standard topic naming scheme, where the info topic is named "camera_info" in the same
   *       namespace as the base image topic.
   * \return The publisher object that can be used to publish images.
   */
  ::image_transport::CameraPublisher advertiseCamera(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos, ::rclcpp::PublisherOptions options = {});

  /**
   * \brief Subscribe to an image topic, version for arbitrary std::function object.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] callback The callback to be called with the decoded raw image.
   * \param[in] tracked_object The object whose lifetime should be tracked.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  ::image_transport::Subscriber subscribe(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      const ::image_transport::Subscriber::Callback& callback,
      const ::image_transport::ImageTransport::VoidPtr& tracked_object = ::image_transport::ImageTransport::VoidPtr(),
      const ::image_transport::TransportHints* transport_hints = nullptr, ::rclcpp::SubscriptionOptions options = {});

  /**
   * \brief Subscribe to an image topic, version for bare function.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] fp The callback to be called with the decoded raw image.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  ::image_transport::Subscriber subscribe(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      void (* fp)(const ::image_transport::ImageTransport::ImageConstPtr&),
      const ::image_transport::TransportHints* transport_hints = nullptr,
      const ::rclcpp::SubscriptionOptions options = {});

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   * \tparam T Type of the object whose member function is passed as callback.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] fp The callback to be called with the decoded raw image.
   * \param[in] obj The object whose member function the callback is.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  template<class T>
  ::image_transport::Subscriber subscribe(
      const ::std::string& base_topic, const ::rclcpp::QoS custom_qos,
      void (T::* fp)(const ::image_transport::ImageTransport::ImageConstPtr&), T* obj,
      const ::image_transport::TransportHints* transport_hints = nullptr,
      const ::rclcpp::SubscriptionOptions options = {}) {
    return ::cras::ImageTransport::subscribe(
      base_topic, custom_qos, ::std::bind(fp, obj, ::std::placeholders::_1), {}, transport_hints, options);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with shared_ptr.
   * \tparam T Type of the object whose member function is passed as callback.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] fp The callback to be called with the decoded raw image.
   * \param[in] obj The object whose member function the callback is.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  template<class T>
  ::image_transport::Subscriber subscribe(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      void (T::* fp)(const ::image_transport::ImageTransport::ImageConstPtr&), const ::std::shared_ptr<T>& obj,
      const ::image_transport::TransportHints* transport_hints = nullptr,
      const ::rclcpp::SubscriptionOptions options = {}) {
    return ::cras::ImageTransport::subscribe(
      base_topic, custom_qos, ::std::bind(fp, obj.get(), ::std::placeholders::_1), obj, transport_hints, options);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary std::function object.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] callback The callback to be called with the decoded raw image and camera info.
   * \param[in] tracked_object The object whose lifetime should be tracked.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \note This assumes the standard topic naming scheme, where the info topic is named "camera_info" in the same
   *       namespace as the base image topic.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  ::image_transport::CameraSubscriber subscribeCamera(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      const ::image_transport::CameraSubscriber::Callback& callback,
      const ::image_transport::ImageTransport::VoidPtr& tracked_object = {},
      const ::image_transport::TransportHints* transport_hints = nullptr);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] fp The callback to be called with the decoded raw image and camera info.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \note This assumes the standard topic naming scheme, where the info topic is named "camera_info" in the same
   *       namespace as the base image topic.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  ::image_transport::CameraSubscriber subscribeCamera(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      void (* fp)(
        const ::image_transport::ImageTransport::ImageConstPtr&,
        const ::image_transport::ImageTransport::CameraInfoConstPtr&),
      const ::image_transport::TransportHints* transport_hints = nullptr);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member function with bare
   *        pointer.
   * \tparam T Type of the object whose member function is passed as callback.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] fp The callback to be called with the decoded raw image and camera info.
   * \param[in] obj The object whose member function the callback is.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \note This assumes the standard topic naming scheme, where the info topic is named "camera_info" in the same
   *       namespace as the base image topic.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  template<class T>
  ::image_transport::CameraSubscriber subscribeCamera(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      void (T::* fp)(
        const ::image_transport::ImageTransport::ImageConstPtr&,
        const ::image_transport::ImageTransport::CameraInfoConstPtr&),
      T* obj, const ::image_transport::TransportHints* transport_hints = nullptr) {
    return ::cras::ImageTransport::subscribeCamera(
      base_topic, custom_qos, ::std::bind(fp, obj, ::std::placeholders::_1, ::std::placeholders::_2), {},
      transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member function with
   *        shared_ptr.
   * \tparam T Type of the object whose member function is passed as callback.
   * \param[in] base_topic Name of the raw image topic.
   * \param[in] custom_qos QoS of the image subscriber.
   * \param[in] fp The callback to be called with the decoded raw image and camera info.
   * \param[in] obj The object whose member function the callback is.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \note This assumes the standard topic naming scheme, where the info topic is named "camera_info" in the same
   *       namespace as the base image topic.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  template<class T>
  ::image_transport::CameraSubscriber subscribeCamera(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      void (T::* fp)(
        const ::image_transport::ImageTransport::ImageConstPtr&,
        const ::image_transport::ImageTransport::CameraInfoConstPtr&),
      const ::std::shared_ptr<T>& obj, const ::image_transport::TransportHints* transport_hints = nullptr) {
    return ::cras::ImageTransport::subscribeCamera(
      base_topic, custom_qos, ::std::bind(fp, obj.get(), ::std::placeholders::_1, ::std::placeholders::_2), obj,
      transport_hints);
  }

protected:
  ::cras::ImageTransport::RequiredInterfaces node_interfaces_;  //!< Node interfaces used by the image transport.

private:
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  /**
   * \brief Private constructor.
   * \note This constructor is used only on older distros to help constructing the new-API transport.
   * \param[in] node The node this transport works in.
   */
  explicit ImageTransport(const ::rclcpp::Node::SharedPtr& node);
#endif

  struct Impl;
  ::std::unique_ptr<::cras::ImageTransport::Impl> impl_;  //!< PIMPL
};

#ifndef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
using ImageTransportHints = ::image_transport::TransportHints;
#else
class ImageTransportHints : public ::image_transport::TransportHints {
public:
  /**
   * \brief Constructor.
   *
   * The default transport can be overridden by setting a certain parameter to the
   * name of the desired transport. By default this parameter is named "image_transport"
   * in the node's local namespace. For consistency across ROS applications, the
   * name of this parameter should not be changed without good reason.
   *
   * \param[in] node_interfaces Node interfaces to use when looking up the transport parameter.
   * \param[in] default_transport Preferred transport to use.
   * \param[in] parameter_name The name of the transport parameter.
   */
  explicit ImageTransportHints(
      ::cras::ImageTransport::RequiredInterfaces node_interfaces, const ::std::string& default_transport = "raw",
      const ::std::string& parameter_name = "image_transport");

  ~ImageTransportHints();

private:
  /**
   * \brief Private constructor.
   * \note This constructor is used only on older distros to help constructing the new-API transport hints.
   * \param[in] node The node this transport hints work in.
   * \param[in] default_transport Preferred transport to use.
   * \param[in] parameter_name The name of the transport parameter.
   */
  explicit ImageTransportHints(
      const ::rclcpp::Node::SharedPtr& node, const ::std::string& default_transport,
      const ::std::string& parameter_name);

  struct Impl;
  ::std::unique_ptr<::cras::ImageTransportHints::Impl> impl_;  //!< PIMPL
};
#endif

}  // namespace cras
