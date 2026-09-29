#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Pointcloud transport wrapper that allows using its modern API even on older distros.
 * \author Martin Pecka
 */

#include <functional>
#include <memory>
#include <string>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include <rclcpp/node.hpp>
#endif

namespace cras {

class PointCloudTransport : public ::point_cloud_transport::PointCloudTransport {
public:
  using RequiredInterfaces = ::rclcpp::node_interfaces::NodeInterfaces<
      ::rclcpp::node_interfaces::NodeBaseInterface,
      ::rclcpp::node_interfaces::NodeParametersInterface,
      ::rclcpp::node_interfaces::NodeLoggingInterface,
      ::rclcpp::node_interfaces::NodeTopicsInterface
  >;

  /**
   * \param[in] required_interfaces Node interfaces used by the point cloud transport.
   */
  explicit PointCloudTransport(::cras::PointCloudTransport::RequiredInterfaces required_interfaces);

  ~PointCloudTransport() override;

  /**
   * \brief Advertise the pointcloud topics for all registered transports.
   * \param[in] base_topic Name of the raw topic.
   * \param[in] custom_qos QoS of the pointcloud publishers.
   * \param[in] options Publisher options.
   * \return The publisher object that can be used to publish pointclouds.
   */
  ::point_cloud_transport::Publisher advertise(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      const ::rclcpp::PublisherOptions& options = ::rclcpp::PublisherOptions());

  /**
   * \brief Subscribe to a pointcloud topic, version for arbitrary std::function object.
   * \param[in] base_topic Name of the raw pointcloud topic.
   * \param[in] custom_qos QoS of the pointcloud subscriber.
   * \param[in] callback The callback to be called with the decoded raw pointcloud.
   * \param[in] tracked_object The object whose lifetime should be tracked (currently ignored).
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  ::point_cloud_transport::Subscriber subscribe(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      const ::point_cloud_transport::Subscriber::Callback& callback,
      const ::point_cloud_transport::PointCloudTransport::VoidPtr& tracked_object = {},
      const ::point_cloud_transport::TransportHints* transport_hints = nullptr,
      ::rclcpp::SubscriptionOptions options = ::rclcpp::SubscriptionOptions());

  /**
   * \brief Subscribe to a pointcloud topic, version for bare function.
   * \param[in] base_topic Name of the raw pointcloud topic.
   * \param[in] custom_qos QoS of the pointcloud subscriber.
   * \param[in] fp The callback to be called with the decoded raw pointcloud.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  ::point_cloud_transport::Subscriber subscribe(
      const ::std::string& base_topic, ::rclcpp::QoS custom_qos,
      void (* fp)(const ::sensor_msgs::msg::PointCloud2::ConstSharedPtr&),
      const ::point_cloud_transport::TransportHints* transport_hints = nullptr,
      ::rclcpp::SubscriptionOptions options = ::rclcpp::SubscriptionOptions());

  /**
   * \brief Subscribe to a pointcloud topic, version for class member function with bare function.
   * \tparam T Type of the object whose member function is passed as callback.
   * \param[in] base_topic Name of the raw pointcloud topic.
   * \param[in] custom_qos QoS of the pointcloud subscriber.
   * \param[in] fp The callback to be called with the decoded raw pointcloud.
   * \param[in] obj The object whose member function the callback is.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  template<class T>
  ::point_cloud_transport::Subscriber subscribe(
      const ::std::string& base_topic, const ::rclcpp::QoS custom_qos,
      void (T::* fp)(const ::sensor_msgs::msg::PointCloud2::ConstSharedPtr&) const, T* obj,
      const ::point_cloud_transport::TransportHints* transport_hints = nullptr,
      const ::rclcpp::SubscriptionOptions options = ::rclcpp::SubscriptionOptions()) {
    return subscribe(
      base_topic, custom_qos, ::std::bind(fp, obj, ::std::placeholders::_1), {}, transport_hints, options);
  }

  /**
   * \brief Subscribe to a pointcloud topic, version for class member function with shared_ptr.
   * \tparam T Type of the object whose member function is passed as callback.
   * \param[in] base_topic Name of the raw pointcloud topic.
   * \param[in] custom_qos QoS of the pointcloud subscriber.
   * \param[in] fp The callback to be called with the decoded raw pointcloud.
   * \param[in] obj The object whose member function the callback is.
   * \param[in] transport_hints Configuration that determines which transport topic will be subscribed.
   * \param[in] options Options for the created subscriber.
   * \return The subscriber object. The topic is subscribed as long as this object lives.
   */
  template<class T>
  ::point_cloud_transport::Subscriber subscribe(
      const ::std::string& base_topic, const ::rclcpp::QoS custom_qos,
      void (T::* fp)(const ::sensor_msgs::msg::PointCloud2::ConstSharedPtr&) const, const ::std::shared_ptr<T>& obj,
      const ::point_cloud_transport::TransportHints* transport_hints = nullptr,
      const ::rclcpp::SubscriptionOptions options = ::rclcpp::SubscriptionOptions()) {
    return subscribe(
      base_topic, custom_qos, ::std::bind(fp, obj, ::std::placeholders::_1), obj, transport_hints, options);
  }

protected:
  ::cras::PointCloudTransport::RequiredInterfaces node_interfaces_;  //!< Node interfaces used by the transport.

private:
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  /**
   * \brief Private constructor.
   * \note This constructor is used only on older distros to help constructing the new-API transport.
   * \param[in] node The node this transport works in.
   */
  explicit PointCloudTransport(rclcpp::Node::SharedPtr node);
#endif

  struct Impl;
  ::std::unique_ptr<Impl> impl_;  //!< PIMPL
};

class PointCloudTransportHints : public ::point_cloud_transport::TransportHints {
public:
  /**
   * \brief Constructor.
   *
   * The default transport can be overridden by setting a certain parameter to the
   * name of the desired transport. By default this parameter is named "point_cloud_transport"
   * in the node's local namespace. For consistency across ROS applications, the
   * name of this parameter should not be changed without good reason.
   *
   * \param[in] node_interfaces Node interfaces to use when looking up the transport parameter.
   * \param[in] default_transport Preferred transport to use.
   * \param[in] parameter_name The name of the transport parameter.
   */
  explicit PointCloudTransportHints(
      ::cras::PointCloudTransport::RequiredInterfaces node_interfaces, const ::std::string& default_transport = "raw",
      const ::std::string& parameter_name = "point_cloud_transport");

  ~PointCloudTransportHints();

private:
  struct Impl;
  ::std::unique_ptr<Impl> impl_;  //!< PIMPL
};

}  // namespace cras
