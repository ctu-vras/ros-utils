// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport wrapper that allows using its modern API even on older distros.
 * \author Martin Pecka
 */

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
// HACK: we need to access private method getTransportOrDefault()
#include <sstream>
#define private protected
#include <image_transport/image_transport.hpp>
#undef private
#else
#include <image_transport/image_transport.hpp>
#endif

#include <string>

#include <cras_cpp_common/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/transport_hints.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_options.hpp>
#endif

#include "impl/node_helper.hpp"

namespace cras {
struct ImageTransport::Impl {
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  rclcpp::Node::SharedPtr node;
#endif
};

ImageTransport::ImageTransport(const RequiredInterfaces required_interfaces) :
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
    ImageTransport(GetNodeSharedPtrFromInterfaces(required_interfaces))
#else
    image_transport::ImageTransport(required_interfaces), node_interfaces_(required_interfaces), impl_(new Impl)
#endif
{
}

ImageTransport::~ImageTransport() = default;

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
ImageTransport::ImageTransport(const rclcpp::Node::SharedPtr& node)
    : image_transport::ImageTransport(node), node_interfaces_(*node), impl_(new Impl) {
  impl_->node = node;
}
#endif

image_transport::Publisher ImageTransport::advertise(
    const std::string& base_topic, rclcpp::QoS custom_qos, rclcpp::PublisherOptions options) {
  const auto topics = node_interfaces_.get_node_topics_interface();
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  return image_transport::create_publisher(
    impl_->node.get(), topics->resolve_topic_name(base_topic), custom_qos.get_rmw_qos_profile(), options);
#else
  return image_transport::create_publisher(
    node_interfaces_, topics->resolve_topic_name(base_topic), custom_qos, options);
#endif
}

image_transport::CameraPublisher ImageTransport::advertiseCamera(
    const std::string& base_topic, rclcpp::QoS custom_qos, rclcpp::PublisherOptions options) {
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  return image_transport::create_camera_publisher(
    impl_->node.get(), base_topic, custom_qos.get_rmw_qos_profile(), options);
#else
  return image_transport::create_camera_publisher(node_interfaces_, base_topic, custom_qos, options);
#endif
}

#ifndef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
image_transport::Subscriber ImageTransport::subscribe(
    const std::string& base_topic, rclcpp::QoS custom_qos, const image_transport::Subscriber::Callback& callback,
    const VoidPtr& tracked_object, const image_transport::TransportHints* transport_hints,
    rclcpp::SubscriptionOptions options) {
  return image_transport::ImageTransport::subscribe(
    base_topic, custom_qos, callback, tracked_object, transport_hints, options);
}
#else
image_transport::Subscriber ImageTransport::subscribe(
    const std::string& base_topic, rclcpp::QoS custom_qos, const image_transport::Subscriber::Callback& callback,
    const VoidPtr& tracked_object, const image_transport::TransportHints* transport_hints,
    rclcpp::SubscriptionOptions options) {
  return image_transport::ImageTransport::subscribe(
    base_topic, custom_qos.get_rmw_qos_profile(), callback, tracked_object, transport_hints, options);
}

image_transport::Subscriber ImageTransport::subscribe(
    const std::string& base_topic, rclcpp::QoS custom_qos, void(* fp)(const ImageConstPtr&),
    const image_transport::TransportHints* transport_hints, const rclcpp::SubscriptionOptions options) {
  return subscribe(base_topic, custom_qos, image_transport::Subscriber::Callback(fp), {}, transport_hints, options);
}

image_transport::CameraSubscriber ImageTransport::subscribeCamera(
    const std::string& base_topic, rclcpp::QoS custom_qos, const image_transport::CameraSubscriber::Callback& callback,
    const image_transport::ImageTransport::VoidPtr& tracked_object,
    const image_transport::TransportHints* transport_hints) {
  return image_transport::create_camera_subscription(
    impl_->node.get(), base_topic, callback, getTransportOrDefault(transport_hints), custom_qos.get_rmw_qos_profile());
}

image_transport::CameraSubscriber ImageTransport::subscribeCamera(
    const std::string& base_topic, rclcpp::QoS custom_qos, void(* fp)(const ImageConstPtr&, const CameraInfoConstPtr&),
    const image_transport::TransportHints* transport_hints) {
  return subscribeCamera(base_topic, custom_qos, image_transport::CameraSubscriber::Callback(fp), {}, transport_hints);
}
#endif

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
struct ImageTransportHints::Impl {
  rclcpp::Node::SharedPtr node;
};

ImageTransportHints::ImageTransportHints(
    const cras::ImageTransport::RequiredInterfaces node_interfaces,
    const std::string& default_transport, const std::string& parameter_name)
    : cras::ImageTransportHints(GetNodeSharedPtrFromInterfaces(node_interfaces), default_transport, parameter_name) {}

ImageTransportHints::~ImageTransportHints() = default;

ImageTransportHints::ImageTransportHints(
    const rclcpp::Node::SharedPtr& node, const std::string& default_transport, const std::string& parameter_name)
    : image_transport::TransportHints(node.get(), default_transport, parameter_name), impl_(new Impl) {
  impl_->node = node;
}
#endif

}  // namespace cras
