// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Pointcloud transport wrapper that allows using its modern API even on older distros.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <cras_cpp_common/point_cloud_transport.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include <rclcpp/node.hpp>
#endif

#include "impl/node_helper.hpp"

namespace cras {

struct PointCloudTransport::Impl {
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  rclcpp::Node::SharedPtr node;
#endif
};

PointCloudTransport::PointCloudTransport(cras::PointCloudTransport::RequiredInterfaces required_interfaces) :
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
    cras::PointCloudTransport(GetNodeSharedPtrFromInterfaces(required_interfaces))
#else
    point_cloud_transport::PointCloudTransport(required_interfaces), node_interfaces_(required_interfaces),
    impl_(new Impl())
#endif
{
}

PointCloudTransport::~PointCloudTransport() = default;

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
PointCloudTransport::PointCloudTransport(const rclcpp::Node::SharedPtr node)
    : point_cloud_transport::PointCloudTransport(node), node_interfaces_(*node), impl_(new Impl()) {
  impl_->node = node;
}
#endif

point_cloud_transport::Publisher PointCloudTransport::advertise(
    const std::string& base_topic, const rclcpp::QoS custom_qos, const rclcpp::PublisherOptions& options) {
  const auto topics = node_interfaces_.get_node_topics_interface();

#ifdef POINT_CLOUD_TRANSPORT_NODE_DOES_NOT_REMAP_TOPICS
  const auto topic = topics->resolve_topic_name(base_topic);
#else
  const auto& topic = base_topic;
#endif

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  return point_cloud_transport::create_publisher(impl_->node, topic, custom_qos.get_rmw_qos_profile(), options);
#else
  return point_cloud_transport::create_publisher(node_interfaces_, topic, custom_qos, options);
#endif
}

point_cloud_transport::Subscriber PointCloudTransport::subscribe(
    const std::string& base_topic, const rclcpp::QoS custom_qos,
    const point_cloud_transport::Subscriber::Callback& callback,
    const point_cloud_transport::PointCloudTransport::VoidPtr&,
    const point_cloud_transport::TransportHints* transport_hints, const rclcpp::SubscriptionOptions options) {
  const auto transport = getTransportOrDefault(transport_hints);
  const auto topics = node_interfaces_.get_node_topics_interface();

#ifdef POINT_CLOUD_TRANSPORT_NODE_DOES_NOT_REMAP_TOPICS
  const auto topic = topics->resolve_topic_name(base_topic);
#else
  const auto& topic = base_topic;
#endif

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  return point_cloud_transport::create_subscription(
    impl_->node, topic, callback, transport, custom_qos.get_rmw_qos_profile(), options);
#else
  return point_cloud_transport::create_subscription(
    node_interfaces_, topic, callback, transport, custom_qos, options);
#endif
}

point_cloud_transport::Subscriber PointCloudTransport::subscribe(
    const std::string& base_topic, const rclcpp::QoS custom_qos,
    void(* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&),
    const point_cloud_transport::TransportHints* transport_hints, const rclcpp::SubscriptionOptions options) {
  return subscribe(
    base_topic, custom_qos, point_cloud_transport::Subscriber::Callback(fp), {}, transport_hints, options);
}

struct PointCloudTransportHints::Impl {};

PointCloudTransportHints::PointCloudTransportHints(
    cras::PointCloudTransport::RequiredInterfaces required_interfaces, const std::string& default_transport,
    const std::string& parameter_name) :
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
    point_cloud_transport::TransportHints(
        GetNodeSharedPtrFromInterfaces(required_interfaces), default_transport, parameter_name),
    impl_(new Impl())
#else
    point_cloud_transport::TransportHints(required_interfaces, default_transport, parameter_name), impl_(new Impl())
#endif
{
}

PointCloudTransportHints::~PointCloudTransportHints() = default;

}  // namespace cras
