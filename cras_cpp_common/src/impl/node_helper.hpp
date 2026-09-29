// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief This is a shim for ROS Kilted and older which hacks in a way to get a shared_ptr on Node from a raw pointer or
 *        node interfaces. This is needed for image/pointcloud transport and other libraries that use NodeInterfaces
 *        in Lyrical and newer, but use Node* or NodeSharedPtr in older distros.
 */

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>

namespace cras {

using ImageTransportInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeTimersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface
>;

using PointCloudTransportInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeTopicsInterface
>;

rclcpp::Node::SharedPtr GetNodeSharedPtrFromRawPtr(rclcpp::Node* node);

rclcpp::Node::SharedPtr GetNodeSharedPtrFromInterfaces(ImageTransportInterfaces node_interfaces);

rclcpp::Node::SharedPtr GetNodeSharedPtrFromInterfaces(PointCloudTransportInterfaces node_interfaces);

}  // namespace cras
