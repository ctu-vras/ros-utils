# cras\_topic\_tools

Nodes and nodeletes for safe and efficient manipulation with topics.
Mostly supplementing what is missing from `topic_tools` and `nodelet_topic_tools`.

## Support and Build Status

This package is supported on Melodic and Noetic until their End of Life (and maybe later). It is occasionally tested with non-default GCC versions like Melodic+GCC8 or Noetic+GCC11.

Development versions: [![CI](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml)
[![Dev noetic ubuntu](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/badge/icon?subject=noetic+ubuntu)](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/)

Release jobs Noetic
[![Noetic version](https://img.shields.io/ros/v/noetic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-noetic-cras-cpp-common/):
[![Bin ubuntu noetic-amd64](https://build.ros.org/job/Nbin_uF64__cras_topic_tools__ubuntu_focal_amd64__binary/badge/icon?subject=focal+amd64)](https://build.ros.org/job/Nbin_uF64__cras_topic_tools__ubuntu_focal_amd64__binary/)
[![Bin ubuntu noetic-arm64](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_topic_tools__ubuntu_focal_arm64__binary/badge/icon?subject=focal+arm64)](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_topic_tools__ubuntu_focal_arm64__binary/)
[![Bin ubuntu noetic-armhf](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_topic_tools__ubuntu_focal_armhf__binary/badge/icon?subject=focal+armhf)](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_topic_tools__ubuntu_focal_armhf__binary/)

## Provided Nodes and Nodelets

Here is an overview of the provided nodes and nodelets. See their [API docs](https://docs.ros.org/en/api/cras_topic_tools/html/) for details.
Most of these nodes/nodelets are message-type-agnostic, but they may differ between messages that have or do not have a `std_msgs/Header header` field.

- `change_header`: Allows modifying headers of messages (frame ID, timestamp). [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1ChangeHeaderNodelet.html#details).
- `count_messages`: Count the number of received messages and make it available as a ROS parameter. [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1CountMessagesNodelet.html#details).
- `filter`: (only node, no nodelet) Filter messages on a topic based on a Python expression. [Usage](https://github.com/ctu-vras/ros-utils/blob/master/cras_topic_tools/nodes/filter).
- `heartbeat`: Publish `cras_msgs/Heartbeat` heartbeat of a topic. [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1HeartbeatNodelet.html#details).
- `mux_replay`: (only node, no nodelet) Control a `mux` with a recording of its `~selected` topic as if it were live. [Usage](https://github.com/ctu-vras/ros-utils/blob/master/cras_topic_tools/nodes/mux_replay).
- `priority_mux`: Inspired by `twist_mux`, provides a hierarchical way of switching output topics. Works for all message types and for multiple output topics. [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1PriorityMuxNodelet.html#details).
- `relay`: Simple relay with the possibility of lazy subscription. [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1RelayNodelet.html#details).
- `repeat`: Increase the frequency of a topic (with various trigger modes etc.). [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1RepeatMessagesNodelet.html#details).
- `throttle_messages`: Throttle down frequency of a topic - to an exact value, not using the imprecise algorithm from `topic_tools throttle`. [Usage](https://docs.ros.org/en/latest/api/cras_topic_tools/html/classcras_1_1ThrottleMessagesNodelet.html#details).

A few convenience libraries for working with generic topics are also provided:

- `shape_shifter`: Easily get the buffer or header of a `ShapeShifter` message or convert a message to `ShapeShifter`. Also contains a safe-to-use version of `ShapeShifter` for Melodic with fixed memory corruption problems.
- `lazy_subscriber`: Base class for all lazy subscribers.
- `generic_lazy_pub_sub`: Base implementation of a generic pair of publisher-subscriber of a type not known at compile type.
- `priority_mux_base`: Base implementation of `priority_mux` that can be used outside a nodelet.