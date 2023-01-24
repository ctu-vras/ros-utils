# cras\_topic\_tools

Nodes and nodeletes for safe and efficient manipulation with topics.
Mostly supplementing what is missing from `topic_tools` and `nodelet_topic_tools`.

## Support and Build Status

This package is supported on Melodic and Noetic until their End of Life (and maybe later). It is occasionally tested with non-default GCC versions like Melodic+GCC8 or Noetic+GCC11.

Development versions: [![CI](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml)
[![Dev melodic](https://build.ros.org/job/Mdev__cras_ros_utils__ubuntu_bionic_amd64/badge/icon?subject=melodic+ubuntu)](https://build.ros.org/job/Mdev__cras_ros_utils__ubuntu_bionic_amd64/)
[![Dev noetic ubuntu](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/badge/icon?subject=noetic+ubuntu)](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/)
[![Dev noetic debian](https://build.ros.org/job/Ndev_db__cras_ros_utils__debian_buster_amd64/badge/icon?subject=noetic+debian)](https://build.ros.org/job/Ndev_db__cras_ros_utils__debian_buster_amd64/)

Release jobs Melodic
[![Melodic version](https://img.shields.io/ros/v/melodic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-cras-cpp-common/):
[![Bin melodic-amd64](https://build.ros.org/job/Mbin_uB64__cras_topic_tools__ubuntu_bionic_amd64__binary/badge/icon?subject=bionic+amd64)](https://build.ros.org/job/Mbin_uB64__cras_topic_tools__ubuntu_bionic_amd64__binary/)
[![Bin melodic-arm64](https://build.ros.org/job/Mbin_ubv8_uBv8__cras_topic_tools__ubuntu_bionic_arm64__binary/badge/icon?subject=bionic+arm64)](https://build.ros.org/job/Mbin_ubv8_uBv8__cras_topic_tools__ubuntu_bionic_arm64__binary/)
[![Bin melodic-armhf](https://build.ros.org/job/Mbin_ubhf_uBhf__cras_topic_tools__ubuntu_bionic_armhf__binary/badge/icon?subject=bionic+armhf)](https://build.ros.org/job/Mbin_ubhf_uBhf__cras_topic_tools__ubuntu_bionic_armhf__binary/)

Release jobs Noetic
[![Noetic version](https://img.shields.io/ros/v/noetic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-noetic-cras-cpp-common/):
[![Bin ubuntu noetic-amd64](https://build.ros.org/job/Nbin_uF64__cras_topic_tools__ubuntu_focal_amd64__binary/badge/icon?subject=focal+amd64)](https://build.ros.org/job/Nbin_uF64__cras_topic_tools__ubuntu_focal_amd64__binary/)
[![Bin ubuntu noetic-arm64](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_topic_tools__ubuntu_focal_arm64__binary/badge/icon?subject=focal+arm64)](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_topic_tools__ubuntu_focal_arm64__binary/)
[![Bin ubuntu noetic-armhf](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_topic_tools__ubuntu_focal_armhf__binary/badge/icon?subject=focal+armhf)](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_topic_tools__ubuntu_focal_armhf__binary/)
[![Bin debian noetic-amd64](https://build.ros.org/job/Nbin_db_dB64__cras_topic_tools__debian_buster_amd64__binary/badge/icon?subject=buster+amd64)](https://build.ros.org/job/Nbin_db_dB64__cras_topic_tools__debian_buster_amd64__binary/)
[![Bin debian noetic-arm64](https://build.ros.org/job/Nbin_dbv8_dBv8__cras_topic_tools__debian_buster_arm64__binary/badge/icon?subject=buster+arm64)](https://build.ros.org/job/Nbin_dbv8_dBv8__cras_topic_tools__debian_buster_arm64__binary/)

## Provided Nodes and Nodelets

Here is an overview of the provided nodes and nodelets. See their [API docs](https://docs.ros.org/en/api/cras_topic_tools/html/) for details.
Most of these nodes/nodelets are message-type-agnostic, but they may differ between messages that have or do not have a `std_msgs/Header header` field.

- `change_header`: Allows modifying headers of messages (frame ID, timestamp).
- `count_messages`: Count the number of received messages and make it available as a ROS parameter.
- `filter`: (only node, no nodelet) Filter messages on a topic based on a Python expression.
- `mux_replay`: (only node, no nodelet) Control a `mux` with a recording of its `~selected` topic as if it were live.
- `priority_mux`: Inspired by `twist_mux`, provides a hierarchical way of switching output topics. Works for all message types and for multiple output topics.
- `relay`: Simple relay with the possibility of lazy subscription.
- `repeat`: Increase the frequency of a topic (with various trigger modes etc.).
- `throttle_messages`: Throttle down frequency of a topic - to an exact value, not using the imprecise algorithm from `topic_tools throttle`.

A few convenience libraries for working with generic topics are also provided:

- `shape_shifter`: Easily get the buffer or header of a `ShapeShifter` message or convert a message to `ShapeShifter`. Also contains a safe-to-use version of `ShapeShifter` for Melodic with fixed memory corruption problems.
- `lazy_subscriber`: Base class for all lazy subscribers.
- `generic_lazy_pub_sub`: Base implementation of a generic pair of publisher-subscriber of a type not known at compile type.