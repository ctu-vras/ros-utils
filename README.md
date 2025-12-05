<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# ros-utils

<img src="ctu-fee.jpg" alt="CTU-CRAS-Norlab team on DARPA SubT Challenge" width="200" />

Various ROS utilities and building blocks developed by [Center for Robotics and Autonomous Systems (CRAS)](https://robotics.fel.cvut.cz/cras/) at [Faculty of Electrical Engineering](https://fel.cvut.cz/) of [Czech Technical University in Prague](https://www.cvut.cz).

Parts of this package were used by [team CTU-CRAS-Norlab in DARPA Subterranean Challenge](https://robotics.fel.cvut.cz/cras/darpa-subt/).

* `cras_bag_tools`: utilities for working with BAG files, including filters and image extractor
* `cras_cpp_common`: convenience utilities for ROS programming in C++
* ~~`cras_docs_common`: utilities for setting up documentation generators~~ (not yet available in ROS 2)
* `cras_lint`: common linter configurations for CRAS packages
* ~~`cras_py_common`: convenience utilities for ROS programming in Python~~ (not yet available in ROS 2)
* ~~`cras_topic_tools`: nodes and nodeletes for safe and efficient manipulation with topics~~ (not yet available in ROS 2)
* ~~`image_transport_codecs`: image\_transport converted to C++ and Python libraries directly usable even without a running ROS system~~ (not yet available in ROS 2)

## Support and Build Status

This package is always tested for architectures amd64 and arm64.

This package is supported on ROS 1 **Melodic** and **Noetic** (on branch `master`) even after their End of Life.

This package is supported on ROS 2 **Jazzy** and **Klited** (on branch `ros2`). Humble and Foxy are not and will not be supported.

### Development versions

ROS 1: [![CI ROS 1](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml)

ROS 2: [![CI ROS 2](https://github.com/ctu-vras/ros-utils/actions/workflows/ci-ros2.yaml/badge.svg?branch=ros2)](https://github.com/ctu-vras/ros-utils/actions/workflows/ci-ros2.yaml)

### Binary ROS Releases

[![Melodic version](https://img.shields.io/ros/v/melodic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-cras-cpp-common/)
[![Noetic version](https://img.shields.io/ros/v/noetic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-noetic-cras-cpp-common/)

<img src="ctu-cras-norlab-team.jpg" alt="CTU-CRAS-Norlab team on DARPA SubT Challenge" width="75%" />
