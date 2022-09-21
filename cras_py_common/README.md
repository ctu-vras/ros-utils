# cras\_py\_common

A Czech-army knife for ROS code written in Python.

The aim of this package is to provide some missing utility functions to rospy. Using libraries from this package, you should be able to write more efficient, safer and less error-prone code without much boilerplate. As this package is intended to handle a lot of the boring code for you "behind the scenes", ubiquitous effort was put into unit-testing everything, so that you can rely on the provided code without the usual fear whether it works or not.

This readme shortly introduces the provided libraries. Detailed documentation can be found in the comments in code and in the [API docs](https://docs.ros.org/en/api/cras_py_common/html/). Examples of usage can be found in the dependent packages from `ros-utils`, and in the unit tests.

Parts of this package were used by [team CTU-CRAS-Norlab in DARPA Subterranean Challenge](https://robotics.fel.cvut.cz/cras/darpa-subt/).


## Support and Build Status

This package is supported on Melodic and Noetic until their End of Life (and maybe later). It is occasionally tested with non-default GCC versions like Melodic+GCC8 or Noetic+GCC11.

Development versions: [![CI](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml)
[![Dev melodic](https://build.ros.org/job/Mdev__cras_ros_utils__ubuntu_bionic_amd64/badge/icon?subject=melodic+ubuntu)](https://build.ros.org/job/Mdev__cras_ros_utils__ubuntu_bionic_amd64/)
[![Dev noetic ubuntu](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/badge/icon?subject=noetic+ubuntu)](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/)
[![Dev noetic debian](https://build.ros.org/job/Ndev_db__cras_ros_utils__debian_buster_amd64/badge/icon?subject=noetic+debian)](https://build.ros.org/job/Ndev_db__cras_ros_utils__debian_buster_amd64/)

Release jobs Melodic
[![Melodic version](https://img.shields.io/ros/v/melodic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-cras-cpp-common/):
[![Bin melodic-amd64](https://build.ros.org/job/Mbin_uB64__cras_py_common__ubuntu_bionic_amd64__binary/badge/icon?subject=bionic+amd64)](https://build.ros.org/job/Mbin_uB64__cras_py_common__ubuntu_bionic_amd64__binary/)
[![Bin melodic-arm64](https://build.ros.org/job/Mbin_ubv8_uBv8__cras_py_common__ubuntu_bionic_arm64__binary/badge/icon?subject=bionic+arm64)](https://build.ros.org/job/Mbin_ubv8_uBv8__cras_py_common__ubuntu_bionic_arm64__binary/)
[![Bin melodic-armhf](https://build.ros.org/job/Mbin_ubhf_uBhf__cras_py_common__ubuntu_bionic_armhf__binary/badge/icon?subject=bionic+armhf)](https://build.ros.org/job/Mbin_ubhf_uBhf__cras_py_common__ubuntu_bionic_armhf__binary/)

Release jobs Noetic
[![Noetic version](https://img.shields.io/ros/v/noetic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-noetic-cras-cpp-common/):
[![Bin ubuntu noetic-amd64](https://build.ros.org/job/Nbin_uF64__cras_py_common__ubuntu_focal_amd64__binary/badge/icon?subject=focal+amd64)](https://build.ros.org/job/Nbin_uF64__cras_py_common__ubuntu_focal_amd64__binary/)
[![Bin ubuntu noetic-arm64](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_py_common__ubuntu_focal_arm64__binary/badge/icon?subject=focal+arm64)](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_py_common__ubuntu_focal_arm64__binary/)
[![Bin ubuntu noetic-armhf](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_py_common__ubuntu_focal_armhf__binary/badge/icon?subject=focal+armhf)](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_py_common__ubuntu_focal_armhf__binary/)
[![Bin debian noetic-amd64](https://build.ros.org/job/Nbin_db_dB64__cras_py_common__debian_buster_amd64__binary/badge/icon?subject=buster+amd64)](https://build.ros.org/job/Nbin_db_dB64__cras_py_common__debian_buster_amd64__binary/)
[![Bin debian noetic-arm64](https://build.ros.org/job/Nbin_dbv8_dBv8__cras_py_common__debian_buster_arm64__binary/badge/icon?subject=buster+arm64)](https://build.ros.org/job/Nbin_dbv8_dBv8__cras_py_common__debian_buster_arm64__binary/)
