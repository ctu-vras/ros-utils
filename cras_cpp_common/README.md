# cras\_cpp\_common

A Czech-army knife for ROS code written in C++.

The aim of this package is to provide some missing utility functions to roscpp. Using libraries from this package, you should be able to write more efficient, safer and less error-prone code without much boilerplate. As this package is intended to handle a lot of the boring code for you "behind the scenes", ubiquitous effort was put into unit-testing everything, so that you can rely on the provided code without the usual fear whether it works or not.

This readme shortly introduces the provided libraries. Detailed documentation can be found in the comments in code and in the [API docs](https://docs.ros.org/en/api/cras_cpp_common/html/). Examples of usage can be found in the dependent packages from `ros-utils`, and in the unit tests.

Parts of this package were used by [team CTU-CRAS-Norlab in DARPA Subterranean Challenge](https://robotics.fel.cvut.cz/cras/darpa-subt/).

## Support and Build Status

This package is supported on Melodic and Noetic until their End of Life (and maybe later). It is occasionally tested with non-default GCC versions like Melodic+GCC8 or Noetic+GCC11.

Development versions: [![CI](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/ros-utils/actions/workflows/ci.yaml)
[![Dev melodic](https://build.ros.org/job/Mdev__cras_ros_utils__ubuntu_bionic_amd64/badge/icon?subject=melodic+ubuntu)](https://build.ros.org/job/Mdev__cras_ros_utils__ubuntu_bionic_amd64/)
[![Dev noetic ubuntu](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/badge/icon?subject=noetic+ubuntu)](https://build.ros.org/job/Ndev__cras_ros_utils__ubuntu_focal_amd64/)
[![Dev noetic debian](https://build.ros.org/job/Ndev_db__cras_ros_utils__debian_buster_amd64/badge/icon?subject=noetic+debian)](https://build.ros.org/job/Ndev_db__cras_ros_utils__debian_buster_amd64/)

Release jobs Melodic
[![Melodic version](https://img.shields.io/ros/v/melodic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-cras-cpp-common/):
[![Bin melodic-amd64](https://build.ros.org/job/Mbin_uB64__cras_cpp_common__ubuntu_bionic_amd64__binary/badge/icon?subject=bionic+amd64)](https://build.ros.org/job/Mbin_uB64__cras_cpp_common__ubuntu_bionic_amd64__binary/)
[![Bin melodic-arm64](https://build.ros.org/job/Mbin_ubv8_uBv8__cras_cpp_common__ubuntu_bionic_arm64__binary/badge/icon?subject=bionic+arm64)](https://build.ros.org/job/Mbin_ubv8_uBv8__cras_cpp_common__ubuntu_bionic_arm64__binary/)
[![Bin melodic-armhf](https://build.ros.org/job/Mbin_ubhf_uBhf__cras_cpp_common__ubuntu_bionic_armhf__binary/badge/icon?subject=bionic+armhf)](https://build.ros.org/job/Mbin_ubhf_uBhf__cras_cpp_common__ubuntu_bionic_armhf__binary/)

Release jobs Noetic
[![Noetic version](https://img.shields.io/ros/v/noetic/cras_ros_utils)](http://packages.ros.org/ros/ubuntu/pool/main/r/ros-noetic-cras-cpp-common/):
[![Bin ubuntu noetic-amd64](https://build.ros.org/job/Nbin_uF64__cras_cpp_common__ubuntu_focal_amd64__binary/badge/icon?subject=focal+amd64)](https://build.ros.org/job/Nbin_uF64__cras_cpp_common__ubuntu_focal_amd64__binary/)
[![Bin ubuntu noetic-arm64](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_cpp_common__ubuntu_focal_arm64__binary/badge/icon?subject=focal+arm64)](https://build.ros.org/job/Nbin_ufv8_uFv8__cras_cpp_common__ubuntu_focal_arm64__binary/)
[![Bin ubuntu noetic-armhf](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_cpp_common__ubuntu_focal_armhf__binary/badge/icon?subject=focal+armhf)](https://build.ros.org/job/Nbin_ufhf_uFhf__cras_cpp_common__ubuntu_focal_armhf__binary/)
[![Bin debian noetic-amd64](https://build.ros.org/job/Nbin_db_dB64__cras_cpp_common__debian_buster_amd64__binary/badge/icon?subject=buster+amd64)](https://build.ros.org/job/Nbin_db_dB64__cras_cpp_common__debian_buster_amd64__binary/)
[![Bin debian noetic-arm64](https://build.ros.org/job/Nbin_dbv8_dBv8__cras_cpp_common__debian_buster_arm64__binary/badge/icon?subject=buster+arm64)](https://build.ros.org/job/Nbin_dbv8_dBv8__cras_cpp_common__debian_buster_arm64__binary/)

## List of provided libraries

- `any`: Provides forward compatibility for [`std::any`](https://en.cppreference.com/w/cpp/utility/any).
- `c_api`: Utilities for writing a C API for your packages.
- `cloud`, `tf2_sensor_msgs`: Utilities for working with pointclouds (iterators, copying, extracting channels, transforming the clouds).
- `diag_utils`: Helpers for easy setup of a diagnosed publisher/subscriber that checks message rate or delay. Configuration of the expected rates/delays is via ROS parameters.
- `expected`: Provides forward compatibility for [`std::expected`](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2017/p0323r3.pdf).
- `filter_utils`:
  - `FilterBase` eases access to filter parameters via `param_utils`.
  - `FilterChain` class and nodelet improve upon [`filters::FilterChain`](https://github.com/ros/filters/blob/noetic-devel/include/filters/filter_chain.hpp) by adding support for dynamic disabling/enabling of individual filters, diagnostics of the individual filters and possibility to publish the intermediate filtering results.
- `functional`: Provides forward compatibility for [`std::apply()`](https://en.cppreference.com/w/cpp/utility/apply), [`std::invoke()`](https://en.cppreference.com/w/cpp/utility/functional/invoke) and [`std::bind_front()`](https://en.cppreference.com/w/cpp/utility/functional/bind_front). Especially `cras::bind_front()` is super useful for specifying ROS subscriber callbacks, where you just bind `this` to the callback, and the rest of the parameters is automatically handled.
- `log_utils`: Unified access to the `ROS_*` and `NODELET_*` logging macros. This is more an internal utility of this package.
- `math_utils`: `sgn()` signum function, and `RunningStats` (computes mean and variance on a stream of data in O(1) time per sample).
- `message_utils`: `BaseMessage<M>` and `IsMessageParam<M>` template helpers for working with ROS message classes.
- `node_utils`: Integration of `diag_utils` and `param_utils` for ROS nodes.
- `nodelet_utils`:
  - `cras::Nodelet` base class provides integration of `diag_utils` and `param_utils` for nodelets, adds the ability to update name of the current thread with name of the nodelet, adds support for sharing a single TF buffer between multiple nodelets, and provides an `ok()` method that is similar to `ros::ok()`, but starts returning `false` when the nodelet is being unloaded.
  - `nodelet_manager_sharing_tf_buffer` is a customized nodelet manager that is able to share its own (single) TF buffer to multiple nodelets (based on `cras::Nodelet`). 
- `optional`: Provides forward compatibility for [`std::optional`](https://en.cppreference.com/w/cpp/utility/optional).
- `param_utils`: Utilities for type-safe, easy, unified and configurable access to ROS parameters. See below for examples and more details.
- `pool_allocator`: Provides a memory-pool-based allocator for ROS messages. It comes handy if you want to publish shared pointer messages on a high rate - it should decrease the time needed for object allocation via `new`.
- `rate_limiter`: Library for intelligent rate-limiting of events (mainly messages). So far it implements the algorithm from `topic_tools throttle` (not very good), and token bucket algorithm (pretty good for most cases).
- `set_utils`: Provides `isSetIntersectionEmpty()` working on a pair of `std::set`s.
- `small_map`: Provides `SmallMap` and `SmallSet`, variants of `std::map` implemented using `std::list` which are append-only and lock-free for reading.
- `span`: Provides forward compatibility for [`std::span`](https://en.cppreference.com/w/cpp/container/span).
- `string_utils`: Provides many string manipulation helpers you've always dreamed of. Universal `to_string()` that converts almost anything to a sensible string. `startsWith()`/`endsWith()`, `replace()`, `contains()`, `split()`/`join()`, `format()` (like `sprintf()` but without hassle and on `std::string`), `stripLeading()`/`stripTrailing()`, `removePrefix()`/`removeSuffix()`, `parseFloat()`/`parseDouble()` (convert string to double independent of locale!), `parseInt32()` and friends (parse many textual representations to an integer).
- `suppress_warnings`: Unified macros that suppress various compiler warnings for a piece of code.
- `test_utils`: Provide a hack that allows loading a locally-defined nodelet without the need to register it via package.xml.
- `tf2_utils`: `getRoll()`, `getPitch()`, `getYaw()`, `getRPY()` from a `tf2::Quaternion` or `geometry_msgs::Quaternion`!
  - Also provides `InterruptibleTFBuffer` that can cooperate with `cras::Nodelet` and stop a TF lookup if the nodelet is being unloaded (normally, the lookup freezes when you pause simulation time).
- `thread_utils`: `getThreadName()` and `setThreadName()`.
  - Also provides `ReverseSemaphore` synchronization primitive that counts towards zero and notifies when empty.
- `time_utils`: `remainingTime()` tells you how much of a timeout remains if you started waiting at some specified time. Conversions between `ros::Rate()` and frequency. Multiplication and division operators for ROS duration types.
- `type_utils`: Provides compile-time and run-time `getTypeName()` helper that converts a C++ type to a string containing its name.
- `urdf_utils`: Conversions between `urdf` and `Eigen` types.
- `xmlrpc_value_traits`: Type traits for `XmlRpc::XmlRpcValue`.
- `xmlrpc_value_utils`: Conversions between `XmlRpc::XmlRpcValue` and C++ and STL types.

## List of provided CMake helpers

- `node_from_nodelet`: Easily convert a nodelet into a standalone node executable. See [`cras_topic_tools`](../cras_topic_tools) package for an example.

## `param_utils`: Parameter Reading Helpers

`param_utils`, `node_utils`, `nodelet_utils` and `filter_utils` provide a type-safe, unified and highly configurable interface for reading ROS parameters. Use the same syntax to read parameters of a node, nodelet, filter, or a custom `XmlRpcValue` struct. Read an Eigen matrix, vector of unsigned ints, `ros::Duration` or `geometry_msgs::Vector3` directly without the need to write a single line of conversion code or value checking. Type of the value to read is automatically determined either from the provided default value, or from template parameter of the `getParam<>()` function.

Example usage:

```c++
// Usage in a nodelet based on `cras::Nodelet`:
// read a parameter of size_t type defaulting to 10 if not set.
// The _sz suffix is a helper to convert a numeric literal to size_t type.
const auto params = this->privateParams();
const size_t queueSize = params->getParam("queue_size", 10_sz, "messages");

// Usage in a node:
// read array of 3 doubles from parameter server into a tf2::Vector3, defaulting to the specified vector if not set.
cras::NodeParamHelper params("~");
const tf2::Vector3 gravity = params->getParam("gravity", tf2::Vector3(0, 0, -9.81), "m.s^-2");

// Usage in a filter based on cras::FilterBase:
// read a required ros::Duration parameter from a float
// the nullopt specifies instead of the default value specifies it is required.
const ros::Duration timeout = this->getParam<ros::Duration>("timeout", cras::nullopt);

// Usage directly from a XmlRpcValue dict
// read an Eigen::Vector3d from a XmlRpcValue array
XmlRpc::XmlRpcValue values;
values["offset"][0] = 1; values["offset"][1] = 2; values["offset"][2] = 3; 
auto logger = std::make_shared<cras::NodeLogHelper>();
auto paramHelper = std::make_shared<cras::XmlRpcValueGetParamAdapter>(values, "");
BoundParamHelper params(logger, paramHelper);
const Eigen::Vector3d offset = params->getParam("offset", Eigen::Vector3d::UnitX());
```

You can also configure the parameter lookup behavior, e.g. to throw exception if the value cannot be converted to the requested type (normally, the default value is used in such case):

```c++
// Will throw cras::GetParamException if float_param has a non-integer value
const int intParam = params->getParam("float_param", 0, "", {.throwIfConvertFails = true});
```

Finally, there is also a more verbose version of `getParam()` that tells more about how the lookup went:

```c++
const cras::GetParamResult<int> res = params->getParamVerbose("int", 0);
const int intParam = res.value;
const bool wasDefaultUsed = res.info.defaultUsed;
```

## `diag_utils`: Easy setup of publisher/subscriber with rate and delay diagnostics

It often happens that you know some topic should be received or published at a specific rate, or with a maximum allowed delay.

Example usage:

```c++
cras::NodeHandle nh;  // similarly in cras::Nodelet
diagnostic_updater::Updater updater(nh);  // in cras::Nodelet, just call this->getDiagUpdater()
auto pub = nh.advertiseDiagnosed<sensor_msgs::PointCloud2>(updater, "points_topic_diag", "points_topic", 10);
```

This subscribes to topic `/points_topic` and reads parameters `/points_topic_diag` to configure the diagnostics. The parameters that are read under the `/points_topic_diag` namespace are: `rate/desired`, `rate/min`, `rate/max`, `rate/tolerance`, `rate/window_size`, `delay/min`, `delay/max`.

So e.g. with the following config:

```YAML
points_topic_diag:
  rate:
    min: 10
    max: 20
    tolerance: 0.1
  delay:
    min: 0.1
    max: 0.2
```

The diagnostics would be in OK status if the topic is published on rates between 9 and 22 HZ (10/20 plus 10% tolerance), and if the difference between ROS time at time of publishing and the messages stamp is between 0.1 and 0.2 seconds. Otherwise, it would report an ERROR status.