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

## List of Provided Modules

The most useful functions and classes from these modules are available directly from the `cras` package, i.e.

```python
from cras import to_str
# instead of
from cras.string_utils import to_str
```

- [`ctypes_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.ctypes_utils): Utilities for working with the ctypes library.
- [`geometry_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.geometry_utils): Finally THE module providing easy and foolproof conversion between quaternions and roll/pitch/yaw notation.
- [`log_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.log_utils): Some convenience for `rospy` logging. Provides `log*_once_identical()` functions to log unique messages.
- [`message_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.message_utils): Conversion of `std_msgs/String` to Python type etc. Generic access to message fields using a string "address".
- [`node_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.node_utils): Utilities for easier writing of nodes, adding e.g. easy-to-write `reset()` function that is automatically called when ROS time jumps back/forward.
- [`param_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.param_utils): Utilities for type-safe, easy, unified and configurable access to ROS parameters. See below for examples and more details.
- [`python_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.python_utils): General Python utilities.
- [`static_transform_broadcaster`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.static_transform_broadcaster): An drop-in replacement of `tf2_ros.static_transform_broadcaster` that can safely publish multiple transforms from a single node ([upstreaming to tf2_ros in progress](https://github.com/ros/geometry2/pull/484)).
- [`string_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.string_utils): Universal `to_str()` that converts almost anything to a sensible string.
- [`test_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.test_utils): Utilities for writing unit tests, e.g. a tool that can "read" what was written by `rospy.loginfo()`.
- [`time_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.time_utils): Conversions between `rospy.Rate` and frequency. `rospy.Rate` equality comparison. Min/max time and duration constants. `WallTime`, `WallRate`, `SteadyTime`, `SteadyRate`, and a `Timer` that can use these custom rates.
- [`topic_utils`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#module-cras.topic_utils): Generic topic subscriber.

## `node_utils`: Resettable nodes

Nodes can support resetting of their state. This is a concept very useful for simulation or postprocessing. Each node that supports resetting should be marked like this in its documentation:

> **Note**
> This node is resettable and checks for time jumps.

This particularly means that the node subscribes to topics `/reset` and `~reset` (any type). Whenever a message is received on either of these topics, the node's `reset()` method is called.

Resetting is also done automatically any time the node figures out ROS time jumped back/forward too much. It is configured via these parameters:
- `/jump_back_tolerance` (float, default 3.0 in wall time and 0.0 in sim time):
  Threshold for ROS time jump back detection.
- `~jump_back_tolerance` (float, default from `/jump_back_tolerance`): Threshold for ROS time jump back detection.
- `~reset_on_time_jump_back` (bool, default True): Whether to call `reset()` when ROS time jumps back.
- `/jump_forward_tolerance` (float, default 10.0 in sim time and max duration in wall time):
  Threshold for ROS time jump forward detection.
- `~jump_forward_tolerance` (float, default from `/jump_forward_tolerance`):
  Threshold for ROS time jump forward detection.
- `~reset_on_time_jump_forward` (bool, default True in sim time and False in wall time):
  Whether to call `reset()` when ROS time jumps forward.

The node should either call `self.check_time_jump()` in each message callback, or call `self.start_auto_check_time_jump()` when the node is initialized (this runs a background thread doing the checks).

Please note that you may have to increase the jump forward tolerance if playing bag files with very high rates. In such case, it is safer to increase the threshold to 60 seconds or even more.

## `param_utils`: Parameter Reading Helpers

`param_utils` provide a type-safe, unified and highly configurable interface for reading ROS parameters. Read a numpy matrix, vector of unsigned ints, `rospy.Duration` or `geometry_msgs.msg.Vector3` directly without the need to write a single line of conversion code or value checking. Type of the value to read is automatically determined either from the provided default value, or from `result_type` parameter.

Example usage:

```python
from cras import get_param, GetParamException
from geometry_msgs.msg import Vector3

# read array of 3 doubles from parameter server into a geometry_msgs.msg.Vector3, defaulting to the specified vector if not set.
gravity = get_param("gravity", Vector3(0, 0, -9.81), "m.s^-2")

# required parameters are specified by not having a default; if you still want conversion to some type, use result_type
try:
    gravity2 = get_param("gravity", "m.s^-2", result_type=Vector3)
except GetParamException as e:
    # the exception is raised if the required parameter is not set
    # e.info contains details about the unsuccessful lookup
    print(e.info.message)
```

You can also configure the parameter lookup behavior, e.g. to throw exception if the value cannot be converted to the requested type (normally, the default value is used in such case):

```python
# will throw GetParamException if int_param has a value different from 0 and 1
from cras import get_param
bool_param = get_param("int_param", False, throw_if_convert_fails=True)
```

Finally, there is also a [more verbose version of `get_param()`](https://docs.ros.org/en/api/cras_py_common/html/cras.html#cras.param_utils.get_param_verbose) that tells more about how the lookup went:

```python
from cras import get_param_verbose
res = get_param_verbose("int", 0)
int_param = res.value
was_default_used = res.info.default_used
```
