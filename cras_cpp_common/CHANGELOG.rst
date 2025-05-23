^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cras_cpp_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.1 (2025-02-18)
------------------

2.5.0 (2025-02-13)
------------------

2.4.8 (2025-01-07)
------------------
* Added GetParamConvertingOptions() to ease defining converting getParam() that works with GCC 7.
* Added NodeWithOptionalMaster.
* Improved compatibility with newer compilers.
  Resolves https://github.com/RoboStack/ros-noetic/pull/501#issuecomment-2567224502 .
* Contributors: Martin Pecka

2.4.7 (2024-12-12)
------------------
* Fixed installation of nodelet_manager_sharing_tf_buffer.
* Contributors: Martin Pecka

2.4.6 (2024-12-12)
------------------
* nodelet_utils: Allowed accessing the shared_ptr of the shared TF buffer.
* node_from_nodelet: Worked around the bug where remapping private topics was impossible for anonymous nodes.
* tf2_utils: Added TfMessageFilter compatible with log_utils.
* time_utils: Added converters between ros::Time and struct tm.
* Fixed a few printf format issues.
* string_utils: Handle possible error in vsnprintf. Added printf-format attributes to cras::format() to enable compile-time checks of format strings.
* string_utils: Added iconvConvert(), transliterateToAscii() and toValidRosName() functions.
* Contributors: Martin Pecka

2.4.5 (2024-11-02)
------------------
* string_utils: Added date/time parsing methods.
* string_utils: Added methods for parsing integers with known radix.
* diag_utils: Added tests for offline diag updater.
* diag_utils: Added offline diag updater.
* node_from_nodelet: Added support for stopping the node when requestStop() has been called in the nodelet code.
* Contributors: Martin Pecka

2.4.4 (2024-09-14)
------------------
* Fixed roslint.
* Contributors: Martin Pecka

2.4.3 (2024-09-14)
------------------
* Fixed build with ros_comm 1.17.0 .
* Contributors: Martin Pecka

2.4.2 (2024-09-05)
------------------

2.4.1 (2024-09-04)
------------------
* Fixed roslint
* Contributors: Martin Pecka

2.4.0 (2024-09-04)
------------------
* Added small_map and fixed concurrency problems in log_utils.
* Updated fast_float to 6.1.5.
* node_from_nodelet: Fixed error message
* Contributors: Martin Pecka

2.3.9 (2024-02-27)
------------------
* Removed catkin_lint buildfarm hacks.
* Updated to fast_float 6.1.0 .
* Contributors: Martin Pecka

2.3.8 (2024-01-12)
------------------
* Fixed FindFilesystem CMake module usage of try_compile
* Contributors: Martin Pecka

2.3.7 (2024-01-09)
------------------
* node_from_nodelet: Fix syntax for Melodic.
* Contributors: Martin Pecka

2.3.6 (2024-01-09)
------------------
* node_from_nodelet: Implemented a simplified version that doesn't need the nodelet header file.
* node_from_nodelet: Fixed a bug with missing return 0 at the end of main.
* Contributors: Martin Pecka

2.3.5 (2023-11-21)
------------------
* param_utils: Added getParam() specialization for geometry_msgs/Pose messages.
* nodelet_utils: Added Resettable interface to NodeletWithSharedTfBuffer.
* Added Resettable interface compatible with cras_py_common.
* time_utils: Added saturateAdd().
* Contributors: Martin Pecka

2.3.4 (2023-10-25)
------------------
* Do not use -march=native optimizations for cras_tf2_sensor_msgs. More generic platform-specific optimizations are used.
* Contributors: Martin Pecka

2.3.3 (2023-10-06)
------------------

2.3.2 (2023-10-06)
------------------
* Fix finding std::filesystem in CMake if a non-default launguage standard is used.
* Contributors: Martin Pecka

2.3.1 (2023-07-13)
------------------

2.3.0 (2023-07-12)
------------------
* Increased minimum CMake version to 3.10.2.
* log_utils: Fixed a potential segfault when instances of MemoryLogHelper get recycled.
* Contributors: Martin Pecka

2.2.3 (2023-06-16)
------------------
* Install node_from_nodelet targets in PACKAGE_BIN and not GLOBAL_BIN
* Contributors: Martin Pecka

2.2.2 (2023-05-15)
------------------

2.2.1 (2023-05-15)
------------------

2.2.0 (2023-04-09)
------------------
* Fixed parseFloat()/parseDouble() tests to reflect the behavior change in fast_float library.
* Update fast_float to v4.0.
* Update fast_float to v3.10.0.
* Update tl/expected.
* Update tl/optional to v1.1.0.
* string_utils: Added toLower/toUpper.
* Added std::span shim.
* Contributors: Martin Pecka

2.1.2 (2023-02-10)
------------------

2.1.1 (2023-02-08)
------------------

2.1.0 (2023-02-08)
------------------
* log_utils: Added a method to set logger to HasLogger class.
* c_api: Added outputRosMessage() method that directly serializes ROS messages into allocated buffers.
* log_utils: Added MemoryLogHelper, reworked the interface of LogHelper a bit.
* Completely reworked log_utils to use macros instead of functions.
  This was needed because of the static log_location variables inside ROS\_ macros - e.g. _ONCE was only triggered once regardless of where was it called from. There were also not so helpful file:line data in the logged messages.
  Backwards compatibility was kept 99%, but there are subtle cases where it will fail - e.g. if there was `this->log->logError()` right after an `if` or `else` without braces.
* Added c_api.h.
* Added cras::expected.
* Fixed doxygen configuration and a few documentation errors.
  To get a clean rosdoc_lite run, set
  `INPUT_FILTER = "sed 's/\([ <]\)::/\1/g'"`
  in doxy.template in rosdoc_lite .
* xmlrpc_value_utils: Added conversion to dynamic_reconfigure/Config message.
* string_utils: Added cras::strip().
* Added std::any shim.
* Contributors: Martin Pecka

2.0.10 (2022-11-24)
-------------------

2.0.9 (2022-11-24)
------------------

2.0.8 (2022-11-24)
------------------

2.0.7 (2022-11-24)
------------------

2.0.6 (2022-11-24)
------------------

2.0.5 (2022-10-23)
------------------
* Added support for std::array parameters.
* Contributors: Martin Pecka

2.0.4 (2022-10-14)
------------------

2.0.3 (2022-10-07)
------------------
* cras_py_common: Extended functionality to get closer to cras_cpp_common.
* Improved readmes and added more badges to them.
* Contributors: Martin Pecka

2.0.2 (2022-08-29)
------------------
* De-flake throttle test and enable catkin_lint when it has chance to run correctly.
* Add linters and licenses.
* Set up roslaunch-check for test files.
* added catkin_lint
* added roslint, fixed issues.
* catkin_lint, moved external folder inside include/project to avoid collisions with other projects.
* Avoid threading errors when stopping nodes created by node_from_nodelet.
* time_utils: Fix build on 32bit armhf.
* Contributors: Martin Pecka

2.0.1 (2022-08-26)
------------------
* Added LICENSE file.
* Improved node_from_nodelet to use node logger instead of nodelet logger.
* Added Github Actions CI.
* Increased test coverage, fixed bug in filter diagnostics.
* tf2_utils: Added convenience methods getRoll(), getPitch() and getYaw().
* filter_utils: Adapt to upstream changes adding FilterChain::getFilters() method.
* string_utils: Allowed to limit replace() only to the beginning or end of the string.
* string_utils: Added parseDouble() and friends.
* node_from_nodelet.cmake: Made autogenerated target names less prone to naming conflicts.
* Added more logging function variants.
* Added support for std::string format in LogHelper.
* Fix logging macros to log under correct rosconsole logger.
* Added cras_node_from_nodelet() CMake function.
* Better support for custom data types in getParam() functions.
* Rename test targets so that their names do not conflict with other projects.
* Compatibility with GCC 9+.
* Fixed invalid rate conversion.
* Backwards compatibility for StatefulNodelet::shutdown().
* Improved CMakeLists.txt and header guard placement.
* Merged cras_nodelet_topic_tools with cras_topic_tools, moved repeater and joy_repeater from cras_cpp_common to cras_topic_tools.
* Implemented rate limiters.
* Refactored nodelet_manager_sharing_tf_buffer and added tests for it.
* Added urdf_utils.h.
* Improved tf2_sensor_msgs.h and added test.
* Improved set_utils.hpp.
* Added better shim for std::optional. It now provides all relevant features.
* Added more diagnostics to filter_chain_nodelet.hpp.
* Added shim for std::bind_front into functional.hpp.
* Added running_stats.hpp implementing Welford's running mean and variance computation.
* Improved filter_chain_nodelet.hpp, added tests.
* Improved cloud.hpp, added tests.
* Reorganize filter_utils directory structure.
* Improved the interface of diag_utils and node_utils, added tests. Added message_utils.
* Improved the interface of nodelet_utils, added tests. Added thread_utils with tests.
* XmlRpcValue docs and code reliability.
* Better test coverage of param_utils. Improved Eigen getParam() interface.
* Improved getParam() behavior, added test_param_utils.
* First part of upgrade: log_utils, param_utils, filter_utils, node_utils, xmlrpc, cloud.

1.0.0
-----
* Added XmlRpcValueTraits and issue an error when getParam() finds a parameter value but it has an incompatible type.
* Made FilterBase getParam() functions const.
  Allowed by https://github.com/ros/filters/pull/35 (released in Melodic filters 1.8.2 (October 2021) and Noetic filters 1.9.1 (September 2021)).
* Fixed diagnosed publisher creation scripts
* Little fixes, added pool allocator helpers.
* Improved diagnostics
* Fix compilation with gcc 8
* Fix for systems with old versions of diagnostic_updater
* Compatibility with diagnostic_updater 1.9.6 and newer.
* Fixed memory corruption by cras::transformOnlyChannels().
* Improve lazy subscription behavior in filter_chain_nodelet.hpp
* Fixed SEVERE_WARNING in nodelet_manager_sharing_tf_buffer.
* Fixed segfaults when unloading NodeletWithDiagnostics.
* node_utils: added paramsForNodeHandle()
* Moved filter_chain_nodelet from nifti_laser_filtering to here.
* Added missing diag functions.
* Added missing nodelet logging macros.
* Refactored param_utils to be also usable in filters.
* Small refactoring of CMakeLists.txt and related stuff, modernize header guards.
* Fixed reading of hierarchical parameters in diag_utils.hpp.
* Added diagnostics utils.
* Reworked getParam helpers, added some more utility functions.
* Added NodeletWithDiagnostics trait.
* Added utilities for working with pointclouds - generic iterator, transformOnlyChannels() and more utility functions.
* Added docs.
* Added NodeletWithSharedTfBuffer::usesSharedBuffer().
* Little fixes, verified that Eigen compiles using AVX instructions.
* Added a mixin for nodelets which share a tf buffer with their nodelet manager (and added that custom manager, too).
* Forced tf2_sensor_msgs cloud transform tools to utilize SIMD instructions.
* Improved nodelet_utils, converted all convenience functions into mixins that can be side-loaded to any class.
* Separated nodelet param loading to a separate class so that it can be utilized even in nodelets that are not descendants of cras::Nodelet().
* Added tf2_sensor_msgs with transformWithChannels() function to help correctly transforming pointclouds.
* Remove build warning.
* Fixed to_string() for collections so that it doesn't include the separator after the last item.
* Added getParamVerboseSet() to filter_utils.hpp
* Repeater and specific joy repeater.
* Topic repeater node (every period, instant republish option).
* Added CMake module for using the most modern C++ filesystem API available.
* Added to_string(bool) to string_utils.hpp
* filter_utils: Added support for disabling filters during runtime.
* Added to_string<std::set>()
* Added tf2_utils.
* Added ros::Time to_string.
* filter_utils: Added a possibility to specify a callback in FilterChain that is called after application of each filter.
* nodelet_utils: Added shutdown() method meant to be called from destructors.
* nodelet_utils: Added option to use nodelet-aware canTransform
* nodelet_utils: Added updateThreadName().
* Added nodelet utils.
* Added set utils.
* Added math utils.
* Added inline modifiers to avoid multiple definitions issues.
* Added std::string - const char* interop overload to getParam.
* Moved cras_cpp_common from subt/tradr-robot/tradr-resources.
* added string_utils::to_string(XmlRpc::XmlRpcValue)
* topic_utils -> string_utils, added string_utils::to_string
* Fixed bad design of filter_utils.
* added ros::Duration specializations for node_utils::getParam() and filter_utils::getParam().
* Added filter_utils, time_utils, topic_utils, added unsigned specializations for node_utils::getParam().
* Added cras_cpp_common.
* Contributors: Martin Pecka, Tomas Petricek
