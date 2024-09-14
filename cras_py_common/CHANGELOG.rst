^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cras_py_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.4 (2024-09-14)
------------------

2.4.3 (2024-09-14)
------------------

2.4.2 (2024-09-05)
------------------
* log_utils: Addded log_once_identical() functions.
* log_utils: Fixed stack information for cras.log().
* Contributors: Martin Pecka

2.4.1 (2024-09-04)
------------------

2.4.0 (2024-09-04)
------------------
* string_utils: Fixed iconv functions when running with LC_ALL=C or other weird locale.
* Added python_utils.
* string_utils: Added methods for iconv-like conversions of strings and sanitization or ROS names.
* Contributors: Martin Pecka

2.3.9 (2024-02-27)
------------------
* Removed catkin_lint buildfarm hacks.
* Contributors: Martin Pecka

2.3.8 (2024-01-12)
------------------

2.3.7 (2024-01-09)
------------------

2.3.6 (2024-01-09)
------------------

2.3.5 (2023-11-21)
------------------

2.3.4 (2023-10-25)
------------------

2.3.3 (2023-10-06)
------------------

2.3.2 (2023-10-06)
------------------
* param_utils: Removed deprecated numpy aliases.
* Contributors: Martin Pecka

2.3.1 (2023-07-13)
------------------

2.3.0 (2023-07-12)
------------------
* Increased minimum CMake version to 3.10.2.
* Contributors: Martin Pecka

2.2.3 (2023-06-16)
------------------

2.2.2 (2023-05-15)
------------------
* ctypes_utils: Added ScalarAllocator.
* Contributors: Martin Pecka

2.2.1 (2023-05-15)
------------------
* message_utils: Added dict_to_dynamic_config_msg().
* ctypes_utils: Added c_array() method.
* message_utils: Added get_srv_types() and get_cfg_module().
* Contributors: Martin Pecka

2.2.0 (2023-04-09)
------------------

2.1.2 (2023-02-10)
------------------

2.1.1 (2023-02-08)
------------------

2.1.0 (2023-02-08)
------------------
* ctypes_utils: Added specialized allocators for ROS messages and for rosconsole logs.
* string_utils: Register genpy Time and Duration for to_str() conversion, too.
* ctypes_utils: Do not add one byte to StringAlloc allocated size. The caller has to do it now.
* ctypes_utils: Autodetection of length of a BufferStringIO stream.
* ctypes_utils: Added get_ro_c_buffer.
* Added utilities for working with ctypes.
* Allow resetting nodes by topic.
* Added support for enums in to_str() and param utils.
* Hide tf2_ros includes in geometry_utils.py inside function calls.
* Contributors: Martin Pecka

2.0.10 (2022-11-24)
-------------------
* Fix test bug.
* Contributors: Martin Pecka

2.0.9 (2022-11-24)
------------------
* Relay attribute access in GenericMessageSubscriber to the raw subscriber (to allow e.g. calling unsubscribe()).
* Contributors: Martin Pecka

2.0.8 (2022-11-24)
------------------
* Pass connection header to user callback in GenericMessageSubscriber.
* Contributors: Martin Pecka

2.0.7 (2022-11-24)
------------------
* Moved get_msg_type from type_utils to message_utils and added get_msg_field there.
* Contributors: Martin Pecka

2.0.6 (2022-11-24)
------------------
* Added topic_utils and type_utils.
* Contributors: Martin Pecka

2.0.5 (2022-10-23)
------------------
* Added static_transform_broadcaster docs to readme.
* Contributors: Martin Pecka

2.0.4 (2022-10-14)
------------------
* Fixed StaticTransformBroadcaster on Noetic and added unit test for it.
* Added missing license notices.
* Contributors: Martin Pecka

2.0.3 (2022-10-07)
------------------
* Improved time_utils, added node_utils.
* Added geometry_utils.py.
* Remove support for long integer type to achieve compatibility with Python 3.
* Extended functionality to get closer to cras_cpp_common.
* Improved readmes and added more badges to them.
* Contributors: Martin Pecka

2.0.2 (2022-08-29)
------------------
* De-flake throttle test and enable catkin_lint when it has chance to run correctly.
* Added website links.
* Add linters and licenses.
* Contributors: Martin Pecka

2.0.1 (2022-08-26)
------------------
* Moved hack_frame_id from cras_py_common to cras_topic_tools.

1.0.0
-----------
* Added improved static_transform_broadcaster for Python.
* Added hack_frame_id
* Added cras_py_common
* Contributors: Martin Pecka
