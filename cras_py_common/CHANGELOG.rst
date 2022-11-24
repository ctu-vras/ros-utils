^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cras_py_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
