^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cras_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2023-07-12)
------------------
* priority_mux: Added possibility to send a message just before disabling a topic.
* shape_shifter: Improved getHeader().
* Increased minimum CMake version to 3.10.2.
* priority_mux: Total rewrite, now it should really work.
* Contributors: Martin Pecka

2.2.3 (2023-06-16)
------------------

2.2.2 (2023-05-15)
------------------

2.2.1 (2023-05-15)
------------------

2.2.0 (2023-04-09)
------------------
* Added heartbeat.
* Simplified lazy subscriber classes and nodelet code. Added ~tcp_no_delay param to most nodelets. API breaks included, but hopefully nothing will be affected.
  API breaks:
  - LazySubscriberBase->ConditionalSubscriber
  - LazySubscriber: callback is now passed as parameter instead of being a virtual member function
  - GenericLazyPubSub: callback is now passed as parameter instead of being a virtual member function
  - the nodelets no longer have a dedicated PubSub object that would be reusable elsewhere (the minimum probability of reuse does not outweigh the code complications it brings)
* Contributors: Martin Pecka

2.1.2 (2023-02-10)
------------------

2.1.1 (2023-02-08)
------------------

2.1.0 (2023-02-08)
------------------
* Added cras::ShapeShifter as a memory-safe option for Melodic.
* Added a method to convert a message instance to ShapeShifter.
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

2.0.4 (2022-10-14)
------------------

2.0.3 (2022-10-07)
------------------
* Lower test_repeat thresholds even more, OSRF buildfarm is really bad at this
* cras_py_common: Extended functionality to get closer to cras_cpp_common.
* Improved readmes and added more badges to them.
* Contributors: Martin Pecka

2.0.2 (2022-08-29)
------------------
* Added option to change_header to apply current ROS time to stamp.
* De-flake throttle test and enable catkin_lint when it has chance to run correctly.
* Satisfy more pedantic Noetic linter.
* Added website links.
* Add linters and licenses.
* Fixed dependency.
* Added linters.
* Fix catkin_lint, re-enable testing.
* Improved comments in nodelet.xml.
* De-flake and speed up throttle test.
* De-flake and speed up repeater test.
* De-flake and speed up relay test.
* De-flake and speed up filter test.
* De-flake and speed up change_header test.
* Avoid threading errors when stopping nodes created by node_from_nodelet.
* Fix compilation on GCC 7.
* Contributors: Martin Pecka

2.0.1 (2022-08-26)
------------------
* Moved hack_frame_id from cras_py_common to cras_topic_tools.
* Moved launch files.
* Merged cras_nodelet_topic_tools with cras_topic_tools, moved repeater and joy_repeater from cras_cpp_common to cras_topic_tools.

1.0.0
-----
* Add option to publish only on timer event.
* Protected mux_replay against looping.
* Added mux_replay
* Added filter node. Added ~reset_timer_on_msg to repeat.
* Added topic repeater.
* Contributors: Martin Pecka
