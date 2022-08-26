^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cras_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
