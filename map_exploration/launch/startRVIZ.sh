#!/bin/bash
# this launches rviz with custom-made config file, which is more suitable for debugging purposes
rosrun rviz rviz -d "$(rospack find robot_mapping)/launch/mappingRVIZdebug2.rviz"

