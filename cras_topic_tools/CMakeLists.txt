cmake_minimum_required(VERSION 3.0.2)
project(cras_topic_tools)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS nodes/filter nodes/repeat
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
