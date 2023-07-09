cmake_minimum_required(VERSION 3.10.2)
project(tf_static_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_static_publisher src/tf_static_publisher.cpp)
target_link_libraries(tf_static_publisher ${catkin_LIBRARIES})
