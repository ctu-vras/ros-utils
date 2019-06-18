cmake_minimum_required(VERSION 2.8.3)
project(cras_cpp_common)

find_package(catkin REQUIRED COMPONENTS
  filters
  nodelet
  roscpp
  rosconsole
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS filters roscpp rosconsole sensor_msgs tf2_ros
  LIBRARIES filter_utils nodelet_utils string_utils tf2_utils time_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(string_utils src/string_utils.cpp)
target_link_libraries(string_utils ${catkin_LIBRARIES})
add_dependencies(string_utils ${catkin_EXPORTED_TARGETS})

add_library(node_utils INTERFACE)
target_sources(node_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/node_utils.hpp)
target_include_directories(node_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_library(nodelet_utils src/nodelet_utils.cpp)
target_link_libraries(nodelet_utils ${catkin_LIBRARIES} pthread)
add_dependencies(nodelet_utils string_utils)

add_library(filter_utils src/filter_utils.cpp)
target_link_libraries(filter_utils ${catkin_LIBRARIES})
add_dependencies(filter_utils string_utils)

add_library(time_utils src/time_utils.cpp)
target_link_libraries(time_utils ${catkin_LIBRARIES})
add_dependencies(time_utils ${catkin_EXPORTED_TARGETS})

add_library(tf2_utils src/tf2_utils.cpp)
target_link_libraries(tf2_utils ${catkin_LIBRARIES})
add_dependencies(tf2_utils ${catkin_EXPORTED_TARGETS})

add_library(math_utils INTERFACE)
target_sources(math_utils INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/math_utils.hpp)
target_include_directories(math_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_library(set_utils INTERFACE)
target_sources(set_utils INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/set_utils.hpp)
target_include_directories(set_utils INTERFACE ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)