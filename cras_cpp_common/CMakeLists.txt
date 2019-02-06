cmake_minimum_required(VERSION 2.8.3)
project(cras_cpp_common)

find_package(catkin REQUIRED COMPONENTS
  filters
  roscpp
  rosconsole
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS filters roscpp rosconsole
  LIBRARIES time_utils string_utils
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

add_library(filter_utils INTERFACE)
target_sources(filter_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/filter_utils.hpp)
target_include_directories(filter_utils INTERFACE ${catkin_INCLUDE_DIRS})
add_dependencies(filter_utils string_utils)

add_library(time_utils src/time_utils.cpp)
target_link_libraries(time_utils ${catkin_LIBRARIES})
add_dependencies(time_utils ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)