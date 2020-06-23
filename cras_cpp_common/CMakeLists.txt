cmake_minimum_required(VERSION 2.8.3)
project(cras_cpp_common)

set(CMAKE_CXX_STANDARD 17)  # allows better Eigen alignment handling

find_package(catkin REQUIRED COMPONENTS
  bondcpp
  diagnostic_updater
  filters
  nodelet
  roscpp
  rosconsole
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  topic_tools
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater filters roscpp rosconsole sensor_msgs tf2_ros topic_tools
  LIBRARIES cloud filter_utils improved_updater nodelet_utils slow_topic_diag_updater string_utils tf2_sensor_msgs_cras tf2_utils time_utils
  CFG_EXTRAS cras_cpp_common-extras.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cloud src/cloud.cpp)
target_link_libraries(cloud ${catkin_LIBRARIES})
add_dependencies(cloud ${catkin_EXPORTED_TARGETS})

add_executable(repeater src/repeater.cpp)
target_link_libraries(repeater ${catkin_LIBRARIES})

add_executable(joy_repeater src/joy_repeater.cpp)
target_link_libraries(joy_repeater ${catkin_LIBRARIES})

add_library(string_utils src/string_utils.cpp)
target_link_libraries(string_utils ${catkin_LIBRARIES})
add_dependencies(string_utils ${catkin_EXPORTED_TARGETS})

add_library(node_utils INTERFACE)
target_sources(node_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/node_utils.hpp)
target_include_directories(node_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_library(improved_updater src/diagnostics/ImprovedUpdater.cpp)
target_link_libraries(improved_updater ${catkin_LIBRARIES})
add_dependencies(improved_updater ${catkin_EXPORTED_TARGETS})

add_library(slow_topic_diag_updater src/diagnostics/SlowTopicDiagUpdater.cpp)
target_link_libraries(slow_topic_diag_updater improved_updater ${catkin_LIBRARIES})
add_dependencies(slow_topic_diag_updater ${catkin_EXPORTED_TARGETS})

add_library(nodelet_utils src/nodelet_utils.cpp)
target_link_libraries(nodelet_utils string_utils improved_updater ${catkin_LIBRARIES} pthread)

add_library(filter_utils src/filter_utils.cpp)
target_link_libraries(filter_utils ${catkin_LIBRARIES})
add_dependencies(filter_utils string_utils)

add_library(time_utils src/time_utils.cpp)
target_link_libraries(time_utils ${catkin_LIBRARIES})
add_dependencies(time_utils ${catkin_EXPORTED_TARGETS})

add_library(tf2_sensor_msgs_cras src/tf2_sensor_msgs.cpp)
target_link_libraries(tf2_sensor_msgs_cras cloud string_utils ${catkin_LIBRARIES})
add_dependencies(tf2_sensor_msgs_cras ${catkin_EXPORTED_TARGETS})
target_compile_options(tf2_sensor_msgs_cras PRIVATE -march=native)

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

add_library(log_utils INTERFACE)
target_sources(log_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/log_utils.h)
target_include_directories(log_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_library(param_utils INTERFACE)
target_sources(param_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/param_utils.hpp)
target_include_directories(param_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_library(type_utils INTERFACE)
target_sources(type_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/type_utils.hpp)
target_include_directories(type_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_library(diag_utils INTERFACE)
target_sources(diag_utils INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/cras_cpp_common/diag_utils.hpp)
target_include_directories(diag_utils INTERFACE ${catkin_INCLUDE_DIRS})

add_executable(nodelet_manager_sharing_tf_buffer nodes/nodelet_manager_sharing_tf_buffer.cpp)
target_link_libraries(nodelet_manager_sharing_tf_buffer nodelet_utils ${catkin_LIBRARIES})
add_dependencies(nodelet_manager_sharing_tf_buffer ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

install(TARGETS cloud repeater joy_repeater string_utils node_utils nodelet_utils filter_utils time_utils
        tf2_sensor_msgs_cras tf2_utils math_utils set_utils improved_updater slow_topic_diag_updater

        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
