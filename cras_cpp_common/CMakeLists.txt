cmake_minimum_required(VERSION 3.0.2)
project(cras_cpp_common)

set(CMAKE_CXX_STANDARD 17)  # allows better Eigen alignment handling

find_package(catkin REQUIRED COMPONENTS
  bondcpp
  diagnostic_updater
  dynamic_reconfigure
  filters
  nodelet
  rosconsole
  roscpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  topic_tools
)

find_package(diagnostic_updater 1.9.6 QUIET)
set(HAS_SLOW_TOPIC_UPDATER ${diagnostic_updater_FOUND})

# detect https://github.com/ros/filters/pull/60 in the filters package added in 1.8.3 and 1.9.2
find_package(filters QUIET)
if(filters_VERSION VERSION_LESS 1.9.0)
  if(filters_VERSION VERSION_GREATER_EQUAL 1.8.3)
    set(HAS_FILTERCHAIN_GET_FILTERS ${filters_FOUND})
  endif()
else()
  if(filters_VERSION VERSION_GREATER_EQUAL 1.9.2)
    set(HAS_FILTERCHAIN_GET_FILTERS ${filters_FOUND})
  endif()
endif()

generate_dynamic_reconfigure_options(
        cfg/FilterChain.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater dynamic_reconfigure filters nodelet rosconsole roscpp sensor_msgs tf2_ros topic_tools
  LIBRARIES cloud diag_utils filter_utils improved_updater nodelet_utils slow_topic_diag_updater string_utils tf2_sensor_msgs_cras tf2_utils time_utils
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
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
add_dependencies(repeater ${catkin_EXPORTED_TARGETS})

add_executable(joy_repeater src/joy_repeater.cpp)
target_link_libraries(joy_repeater ${catkin_LIBRARIES})
add_dependencies(joy_repeater ${catkin_EXPORTED_TARGETS})

add_library(string_utils src/string_utils.cpp)
target_link_libraries(string_utils ${catkin_LIBRARIES})
add_dependencies(string_utils ${catkin_EXPORTED_TARGETS})

add_library(log_utils INTERFACE)
target_sources(log_utils INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/log_utils.h>)
target_include_directories(log_utils INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(log_utils INTERFACE string_utils)

add_library(param_utils INTERFACE)
target_sources(param_utils INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/param_utils.hpp>)
target_include_directories(param_utils INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(param_utils INTERFACE log_utils ${catkin_LIBRARIES})

add_library(node_utils INTERFACE)
target_sources(node_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/node_utils.hpp>)
target_include_directories(node_utils INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(node_utils INTERFACE string_utils param_utils ${catkin_LIBRARIES})

add_library(improved_updater src/diagnostics/ImprovedUpdater.cpp)
target_link_libraries(improved_updater ${catkin_LIBRARIES})
add_dependencies(improved_updater ${catkin_EXPORTED_TARGETS})

add_library(slow_topic_diag_updater src/diagnostics/SlowTopicDiagUpdater.cpp)
target_link_libraries(slow_topic_diag_updater improved_updater ${catkin_LIBRARIES})
add_dependencies(slow_topic_diag_updater ${catkin_EXPORTED_TARGETS})
if (HAS_SLOW_TOPIC_UPDATER)
  target_compile_definitions(slow_topic_diag_updater PUBLIC HAS_SLOW_TOPIC_UPDATER=1)
endif()

add_library(nodelet_utils src/nodelet_utils.cpp)
target_link_libraries(nodelet_utils PRIVATE string_utils PUBLIC improved_updater ${catkin_LIBRARIES} pthread slow_topic_diag_updater)
add_dependencies(nodelet_utils ${catkin_EXPORTED_TARGETS})

add_library(filter_utils src/filter_utils.cpp)
target_link_libraries(filter_utils string_utils ${catkin_LIBRARIES})
add_dependencies(filter_utils ${catkin_EXPORTED_TARGETS})
if (HAS_FILTERCHAIN_GET_FILTERS)
  target_compile_definitions(filter_utils PRIVATE HAS_FILTERCHAIN_GET_FILTERS=1)
endif()

add_library(time_utils src/time_utils.cpp)
target_link_libraries(time_utils ${catkin_LIBRARIES})
add_dependencies(time_utils ${catkin_EXPORTED_TARGETS})

add_library(tf2_sensor_msgs_cras src/tf2_sensor_msgs.cpp)
target_link_libraries(tf2_sensor_msgs_cras PRIVATE cloud string_utils PUBLIC ${catkin_LIBRARIES})
add_dependencies(tf2_sensor_msgs_cras ${catkin_EXPORTED_TARGETS})
target_compile_options(tf2_sensor_msgs_cras PRIVATE -march=native)

add_library(tf2_utils src/tf2_utils.cpp)
target_link_libraries(tf2_utils ${catkin_LIBRARIES})
add_dependencies(tf2_utils ${catkin_EXPORTED_TARGETS})

add_library(math_utils INTERFACE)
target_sources(math_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/math_utils.hpp>)
target_include_directories(math_utils INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(set_utils INTERFACE)
target_sources(set_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/set_utils.hpp>)
target_include_directories(set_utils INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(type_utils INTERFACE)
target_sources(type_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/type_utils.hpp>)

add_library(xmlrpc_value_traits INTERFACE)
target_sources(xmlrpc_value_traits INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/xmlrpc_value_traits.hpp>)
target_include_directories(xmlrpc_value_traits INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(diag_utils src/diag_utils.cpp)
target_link_libraries(diag_utils PRIVATE type_utils PUBLIC param_utils slow_topic_diag_updater ${catkin_LIBRARIES})
add_dependencies(diag_utils ${catkin_EXPORTED_TARGETS})
target_include_directories(type_utils INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(filter_chain_nodelet INTERFACE)
target_sources(filter_chain_nodelet INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/filter_chain_nodelet.hpp>)
target_link_libraries(filter_chain_nodelet INTERFACE diag_utils nodelet_utils filter_utils)
target_include_directories(filter_chain_nodelet INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(nodelet_manager_sharing_tf_buffer nodes/nodelet_manager_sharing_tf_buffer.cpp)
target_link_libraries(nodelet_manager_sharing_tf_buffer nodelet_utils ${catkin_LIBRARIES})
add_dependencies(nodelet_manager_sharing_tf_buffer ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

install(TARGETS cloud diag_utils filter_utils improved_updater joy_repeater math_utils node_utils
  nodelet_manager_sharing_tf_buffer nodelet_utils repeater set_utils slow_topic_diag_updater string_utils
  tf2_sensor_msgs_cras tf2_utils time_utils

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
