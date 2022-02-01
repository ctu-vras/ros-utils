cmake_minimum_required(VERSION 3.1)
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

generate_dynamic_reconfigure_options(
  cfg/FilterChain.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater dynamic_reconfigure filters nodelet rosconsole roscpp sensor_msgs tf2_ros topic_tools
  LIBRARIES cloud diag_utils cras_filter_chain improved_updater log_utils nodelet_utils param_utils slow_topic_diag_updater string_utils tf2_sensor_msgs_cras tf2_utils time_utils
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CCC_INCLUDE_BASE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")

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

add_library(log_utils src/log_utils/node.cpp src/log_utils/nodelet.cpp)
target_sources(log_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/log_utils.h>)
target_link_libraries(log_utils string_utils ${catkin_LIBRARIES})
add_dependencies(log_utils ${catkin_EXPORTED_TARGETS})

add_library(param_utils
  src/param_utils/get_param_adapters/node_handle.cpp
  src/param_utils/get_param_adapters/xmlrpc_value.cpp
)
target_sources(param_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/param_utils.hpp>)
target_link_libraries(param_utils log_utils string_utils type_utils xmlrpc_value_traits ${catkin_LIBRARIES})
add_dependencies(param_utils ${catkin_EXPORTED_TARGETS})

add_library(node_utils INTERFACE)
target_sources(node_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/node_utils.hpp>)
target_link_libraries(node_utils INTERFACE log_utils string_utils param_utils ${catkin_LIBRARIES})

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
target_link_libraries(nodelet_utils PRIVATE string_utils PUBLIC log_utils improved_updater param_utils ${catkin_LIBRARIES} pthread slow_topic_diag_updater)
add_dependencies(nodelet_utils ${catkin_EXPORTED_TARGETS})

add_library(cras_filter_base INTERFACE)
target_sources(cras_filter_base INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_base.hpp>)
target_link_libraries(cras_filter_base INTERFACE string_utils param_utils)

add_library(cras_filter_chain src/filter_chain.cpp)
target_link_libraries(cras_filter_chain string_utils ${catkin_LIBRARIES})
add_dependencies(cras_filter_chain log_utils)

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
target_sources(math_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/math_utils.hpp>)

add_library(set_utils INTERFACE)
target_sources(set_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/set_utils.hpp>)

add_library(type_utils INTERFACE)
target_sources(type_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/type_utils.hpp>)
target_link_libraries(type_utils INTERFACE string_utils)

add_library(xmlrpc_value_traits INTERFACE)
target_sources(xmlrpc_value_traits INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/xmlrpc_value_traits.hpp>)
target_link_libraries(xmlrpc_value_traits INTERFACE string_utils ${catkin_LIBRARIES})

add_library(xmlrpc_value_utils INTERFACE)
target_sources(xmlrpc_value_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/xmlrpc_value_utils.hpp>)
target_link_libraries(xmlrpc_value_utils INTERFACE xmlrpc_value_traits)

add_library(diag_utils src/diag_utils.cpp)
target_link_libraries(diag_utils PUBLIC slow_topic_diag_updater ${catkin_LIBRARIES})
add_dependencies(diag_utils ${catkin_EXPORTED_TARGETS})

add_library(filter_chain_nodelet INTERFACE)
target_sources(filter_chain_nodelet INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_chain_nodelet.hpp>)
target_link_libraries(filter_chain_nodelet INTERFACE diag_utils nodelet_utils cras_filter_chain)
add_dependencies(filter_chain_nodelet INTERFACE ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(nodelet_manager_sharing_tf_buffer nodes/nodelet_manager_sharing_tf_buffer.cpp)
target_link_libraries(nodelet_manager_sharing_tf_buffer nodelet_utils ${catkin_LIBRARIES})
add_dependencies(nodelet_manager_sharing_tf_buffer ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

install(TARGETS cloud diag_utils cras_filter_base cras_filter_chain improved_updater joy_repeater
  nodelet_manager_sharing_tf_buffer repeater slow_topic_diag_updater string_utils
  tf2_sensor_msgs_cras tf2_utils time_utils

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_cloud test/test_cloud.cpp)
  target_link_libraries(test_cloud cloud ${catkin_LIBRARIES})

  
  catkin_add_gtest(test_xmlrpc_value_traits test/test_xmlrpc_value_traits.cpp)
  target_link_libraries(test_xmlrpc_value_traits xmlrpc_value_traits ${catkin_LIBRARIES})

  catkin_add_gtest(test_xmlrpc_value_utils test/test_xmlrpc_value_utils.cpp)
  target_link_libraries(test_xmlrpc_value_utils xmlrpc_value_utils ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_base test/test_filter_base.test test/test_filter_base.cpp)
  target_link_libraries(test_filter_base cras_filter_base ${catkin_LIBRARIES})
  target_compile_options(test_filter_base PRIVATE -fno-var-tracking-assignments)  # speed up build
  
  add_rostest_gtest(test_param_utils test/test_param_utils.test test/test_param_utils.cpp)
  target_link_libraries(test_param_utils param_utils ${catkin_LIBRARIES})
  target_compile_options(test_param_utils PRIVATE -fno-var-tracking-assignments)  # speed up build
  
  add_rostest_gtest(test_node_utils test/test_node_utils.test test/test_node_utils.cpp)
  target_link_libraries(test_node_utils param_utils node_utils ${catkin_LIBRARIES})
  target_compile_options(test_node_utils PRIVATE -fno-var-tracking-assignments)  # speed up build
  
  add_rostest_gtest(test_nodelet_get_param test/test_nodelet_get_param.test test/test_nodelet_get_param.cpp)
  target_link_libraries(test_nodelet_get_param param_utils nodelet_utils ${catkin_LIBRARIES})
  target_compile_options(test_nodelet_get_param PRIVATE -fno-var-tracking-assignments)  # speed up build
endif()
