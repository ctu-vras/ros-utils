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

# find pthread and provide it as Threads::Threads imported target
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/FilterChain.cfg
)

catkin_package(
  INCLUDE_DIRS external include
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    filters
    nodelet
    rosconsole
    roscpp
    sensor_msgs
    tf2_ros
    topic_tools
  DEPENDS
    Boost
    EIGEN3
    urdfdom_headers
  LIBRARIES
    cloud
    cras_diag_updater
    cras_filter_chain
    cras_nodelet_manager
    cras_rate_limiter
    cras_semaphore
    diag_utils
    log_utils
    node_utils
    nodelet_manager_sharing_tf_buffer_lib
    nodelet_utils
    param_utils
    string_utils
    tf2_sensor_msgs_cras
    tf2_utils
    time_utils
    thread_utils
    type_utils
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
)

include_directories(
  external
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
target_link_libraries(string_utils time_utils ${catkin_LIBRARIES})
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

add_library(node_utils src/node_utils/node_handle_with_diagnostics.cpp src/node_utils/param_helper.cpp)
target_sources(node_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/node_utils.hpp>)
target_link_libraries(node_utils diag_utils log_utils string_utils param_utils ${catkin_LIBRARIES})
add_dependencies(node_utils ${catkin_EXPORTED_TARGETS})

add_library(cras_diag_updater src/diag_utils/updater.cpp)
target_link_libraries(cras_diag_updater ${catkin_LIBRARIES})
add_dependencies(cras_diag_updater ${catkin_EXPORTED_TARGETS})

add_library(nodelet_utils src/nodelet_utils/nodelet_aware_tf_buffer.cpp src/nodelet_utils/stateful_nodelet.cpp)
target_link_libraries(nodelet_utils PRIVATE string_utils PUBLIC thread_utils log_utils cras_diag_updater param_utils time_utils tf2_utils ${catkin_LIBRARIES})
add_dependencies(nodelet_utils ${catkin_EXPORTED_TARGETS})

add_library(cras_nodelet_manager src/nodelet_utils/loader_ros.cpp src/nodelet_utils/nodelet_manager.cpp)
target_link_libraries(cras_nodelet_manager ${catkin_LIBRARIES})
add_dependencies(cras_nodelet_manager ${catkin_EXPORTED_TARGETS})

add_library(cras_filter_base INTERFACE)
target_sources(cras_filter_base INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_utils/filter_base.hpp>)
target_link_libraries(cras_filter_base INTERFACE string_utils param_utils)

add_library(cras_filter_chain src/filter_chain.cpp)
target_link_libraries(cras_filter_chain cras_filter_base string_utils ${catkin_LIBRARIES})
add_dependencies(cras_filter_chain log_utils ${catkin_EXPORTED_TARGETS})

add_library(time_utils src/time_utils.cpp src/time_utils/interruptible_sleep_interface.cpp)
target_link_libraries(time_utils PUBLIC ${catkin_LIBRARIES} PRIVATE cras_semaphore)
add_dependencies(time_utils ${catkin_EXPORTED_TARGETS})

add_library(tf2_sensor_msgs_cras src/tf2_sensor_msgs.cpp)
target_link_libraries(tf2_sensor_msgs_cras PRIVATE cloud string_utils PUBLIC ${catkin_LIBRARIES})
add_dependencies(tf2_sensor_msgs_cras ${catkin_EXPORTED_TARGETS})
target_compile_options(tf2_sensor_msgs_cras PRIVATE -march=native)

add_library(tf2_utils src/tf2_utils.cpp src/tf2_utils/interruptible_buffer.cpp)
target_link_libraries(tf2_utils thread_utils time_utils ${catkin_LIBRARIES})
add_dependencies(tf2_utils ${catkin_EXPORTED_TARGETS})

add_library(cras_math_utils INTERFACE)
target_sources(cras_math_utils INTERFACE 
  $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/math_utils.hpp>
  $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/math_utils/running_stats.hpp>
  $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/math_utils/running_stats_duration.hpp>)
target_link_libraries(cras_math_utils INTERFACE ${catkin_LIBRARIES})

add_library(set_utils INTERFACE)
target_sources(set_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/set_utils.hpp>)

add_library(type_utils src/type_utils.cpp)
target_link_libraries(type_utils PRIVATE string_utils)

add_library(xmlrpc_value_traits INTERFACE)
target_sources(xmlrpc_value_traits INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/xmlrpc_value_traits.hpp>)
target_link_libraries(xmlrpc_value_traits INTERFACE string_utils ${catkin_LIBRARIES})

add_library(xmlrpc_value_utils INTERFACE)
target_sources(xmlrpc_value_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/xmlrpc_value_utils.hpp>)
target_link_libraries(xmlrpc_value_utils INTERFACE xmlrpc_value_traits)

add_library(diag_utils src/diag_utils/topic_status_param.cpp src/diag_utils/topic_diagnostic.cpp src/diag_utils/duration_status_param.cpp src/diag_utils/duration_status.cpp)
target_link_libraries(diag_utils PUBLIC log_utils param_utils time_utils type_utils cras_diag_updater ${catkin_LIBRARIES})
add_dependencies(diag_utils ${catkin_EXPORTED_TARGETS})

add_library(filter_chain_nodelet INTERFACE)
target_sources(filter_chain_nodelet INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_chain_nodelet.hpp>)
target_link_libraries(filter_chain_nodelet INTERFACE diag_utils nodelet_utils cras_filter_chain)
add_dependencies(filter_chain_nodelet INTERFACE ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(cras_semaphore src/thread_utils/semaphore.cpp)

add_library(thread_utils src/thread_utils.cpp)
target_link_libraries(thread_utils PUBLIC cras_semaphore PRIVATE string_utils Threads::Threads)

add_library(urdf_utils src/urdf_utils.cpp)
target_link_libraries(urdf_utils Eigen3::Eigen)
target_include_directories(urdf_utils PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

add_library(cras_pool_allocator INTERFACE)
target_sources(cras_pool_allocator INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/pool_allocator.hpp>)
target_link_libraries(cras_pool_allocator INTERFACE ${Boost_LIBRARIES})

add_library(nodelet_manager_sharing_tf_buffer_lib src/nodelet_utils/nodelet_manager_sharing_tf_buffer.cpp)
target_link_libraries(nodelet_manager_sharing_tf_buffer_lib nodelet_utils cras_nodelet_manager ${catkin_LIBRARIES})
add_dependencies(nodelet_manager_sharing_tf_buffer_lib ${catkin_EXPORTED_TARGETS})

add_executable(nodelet_manager_sharing_tf_buffer nodes/nodelet_manager_sharing_tf_buffer.cpp)
target_link_libraries(nodelet_manager_sharing_tf_buffer nodelet_manager_sharing_tf_buffer_lib ${catkin_LIBRARIES})

add_library(cras_test_utils INTERFACE)
target_sources(cras_test_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/test_utils/preloading_class_loader.hpp>)
target_link_libraries(cras_test_utils INTERFACE type_utils)

add_library(cras_rate_limiter src/rate_limiter.cpp)
target_link_libraries(cras_rate_limiter time_utils ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

install(TARGETS cloud diag_utils cras_filter_base cras_filter_chain cras_diag_updater joy_repeater
  nodelet_manager_sharing_tf_buffer repeater string_utils
  tf2_sensor_msgs_cras tf2_utils time_utils

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_set_utils test/test_set_utils.cpp)
  target_link_libraries(test_set_utils set_utils)

  catkin_add_gtest(test_urdf_utils test/test_urdf_utils.cpp)
  target_link_libraries(test_urdf_utils urdf_utils)

  catkin_add_gtest(test_time_utils test/test_time_utils.cpp)
  target_link_libraries(test_time_utils time_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_string_utils test/test_string_utils.cpp)
  target_link_libraries(test_string_utils string_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_cloud test/test_cloud.cpp)
  target_link_libraries(test_cloud cloud ${catkin_LIBRARIES})

  catkin_add_gtest(test_tf2_sensor_msgs test/test_tf2_sensor_msgs.cpp)
  target_link_libraries(test_tf2_sensor_msgs tf2_sensor_msgs_cras ${catkin_LIBRARIES})
  
  catkin_add_gtest(test_xmlrpc_value_traits test/test_xmlrpc_value_traits.cpp)
  target_link_libraries(test_xmlrpc_value_traits xmlrpc_value_traits ${catkin_LIBRARIES})

  catkin_add_gtest(test_xmlrpc_value_utils test/test_xmlrpc_value_utils.cpp)
  target_link_libraries(test_xmlrpc_value_utils xmlrpc_value_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_filter_chain test/test_filter_chain.cpp)
  target_link_libraries(test_filter_chain cras_filter_chain ${catkin_LIBRARIES})

  catkin_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils cras_math_utils ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_chain_nodelet test/test_filter_chain_nodelet.test test/test_filter_chain_nodelet.cpp)
  target_link_libraries(test_filter_chain_nodelet filter_chain_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(test_diag_utils test/test_diag_utils.test test/test_diag_utils.cpp)
  target_link_libraries(test_diag_utils diag_utils ${catkin_LIBRARIES})
  target_compile_options(test_diag_utils PRIVATE -fno-var-tracking-assignments)  # speed up build

  catkin_add_gtest(test_log_utils test/test_log_utils.cpp)
  target_link_libraries(test_log_utils log_utils ${catkin_LIBRARIES})

  add_rostest_gtest(test_tf2_utils test/test_tf2_utils.test test/test_tf2_utils.cpp)
  target_link_libraries(test_tf2_utils tf2_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_thread_utils test/test_thread_utils.cpp)
  target_link_libraries(test_thread_utils thread_utils string_utils Threads::Threads)

  catkin_add_gtest(test_thread_name_updating_nodelet test/test_thread_name_updating_nodelet.cpp)
  target_link_libraries(test_thread_name_updating_nodelet nodelet_utils)

  catkin_add_gtest(test_type_utils test/test_type_utils.cpp)
  target_link_libraries(test_type_utils type_utils)

  catkin_add_gtest(test_pool_allocator test/test_pool_allocator.cpp)
  target_link_libraries(test_pool_allocator cras_pool_allocator)

  catkin_add_gtest(test_nodelet_manager test/test_nodelet_manager.cpp)
  target_link_libraries(test_nodelet_manager cras_nodelet_manager nodelet_manager_sharing_tf_buffer_lib ${catkin_LIBRARIES} cras_test_utils)

  catkin_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter cras_rate_limiter ${catkin_LIBRARIES})
  
  add_rostest_gtest(test_filter_base test/test_filter_base.test test/test_filter_base.cpp)
  target_link_libraries(test_filter_base cras_filter_base ${catkin_LIBRARIES})
  target_compile_options(test_filter_base PRIVATE -fno-var-tracking-assignments)  # speed up build
  
  add_rostest_gtest(test_param_utils test/test_param_utils.test test/test_param_utils.cpp)
  target_link_libraries(test_param_utils param_utils ${catkin_LIBRARIES})
  target_compile_options(test_param_utils PRIVATE -fno-var-tracking-assignments)  # speed up build

  # node_utils runs multiple tests from a single rostest launch file

  catkin_add_executable_with_gtest(test_node_utils_get_param test/test_node_utils_get_param.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_node_utils_get_param param_utils node_utils ${catkin_LIBRARIES})
  target_compile_options(test_node_utils_get_param PRIVATE -fno-var-tracking-assignments)  # speed up build
  add_dependencies(tests test_node_utils_get_param)
  
  catkin_add_executable_with_gtest(test_node_utils_diagnostics test/test_node_utils_diagnostics.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_node_utils_diagnostics param_utils node_utils ${catkin_LIBRARIES})
  target_compile_options(test_node_utils_diagnostics PRIVATE -fno-var-tracking-assignments)  # speed up build
  add_dependencies(tests test_node_utils_diagnostics)

  add_rostest(test/test_node_utils.test DEPENDENCIES test_node_utils_get_param test_node_utils_diagnostics)
  
  add_rostest_gtest(test_nodelet_get_param test/test_nodelet_get_param.test test/test_nodelet_get_param.cpp)
  target_link_libraries(test_nodelet_get_param param_utils nodelet_utils ${catkin_LIBRARIES})
  target_compile_options(test_nodelet_get_param PRIVATE -fno-var-tracking-assignments)  # speed up build

  # nodelet_utils runs multiple tests from a single rostest launch file
  catkin_add_executable_with_gtest(test_nodelet_aware_tf_buffer test/test_nodelet_aware_tf_buffer.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_nodelet_aware_tf_buffer nodelet_utils ${catkin_LIBRARIES})
  add_dependencies(tests test_nodelet_aware_tf_buffer)
  
  catkin_add_executable_with_gtest(test_stateful_nodelet test/test_stateful_nodelet.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_stateful_nodelet nodelet_utils ${catkin_LIBRARIES})
  add_dependencies(tests test_stateful_nodelet)
  
  catkin_add_executable_with_gtest(test_nodelet_log_macros test/test_nodelet_log_macros.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_nodelet_log_macros nodelet_utils ${catkin_LIBRARIES})
  add_dependencies(tests test_nodelet_log_macros)
  
  catkin_add_executable_with_gtest(test_nodelet_with_shared_tf_buffer test/test_nodelet_with_shared_tf_buffer.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_nodelet_with_shared_tf_buffer nodelet_utils ${catkin_LIBRARIES})
  add_dependencies(tests test_nodelet_with_shared_tf_buffer)
  
  catkin_add_executable_with_gtest(test_nodelet_diagnostics test/test_nodelet_diagnostics.cpp EXCLUDE_FROM_ALL)
  target_link_libraries(test_nodelet_diagnostics nodelet_utils diag_utils ${catkin_LIBRARIES})
  add_dependencies(tests test_nodelet_diagnostics)
  target_compile_options(test_nodelet_diagnostics PRIVATE -fno-var-tracking-assignments)  # speed up build
  
  add_rostest(test/test_nodelet_utils.test DEPENDENCIES test_nodelet_aware_tf_buffer test_stateful_nodelet test_nodelet_log_macros test_nodelet_with_shared_tf_buffer test_nodelet_diagnostics)
endif()
