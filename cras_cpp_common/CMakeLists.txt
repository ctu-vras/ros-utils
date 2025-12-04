cmake_minimum_required(VERSION 3.20.2)
cmake_policy(SET CMP0022 NEW)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)  # if boost installs Find module, use it
endif()

project(cras_cpp_common)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

find_package(Boost REQUIRED)

# find pthread and provide it as Threads::Threads imported target
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
try_compile(HAS_DESIGNATED_INITIALIZERS
  ${CMAKE_BINARY_DIR}/designated_initializers ${CMAKE_CURRENT_SOURCE_DIR}/cmake/designated_initializers_try_compile.cpp
  CXX_STANDARD 20)

set(CCC_INCLUDE_BASE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")

add_library(cras_expected INTERFACE)
target_sources(cras_expected INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/expected.hpp>)

add_library(cras_small_map INTERFACE)
target_sources(cras_small_map INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/small_map.hpp>)

add_library(cras_c_api SHARED src/c_api.cpp)
target_include_directories(cras_c_api PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_library(cras_semaphore SHARED src/thread_utils/semaphore.cpp)
target_include_directories(cras_semaphore PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_library(cras_set_utils INTERFACE)
target_sources(cras_set_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/set_utils.hpp>)

add_library(cras_type_traits INTERFACE)
target_sources(cras_type_traits INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/type_utils/string_traits.hpp>)

add_library(cras_literal_sz INTERFACE)
target_sources(cras_literal_sz INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/type_utils/literal_sz.hpp>)

#add_library(cras_urdf_utils src/urdf_utils.cpp)
##catkin_lint: ignore_once external_interface_path
#target_include_directories(cras_urdf_utils PUBLIC ${urdfdom_headers_INCLUDE_DIRS})
#target_link_libraries(cras_urdf_utils
#  PUBLIC Eigen3::Eigen)

#add_library(cras_message_utils INTERFACE)
#target_sources(cras_message_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/message_utils.hpp>)
#target_link_libraries(cras_message_utils
#  INTERFACE ${catkin_LIBRARIES})
#add_dependencies(cras_message_utils INTERFACE ${catkin_EXPORTED_TARGETS})

#add_library(cras_cloud src/cloud.cpp)
#add_dependencies(cras_cloud ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_cloud
#  PUBLIC ${catkin_LIBRARIES})

#add_library(cras_diag_updater src/diag_utils/updater.cpp)
#add_dependencies(cras_diag_updater ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_diag_updater
#  PUBLIC ${catkin_LIBRARIES})

#add_library(cras_nodelet_manager
#  src/nodelet_utils/loader_ros.cpp
#  src/nodelet_utils/nodelet_manager.cpp)
#add_dependencies(cras_nodelet_manager ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_nodelet_manager
#  PUBLIC ${catkin_LIBRARIES} Boost::boost Boost::thread)

add_library(cras_math_utils INTERFACE)
target_sources(cras_math_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/math_utils.hpp>)
target_sources(cras_math_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/math_utils/running_stats.hpp>)

add_library(cras_time_utils SHARED
  src/time_utils.cpp
#  src/time_utils/interruptible_sleep_interface.cpp
)
target_include_directories(cras_time_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(cras_time_utils
  PUBLIC rclcpp::rclcpp
  PRIVATE Boost::boost
  # cras_semaphore
)

#add_library(cras_rate_limiter src/rate_limiter.cpp)
#target_link_libraries(cras_rate_limiter
#  PUBLIC ${catkin_LIBRARIES}
#  PRIVATE cras_time_utils)

#add_library(cras_tf2_utils
#  src/tf2_utils.cpp
#  src/tf2_utils/interruptible_buffer.cpp)
#target_sources(cras_tf2_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/tf2_utils/message_filter.hpp>)
#add_dependencies(cras_tf2_utils ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_tf2_utils
#  PUBLIC cras_log_utils cras_thread_utils cras_time_utils ${catkin_LIBRARIES})

add_library(cras_string_utils SHARED
  src/string_utils.cpp
  #src/string_utils/ros.cpp
)
target_include_directories(cras_string_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(cras_string_utils PUBLIC cras_type_traits)

add_library(cras_thread_utils SHARED src/thread_utils.cpp)
target_include_directories(cras_thread_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(cras_thread_utils
  PRIVATE cras_string_utils Threads::Threads)

#add_library(cras_log_utils src/log_utils.cpp src/log_utils/memory.cpp src/log_utils/node.cpp src/log_utils/nodelet.cpp)
#target_link_libraries(cras_log_utils
#  PUBLIC cras_string_utils ${catkin_LIBRARIES} PRIVATE cras_time_utils)

# Detect compiler SIMD support. We'll need it for cras_tf2_sensor_msgs target.
include(CheckCXXCompilerFlag)
unset(COMPILER_SUPPORTS_MARCH_X86_V3 CACHE)
unset(COMPILER_SUPPORTS_MARCH_X86_V2 CACHE)
unset(COMPILER_SUPPORTS_MARCH_ARMV8 CACHE)
unset(COMPILER_SUPPORTS_MARCH_ARMV7 CACHE)
unset(COMPILER_SUPPORTS_MARCH_NATIVE CACHE)
set(X86_V2_FLAGS -msse4 -msse3)
set(X86_V3_FLAGS -mavx2 -mavx -mfma)
list(APPEND X86_V3_FLAGS ${X86_V2_FLAGS})
check_cxx_compiler_flag("-march=x86-64 ${X86_V3_FLAGS}" COMPILER_SUPPORTS_MARCH_X86_V3)
check_cxx_compiler_flag("-march=x86-64 ${X86_V2_FLAGS}" COMPILER_SUPPORTS_MARCH_X86_V2)
check_cxx_compiler_flag("-march=armv8-a" COMPILER_SUPPORTS_MARCH_ARMV8)
check_cxx_compiler_flag("-march=armv7-a+neon" COMPILER_SUPPORTS_MARCH_ARMV7)
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)

#add_library(cras_tf2_sensor_msgs src/tf2_sensor_msgs.cpp)
#add_dependencies(cras_tf2_sensor_msgs ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_tf2_sensor_msgs
#  PUBLIC ${catkin_LIBRARIES}
#  PRIVATE cras_cloud cras_string_utils Eigen3::Eigen)
## The pointcloud processing loop really needs SIMD. We enable it conservatively.
## ROS buildfarm should pick up x86-64-v3 for x86 builds, armv7 for armhf builds and armv8 for arm64 builds.
#if (COMPILER_SUPPORTS_MARCH_X86_V3)
#  target_compile_options(cras_tf2_sensor_msgs PRIVATE -march=x86-64 ${X86_V3_FLAGS})
#  message("cras_tf2_sensor_msgs uses x86-64-v3 SIMD")
#elseif (COMPILER_SUPPORTS_MARCH_X86_V2)
#  target_compile_options(cras_tf2_sensor_msgs PRIVATE -march=x86-64 ${X86_V2_FLAGS})
#  message("cras_tf2_sensor_msgs uses x86-64-v2 SIMD")
#elseif (COMPILER_SUPPORTS_MARCH_ARMV7)
#  target_compile_options(cras_tf2_sensor_msgs PRIVATE -march=armv7-a+neon)
#  message("cras_tf2_sensor_msgs uses armv7-a+simd SIMD")
#elseif (COMPILER_SUPPORTS_MARCH_ARMV8)
#  target_compile_options(cras_tf2_sensor_msgs PRIVATE -march=armv8-a)
#  message("cras_tf2_sensor_msgs uses armv8-a SIMD")
#elseif (COMPILER_SUPPORTS_MARCH_NATIVE)
#  target_compile_options(cras_tf2_sensor_msgs PRIVATE -march=native)
#  message("cras_tf2_sensor_msgs uses native SIMD")
#endif()

add_library(cras_type_utils SHARED src/type_utils.cpp)
target_include_directories(cras_type_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(cras_type_utils
  PUBLIC cras_type_traits cras_literal_sz
  PRIVATE cras_string_utils)

#add_library(cras_test_utils INTERFACE)
#target_sources(cras_test_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/test_utils/preloading_class_loader.hpp>)
#target_link_libraries(cras_test_utils
#  INTERFACE cras_type_utils ${catkin_LIBRARIES})

add_library(cras_param_utils INTERFACE)
target_sources(cras_param_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/param_utils.hpp>)
target_link_libraries(cras_param_utils
  INTERFACE rclcpp::rclcpp)

add_library(cras_qos_utils SHARED src/qos.cpp)
target_include_directories(cras_qos_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(cras_qos_utils PUBLIC rclcpp::rclcpp)

#add_library(cras_filter_base INTERFACE)
#target_sources(cras_filter_base INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_utils/filter_base.hpp>)
#target_link_libraries(cras_filter_base
#  INTERFACE cras_log_utils cras_param_utils cras_string_utils ${catkin_LIBRARIES})

#add_library(cras_filter_chain INTERFACE)
#target_sources(cras_filter_chain INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_utils/filter_chain.hpp>)
#target_link_libraries(cras_filter_chain
#  INTERFACE cras_filter_base cras_log_utils cras_string_utils cras_type_utils ${catkin_LIBRARIES} Boost::boost)

#add_library(cras_diag_utils
#  src/diag_utils/duration_status.cpp
#  src/diag_utils/duration_status_param.cpp
#  src/diag_utils/offline_diag_updater.cpp
#  src/diag_utils/topic_diagnostic.cpp
#  src/diag_utils/topic_status_param.cpp)
#add_dependencies(cras_diag_utils ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_diag_utils
#  PUBLIC cras_diag_updater cras_log_utils cras_math_utils cras_message_utils cras_optional
#    cras_param_utils cras_time_utils cras_type_utils ${catkin_LIBRARIES} Boost::boost)

#add_library(cras_resettable src/resettable.cpp)
#add_dependencies(cras_resettable ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_resettable PUBLIC cras_log_utils ${catkin_LIBRARIES})
#target_link_libraries(cras_resettable PRIVATE cras_functional cras_param_utils cras_optional cras_time_utils)

#add_library(cras_node_utils
#  src/node_utils/node_handle_with_diagnostics.cpp
#  src/node_utils/node_with_optional_master.cpp
#  src/node_utils/param_helper.cpp)
#target_sources(cras_node_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/node_utils.hpp>)
#add_dependencies(cras_node_utils ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_node_utils
#  PUBLIC cras_diag_utils cras_log_utils cras_message_utils cras_optional cras_param_utils cras_string_utils
#    ${catkin_LIBRARIES} Boost::boost)

#add_library(cras_nodelet_utils
#  src/nodelet_utils/nodelet_aware_tf_buffer.cpp
#  src/nodelet_utils/stateful_nodelet.cpp)
#target_sources(cras_nodelet_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/nodelet_utils.hpp>)
#target_sources(cras_nodelet_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/nodelet_utils/log_macros.h>)
#target_sources(
#  cras_nodelet_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/nodelet_utils/nodelet_with_diagnostics.hpp>)
#target_sources(cras_nodelet_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/nodelet_utils/param_helper.hpp>)
#target_sources(
#  cras_nodelet_utils INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/nodelet_utils/thread_name_updating_nodelet.hpp>)
#add_dependencies(cras_nodelet_utils ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_nodelet_utils
#  PUBLIC cras_diag_updater cras_diag_utils cras_log_utils cras_message_utils cras_node_utils cras_param_utils
#    cras_resettable cras_string_utils cras_thread_utils cras_time_utils cras_tf2_utils ${catkin_LIBRARIES}
#    Boost::boost)

#add_library(cras_filter_chain_nodelet INTERFACE)
#target_sources(cras_filter_chain_nodelet INTERFACE $<BUILD_INTERFACE:${CCC_INCLUDE_BASE}/filter_chain_nodelet.hpp>)
#add_dependencies(cras_filter_chain_nodelet INTERFACE ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(cras_filter_chain_nodelet
#  INTERFACE cras_diag_utils cras_filter_chain cras_functional cras_nodelet_utils cras_string_utils
#    ${catkin_LIBRARIES} Boost::thread)

#add_library(cras_nodelet_manager_sharing_tf_buffer src/nodelet_utils/nodelet_manager_sharing_tf_buffer.cpp)
#add_dependencies(cras_nodelet_manager_sharing_tf_buffer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cras_nodelet_manager_sharing_tf_buffer
#  PUBLIC cras_nodelet_manager cras_nodelet_utils cras_resettable ${catkin_LIBRARIES} Boost::boost)

#add_executable(nodelet_manager_sharing_tf_buffer nodes/nodelet_manager_sharing_tf_buffer.cpp)
#add_dependencies(nodelet_manager_sharing_tf_buffer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(nodelet_manager_sharing_tf_buffer
#  PUBLIC cras_nodelet_manager_sharing_tf_buffer ${catkin_LIBRARIES})

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}/
)

# Install libraries
install(TARGETS
  cras_c_api
#  cras_cloud
#  cras_diag_updater
#  cras_diag_utils
  cras_expected
#  cras_filter_base
#  cras_filter_chain
#  cras_filter_chain_nodelet
  cras_literal_sz
#  cras_log_utils
  cras_math_utils
#  cras_message_utils
#  cras_node_utils
#  cras_nodelet_manager
#  cras_nodelet_manager_sharing_tf_buffer
#  cras_nodelet_utils
  cras_param_utils
  cras_qos_utils
#  cras_rate_limiter
#  cras_resettable
  cras_semaphore
  cras_set_utils
  cras_string_utils
#  cras_test_utils
#  cras_tf2_sensor_msgs
#  cras_tf2_utils
  cras_thread_utils
  cras_time_utils
  cras_type_traits
  cras_type_utils
#  cras_urdf_utils

  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
#install(TARGETS
#  nodelet_manager_sharing_tf_buffer
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

if(BUILD_TESTING)
  # cras_lint - checks formatting and some other rules for C++ files
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  set(cras_lint_FILE_EXCLUDE
    include/${PROJECT_NAME}/cloud/impl/*
    include/${PROJECT_NAME}/diag_utils/*
    include/${PROJECT_NAME}/diag_utils/deprecated/*
    include/${PROJECT_NAME}/diag_utils/impl/*
    include/${PROJECT_NAME}/diagnostics/*
    include/${PROJECT_NAME}/filter_utils/*
    include/${PROJECT_NAME}/filter_utils/impl/*
    include/${PROJECT_NAME}/log_utils/*
    include/${PROJECT_NAME}/log_utils/deprecated/*
    include/${PROJECT_NAME}/node_utils/*
    include/${PROJECT_NAME}/nodelet_utils/*
    include/${PROJECT_NAME}/nodelet_utils/impl/*
    include/${PROJECT_NAME}/string_utils/ros.hpp
    include/${PROJECT_NAME}/string_utils/tf2.hpp
    include/${PROJECT_NAME}/test_utils/*
    include/${PROJECT_NAME}/tf2_utils/*
    include/${PROJECT_NAME}/cloud.hpp
    include/${PROJECT_NAME}/diag_utils.hpp
    include/${PROJECT_NAME}/filter_chain_nodelet.hpp
    include/${PROJECT_NAME}/filter_utils.hpp
    include/${PROJECT_NAME}/log_utils.h
    include/${PROJECT_NAME}/message_utils.hpp
    include/${PROJECT_NAME}/node_utils.hpp
    include/${PROJECT_NAME}/nodelet_utils.hpp
    include/${PROJECT_NAME}/rate_limiter.h
    include/${PROJECT_NAME}/resettable.h
    include/${PROJECT_NAME}/tf2_sensor_msgs.h
    include/${PROJECT_NAME}/tf2_utils.hpp
    include/${PROJECT_NAME}/urdf_utils.h
    nodes/*
    src/diag_utils/*
    src/log_utils/*
    src/node_utils/*
    src/nodelet_utils/*
    src/string_utils/ros.cpp
    src/tf2_utils/interruptible_buffer.cpp
    src/time_utils/*
    src/cloud.cpp
    src/log_utils.cpp
    src/rate_limiter.cpp
    src/resettable.cpp
    src/tf2_sensor_msgs.cpp
    src/tf2_utils.cpp
    src/time_utils/interruptible_sleep_interface.cpp
    src/urdf_utils.cpp
    test/*
  )
  set(ament_cmake_cpplint_EXTRA_FILTERS "-build/include_order")
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
#
#  # GTEST tests (no rosmaster required)
#
#  catkin_add_gtest(test_c_api test/test_c_api.cpp)
#  target_link_libraries(test_c_api cras_c_api ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_cloud test/test_cloud.cpp)
#  target_link_libraries(test_cras_cloud cras_cloud ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_filter_chain test/test_filter_chain.cpp)
#  target_link_libraries(test_cras_filter_chain cras_filter_chain ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_log_utils test/test_log_utils.cpp)
#  target_link_libraries(test_cras_log_utils cras_log_utils ${catkin_LIBRARIES})
#  target_compile_options(test_cras_log_utils PRIVATE -save-temps)
#
#  catkin_add_gtest(test_cras_math_utils test/test_math_utils.cpp)
#  target_link_libraries(test_cras_math_utils cras_math_utils ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_pool_allocator test/test_pool_allocator.cpp)
#  target_link_libraries(test_pool_allocator cras_pool_allocator)
#
#  catkin_add_gtest(test_cras_rate_limiter test/test_rate_limiter.cpp)
#  target_link_libraries(test_cras_rate_limiter cras_rate_limiter ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_set_utils test/test_set_utils.cpp)
#  target_link_libraries(test_cras_set_utils cras_set_utils)
#
#  catkin_add_gtest(test_cras_string_utils test/test_string_utils.cpp)
#  target_link_libraries(test_cras_string_utils cras_string_utils ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_tf2_sensor_msgs test/test_tf2_sensor_msgs.cpp)
#  target_link_libraries(test_cras_tf2_sensor_msgs cras_tf2_sensor_msgs ${catkin_LIBRARIES})
#
#  add_rostest_gtest(test_cras_tf2_utils test/test_tf2_utils.test test/test_tf2_utils.cpp)
#  target_link_libraries(test_cras_tf2_utils cras_tf2_utils ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_thread_name_updating_nodelet test/test_thread_name_updating_nodelet.cpp)
#  target_link_libraries(test_cras_thread_name_updating_nodelet cras_nodelet_utils)
#
#  catkin_add_gtest(test_cras_small_map test/test_small_map.cpp)
#
#  catkin_add_gtest(test_cras_thread_utils test/test_thread_utils.cpp)
#  target_link_libraries(test_cras_thread_utils cras_semaphore cras_string_utils cras_thread_utils Threads::Threads)
#
  ament_add_gtest(test_cras_time_utils test/test_time_utils.cpp)
  target_link_libraries(test_cras_time_utils cras_time_utils)
#
#  catkin_add_gtest(test_cras_type_utils test/test_type_utils.cpp)
#  target_link_libraries(test_cras_type_utils cras_type_utils)
#
#  catkin_add_gtest(test_cras_urdf_utils test/test_urdf_utils.cpp)
#  target_link_libraries(test_cras_urdf_utils cras_urdf_utils)
#
#  catkin_add_gtest(test_cras_xmlrpc_value_traits test/test_xmlrpc_value_traits.cpp)
#  target_link_libraries(test_cras_xmlrpc_value_traits xmlrpc_value_traits ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_xmlrpc_value_utils test/test_xmlrpc_value_utils.cpp)
#  target_link_libraries(test_cras_xmlrpc_value_utils xmlrpc_value_utils ${catkin_LIBRARIES})
#
#  catkin_add_gtest(test_cras_node_without_master test/test_node_utils_node_without_master.cpp)
#  target_link_libraries(test_cras_node_without_master cras_node_utils cras_param_utils ${catkin_LIBRARIES})
#
#  # ROSTEST tests (need rosmaster)
#
#  add_rostest_gtest(
#    test_cras_filter_chain_nodelet test/test_filter_chain_nodelet.test test/test_filter_chain_nodelet.cpp)
#  target_link_libraries(test_cras_filter_chain_nodelet cras_filter_chain_nodelet ${catkin_LIBRARIES})
#  roslaunch_add_file_check(test/test_filter_chain_nodelet.test USE_TEST_DEPENDENCIES)
#
#  add_rostest_gtest(test_cras_nodelet_manager test/test_nodelet_manager.test test/test_nodelet_manager.cpp)
#  target_link_libraries(test_cras_nodelet_manager
#    cras_nodelet_manager cras_nodelet_manager_sharing_tf_buffer cras_test_utils ${catkin_LIBRARIES})
#  roslaunch_add_file_check(test/test_nodelet_manager.test USE_TEST_DEPENDENCIES)
#
#  add_rostest_gtest(test_cras_diag_utils test/test_diag_utils.test test/test_diag_utils.cpp)
#  target_link_libraries(test_cras_diag_utils cras_diag_utils ${catkin_LIBRARIES})
#  target_compile_options(test_cras_diag_utils PRIVATE -fno-var-tracking-assignments)  # speed up build
#  if (HAS_DESIGNATED_INITIALIZERS)
#    target_compile_definitions(test_cras_diag_utils PRIVATE HAS_DESIGNATED_INITIALIZERS=1)
#  endif()
#  roslaunch_add_file_check(test/test_diag_utils.test USE_TEST_DEPENDENCIES)
#
#  add_rostest_gtest(test_cras_filter_base test/test_filter_base.test test/test_filter_base.cpp)
#  target_link_libraries(test_cras_filter_base cras_filter_base ${catkin_LIBRARIES})
#  target_compile_options(test_cras_filter_base PRIVATE -fno-var-tracking-assignments)  # speed up build
#  # roslaunch_add_file_check(test/test_filter_base.test USE_TEST_DEPENDENCIES)  # Disabled due to conflicting params
#
#  add_rostest_gtest(test_cras_param_utils test/test_param_utils.test test/test_param_utils.cpp)
#  target_link_libraries(test_cras_param_utils cras_param_utils ${catkin_LIBRARIES})
#  target_compile_options(test_cras_param_utils PRIVATE -fno-var-tracking-assignments)  # speed up build
#  if (HAS_DESIGNATED_INITIALIZERS)
#    target_compile_definitions(test_cras_param_utils PRIVATE HAS_DESIGNATED_INITIALIZERS=1)
#  endif()
#  # roslaunch_add_file_check(test/test_param_utils.test USE_TEST_DEPENDENCIES)  # Disabled due to conflicting params
#
#  add_rostest_gtest(test_cras_nodelet_get_param test/test_nodelet_get_param.test test/test_nodelet_get_param.cpp)
#  target_link_libraries(test_cras_nodelet_get_param cras_nodelet_utils cras_param_utils ${catkin_LIBRARIES})
#  target_compile_options(test_cras_nodelet_get_param PRIVATE -fno-var-tracking-assignments)  # speed up build
#  # Disabled due to conflicting params
#  # roslaunch_add_file_check(test/test_nodelet_get_param.test USE_TEST_DEPENDENCIES)
#
#  # node_utils runs multiple tests from a single rostest launch file
#
#  catkin_add_executable_with_gtest(test_cras_node_utils_get_param test/test_node_utils_get_param.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_node_utils_get_param cras_node_utils cras_param_utils ${catkin_LIBRARIES})
#  target_compile_options(test_cras_node_utils_get_param PRIVATE -fno-var-tracking-assignments)  # speed up build
#  add_dependencies(tests test_cras_node_utils_get_param)
#
#  catkin_add_executable_with_gtest(
#    test_cras_node_utils_diagnostics test/test_node_utils_diagnostics.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_node_utils_diagnostics cras_node_utils cras_param_utils ${catkin_LIBRARIES})
#  target_compile_options(test_cras_node_utils_diagnostics PRIVATE -fno-var-tracking-assignments)  # speed up build
#  add_dependencies(tests test_cras_node_utils_diagnostics)
#  if (HAS_DESIGNATED_INITIALIZERS)
#    target_compile_definitions(test_cras_node_utils_diagnostics PRIVATE HAS_DESIGNATED_INITIALIZERS=1)
#  endif()
#
#  catkin_add_executable_with_gtest(
#    test_cras_node_with_master test/test_node_utils_node_with_master.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_node_with_master cras_node_utils cras_param_utils ${catkin_LIBRARIES})
#  add_dependencies(tests test_cras_node_with_master)
#
#  add_rostest(test/test_node_utils.test DEPENDENCIES
#    test_cras_node_utils_diagnostics
#    test_cras_node_utils_get_param
#    test_cras_node_with_master)
#  # roslaunch_add_file_check(test/test_node_utils.test USE_TEST_DEPENDENCIES)  # Disabled due to conflicting params
#
#  # nodelet_utils runs multiple tests from a single rostest launch file
#
#  catkin_add_executable_with_gtest(
#    test_cras_nodelet_aware_tf_buffer test/test_nodelet_aware_tf_buffer.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_nodelet_aware_tf_buffer cras_nodelet_utils ${catkin_LIBRARIES})
#  add_dependencies(tests test_cras_nodelet_aware_tf_buffer)
#
#  catkin_add_executable_with_gtest(test_cras_stateful_nodelet test/test_stateful_nodelet.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_stateful_nodelet cras_nodelet_utils ${catkin_LIBRARIES})
#  add_dependencies(tests test_cras_stateful_nodelet)
#
#  catkin_add_executable_with_gtest(test_cras_nodelet_log_macros test/test_nodelet_log_macros.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_nodelet_log_macros cras_nodelet_utils ${catkin_LIBRARIES})
#  add_dependencies(tests test_cras_nodelet_log_macros)
#
#  catkin_add_executable_with_gtest(
#    test_cras_nodelet_with_shared_tf_buffer test/test_nodelet_with_shared_tf_buffer.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_nodelet_with_shared_tf_buffer cras_nodelet_utils ${catkin_LIBRARIES})
#  add_dependencies(tests test_cras_nodelet_with_shared_tf_buffer)
#
#  catkin_add_executable_with_gtest(test_cras_nodelet_diagnostics test/test_nodelet_diagnostics.cpp EXCLUDE_FROM_ALL)
#  target_link_libraries(test_cras_nodelet_diagnostics cras_diag_utils cras_nodelet_utils ${catkin_LIBRARIES})
#  add_dependencies(tests test_cras_nodelet_diagnostics)
#  target_compile_options(test_cras_nodelet_diagnostics PRIVATE -fno-var-tracking-assignments)  # speed up build
#  if (HAS_DESIGNATED_INITIALIZERS)
#    target_compile_definitions(test_cras_nodelet_diagnostics PRIVATE HAS_DESIGNATED_INITIALIZERS=1)
#  endif()
#
#  add_rostest(test/test_nodelet_utils.test DEPENDENCIES
#    test_cras_nodelet_aware_tf_buffer
#    test_cras_nodelet_diagnostics
#    test_cras_nodelet_log_macros
#    test_cras_nodelet_with_shared_tf_buffer
#    test_cras_stateful_nodelet)
#  roslaunch_add_file_check(test/test_nodelet_utils.test USE_TEST_DEPENDENCIES)
#
#  add_rostest_gtest(test_resettable test/test_resettable.test test/test_resettable.cpp)
#  target_link_libraries(test_resettable cras_resettable cras_test_utils ${catkin_LIBRARIES})
#  roslaunch_add_file_check(test/test_resettable.test USE_TEST_DEPENDENCIES)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_libraries(cras_c_api)
#ament_export_libraries(cras_cloud)
#ament_export_libraries(cras_diag_updater)
#ament_export_libraries(cras_diag_utils)
#ament_export_libraries(cras_log_utils)
#ament_export_libraries(cras_node_utils)
#ament_export_libraries(cras_nodelet_manager)
#ament_export_libraries(cras_nodelet_manager_sharing_tf_buffer)
#ament_export_libraries(cras_nodelet_utils)
ament_export_libraries(cras_qos_utils)
#ament_export_libraries(cras_rate_limiter)
#ament_export_libraries(cras_resettable)
ament_export_libraries(cras_semaphore)
ament_export_libraries(cras_string_utils)
#ament_export_libraries(cras_tf2_sensor_msgs)
#ament_export_libraries(cras_tf2_utils)
ament_export_libraries(cras_thread_utils)
ament_export_libraries(cras_time_utils)
ament_export_libraries(cras_type_utils)
#ament_export_libraries(cras_urdf_utils)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rcl)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(Threads)

ament_package()