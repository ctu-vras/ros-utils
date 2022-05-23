# \file
# \brief Template for node made from a nodelet.
# \author Martin Pecka
# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# \brief Generate a node executable target from the given nodelet target.
# \param target: The nodelet target.
# \param include_file: Path to the include file declaring the nodelet class. It will be passed to #include .
# \param class_name: Full name of the nodelet class (including all namespaces).
# \param OUTPUT_NAME node_name[optional]: If specified, this name will be used as the name for the node executable.
#                                         If not specified, the node will be named ${target}_node.
# \param DEFAULT_NUM_THREADS num_threads[optional]: Default number of threads used for multi-threaded node handle.
#                                                   If not specified, 4 threads are the default value for ROS parameter
#                                                   ~num_worker_threads.
# param ANONYMOUS[options]: If specified, the default node name will be anonymous.
# \note The C++ code for the node is generated from template node_from_nodelet.cpp.in in this directory.
# \note The name of the generated CMake target will always be ${target}_node regardless of node_name setting (that only
#       affects the name of the built executable).
function(cras_node_from_nodelet target include_file class_name)
  @[if DEVELSPACE]@
    set(cras_cpp_common_CMAKE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/cmake")
  @[else]@
    set(cras_cpp_common_CMAKE_DIR "${cras_cpp_common_DIR}")
  @[end if]@

  cmake_parse_arguments(CRAS_NODE "ANONYMOUS" "OUTPUT_NAME;DEFAULT_NUM_THREADS" "" ${ARGN})

  set(NODELET_INCLUDE_FILE ${include_file})
  set(NODELET_CLASS ${class_name})
  set(NODE_TARGET_NAME ${target}_node)
  set(NODE_NAME ${NODE_TARGET_NAME})
  if(NOT "${CRAS_NODE_OUTPUT_NAME}" STREQUAL "")
    set(NODE_NAME ${CRAS_NODE_OUTPUT_NAME})
  endif()

  if(NOT DEFINED CRAS_NODE_DEFAULT_NUM_THREADS)
    set(CRAS_NODE_DEFAULT_NUM_THREADS 4)
  endif()
  
  message(STATUS "- Generating node ${NODE_NAME} from nodelet ${class_name}")
  
  find_package(roscpp REQUIRED)
  
  configure_file(${cras_cpp_common_CMAKE_DIR}/node_from_nodelet.cpp.in ${NODE_NAME}.cpp @@ONLY)
  add_executable(${NODE_TARGET_NAME} ${NODE_NAME}.cpp)
  target_link_libraries(${NODE_TARGET_NAME} ${target} ${roscpp_LIBRARIES})
  set_target_properties(${NODE_TARGET_NAME} PROPERTIES OUTPUT_NAME ${NODE_NAME} PREFIX "")
  
  install(TARGETS ${NODE_TARGET_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endfunction()