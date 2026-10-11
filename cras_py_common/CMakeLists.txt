# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(cras_py_common)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(cras)

#add_library(sensor_msgs_py src/distortion_models.cpp src/image_encodings.cpp)
#target_include_directories(sensor_msgs_py PRIVATE ${catkin_INCLUDE_DIRS})
#target_link_libraries(sensor_msgs_py ${catkin_LIBRARIES})

#install(TARGETS sensor_msgs_py
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_py(EXCLUDE
    cras/impl/__init__.py
    cras/impl/param_utils_geometry_msgs.py
    cras/impl/param_utils_numpy.py
    cras/impl/param_utils_rospy.py
    cras/distortion_models.py
    cras/geometry_utils.py
    cras/image_encodings.py
    cras/log_utils.py
    cras/message_utils.py
    cras/param_utils.py
    cras/plugin_utils.py
    cras/python_utils.py
    cras/static_transform_broadcaster.py
    cras/string_utils.py
    cras/test_utils.py
    cras/time_utils.py
    cras/topic_utils.py
    doc/conf.py
    test/publish_static_tfs.py
    test/test_ctypes_utils.py
    test/test_distortion_models.py
    test/test_geometry_utils.py
    test/test_image_encodings.py
    test/test_log_utils.py
    test/test_message_utils.py
    test/test_param_utils.py
    test/test_plugin_utils.py
    test/test_python_utils.py
    test/test_static_tf_broadcaster.py
    test/test_string_utils.py
    test/test_test_utils.py
    test/test_time_utils.py
    test/test_topic_utils.py
  )

  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_ctypes_utils.py
    test/test_node_utils.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()
