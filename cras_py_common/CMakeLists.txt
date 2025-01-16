cmake_minimum_required(VERSION 3.10.2)
project(cras_py_common)

find_package(catkin REQUIRED COMPONENTS sensor_msgs)

catkin_python_setup()

catkin_package()

add_library(sensor_msgs_py src/distortion_models.cpp src/image_encodings.cpp)
target_include_directories(sensor_msgs_py PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(sensor_msgs_py ${catkin_LIBRARIES})

install(TARGETS sensor_msgs_py
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Run roslint on Python sources
  file(GLOB_RECURSE python_files src/*.py test/*.py)
  roslint_python("${python_files}")

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_FILES src/*.cpp)
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  roslint_add_test()

  add_rostest(test/test_log_utils.test)
  add_rostest(test/test_node_utils.test)
  add_rostest(test/test_param_utils.test)
  add_rostest(test/test_static_transform_broadcaster.test)
  add_rostest(test/test_topic_utils.test)

  catkin_add_nosetests(test/test_ctypes_utils.py)
  catkin_add_nosetests(test/test_geometry_utils.py)
  catkin_add_nosetests(test/test_image_encodings.py)
  catkin_add_nosetests(test/test_message_utils.py)
  catkin_add_nosetests(test/test_python_utils.py)
  catkin_add_nosetests(test/test_string_utils.py)
  catkin_add_nosetests(test/test_test_utils.py)
  catkin_add_nosetests(test/test_time_utils.py)
endif()
