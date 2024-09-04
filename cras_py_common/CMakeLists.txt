cmake_minimum_required(VERSION 3.10.2)
project(cras_py_common)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Run roslint on Python sources
  file(GLOB_RECURSE python_files src/*.py)
  roslint_python("${python_files}")

  roslint_add_test()

  add_rostest(test/test_log_utils.test)
  add_rostest(test/test_node_utils.test)
  add_rostest(test/test_param_utils.test)
  add_rostest(test/test_static_transform_broadcaster.test)
  add_rostest(test/test_topic_utils.test)

  catkin_add_nosetests(test/test_ctypes_utils.py)
  catkin_add_nosetests(test/test_geometry_utils.py)
  catkin_add_nosetests(test/test_message_utils.py)
  catkin_add_nosetests(test/test_python_utils.py)
  catkin_add_nosetests(test/test_string_utils.py)
  catkin_add_nosetests(test/test_test_utils.py)
  catkin_add_nosetests(test/test_time_utils.py)
endif()
