cmake_minimum_required(VERSION 3.7.2)
project(cras_py_common)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment and with empty rosdep cache,
  # so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_HOME})
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{ROS_HOME}")
  else()
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{HOME}/.ros")
  endif()

  #catkin_lint: ignore_once env_var
  if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
    roslint_custom(catkin_lint "-W2" .)
  endif()

  # Run roslint on Python sources
  file(GLOB_RECURSE python_files src/*.py)
  roslint_python("${python_files}")

  roslint_add_test()

  add_rostest(test/test_log_utils.test)
  add_rostest(test/test_node_utils.test)
  add_rostest(test/test_param_utils.test)
  add_rostest(test/test_static_transform_broadcaster.test)
  add_rostest(test/test_topic_utils.test)

  catkin_add_nosetests(test/test_geometry_utils.py)
  catkin_add_nosetests(test/test_message_utils.py)
  catkin_add_nosetests(test/test_string_utils.py)
  catkin_add_nosetests(test/test_test_utils.py)
  catkin_add_nosetests(test/test_time_utils.py)
endif()
